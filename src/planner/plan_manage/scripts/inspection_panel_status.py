#!/usr/bin/env python3
"""Relay measured mission and planning status to the local panel."""
import json
import math
import os
import threading
import time
from collections import deque
from pathlib import Path


class PlanningMetrics:
    def __init__(self):
        self.records = deque(maxlen=100)
        self.count = self.failed = self.missed = self.last_seq = 0
        self.total = self.maximum = 0.0
        self.response = None

    def add(self, data):
        if data.get('stage') not in ('global', 'local_initial', 'local_replan'):
            raise ValueError('Unknown planning stage')
        for key in ('seq', 'goal_id'):
            if type(data.get(key)) is not int or data[key] < 1:
                raise ValueError('Invalid sequence')
        if type(data.get('success')) is not bool:
            raise ValueError('Invalid result')
        for key in ('time', 'duration_ms', 'goal_response_ms'):
            value = data.get(key)
            if key == 'goal_response_ms' and value is None:
                continue
            if type(value) not in (int, float) or not math.isfinite(value) or value < 0:
                raise ValueError('Invalid duration or time')
        if data['seq'] <= self.last_seq:
            return False
        self.missed += data['seq'] - self.last_seq - 1
        self.last_seq = data['seq']
        self.count += 1
        self.failed += not data['success']
        self.total += data['duration_ms']
        self.maximum = max(self.maximum, data['duration_ms'])
        if data['goal_response_ms'] is not None:
            self.response = data['goal_response_ms']
        self.records.append(dict(data))
        return True

    def snapshot(self):
        return dict(count=self.count, failed=self.failed, missed=self.missed,
                    average_ms=self.total / self.count if self.count else None,
                    maximum_ms=self.maximum if self.count else None,
                    goal_response_ms=self.response, records=list(self.records))


def main():
    import rospy
    from std_msgs.msg import String
    rospy.init_node('inspection_panel_status')
    destination = Path(rospy.get_param('~status_file'))
    destination.parent.mkdir(parents=True, exist_ok=True)
    metrics = PlanningMetrics()
    mission = {}
    lock = threading.RLock()
    history = (destination.parent / 'planning-metrics.jsonl').open('w', encoding='utf-8')

    def receive(msg):
        nonlocal mission
        with lock:
            mission = json.loads(msg.data)
            mission['received_at'] = time.time()

    def planning(msg):
        try:
            data = json.loads(msg.data)
            with lock:
                if metrics.add(data):
                    history.write(json.dumps(data, allow_nan=False) + '\n')
                    history.flush()
        except (ValueError, TypeError, KeyError) as error:
            rospy.logwarn_throttle(5, 'Invalid planning metric: %s', str(error))

    def flush(_):
        with lock:
            if not mission:
                return
            data = dict(mission, planning=metrics.snapshot())
            temporary = destination.with_suffix('.tmp')
            temporary.write_text(json.dumps(data, allow_nan=False), encoding='utf-8')
            os.replace(temporary, destination)

    subscriber = rospy.Subscriber('/inspection/status', String, receive, queue_size=1)
    metric_subscriber = rospy.Subscriber('/inspection/planning_metrics', String, planning, queue_size=1000)
    timer = rospy.Timer(rospy.Duration(0.2), flush)
    rospy.spin()
    timer.shutdown()
    history.close()


if __name__ == '__main__':
    main()
