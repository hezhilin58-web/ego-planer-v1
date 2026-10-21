#!/usr/bin/env python3
"""Sequential inspection supervisor; EGO-Planner retains trajectory ownership."""
import json
import math
import os
import threading
import time
from pathlib import Path as FilePath

import rospy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path
from quadrotor_msgs.msg import PositionCommand
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger, TriggerResponse
from visualization_msgs.msg import Marker, MarkerArray


def norm(values):
    return math.sqrt(sum(x * x for x in values))


def angle_error(target, actual):
    return math.atan2(math.sin(target - actual), math.cos(target - actual))


class InspectionMission:
    def __init__(self):
        self.lock = threading.RLock()
        self.stations = rospy.get_param('~stations')
        if not self.stations:
            raise ValueError('At least one station is required')
        for station in self.stations:
            if len(station['position']) != 3 or not all(
                    math.isfinite(float(v)) for v in station['position'] + [station['yaw_deg']]):
                raise ValueError('Station position and yaw must be finite')
            # The existing manual-goal input uses a fixed 1 metre altitude.
            if abs(station['position'][2] - 1.0) > 1e-6:
                raise ValueError('This simulation goal interface currently requires z=1 metre')
        self.pos_tol = self.positive('position_tolerance', 0.2)
        self.vel_tol = self.positive('speed_tolerance', 0.1)
        self.yaw_tol = math.radians(self.positive('yaw_tolerance_deg', 5.0))
        self.rate_tol = math.radians(self.positive('yaw_rate_tolerance_deg', 5.0))
        self.stable_time = self.positive('stable_time', 1.0)
        self.hover_time = self.positive('hover_time', 3.0)
        self.hold_time = self.positive('station_hold_time', 1.0)
        self.leg_timeout = self.positive('leg_timeout', 120.0)
        self.station_timeout = self.positive('station_timeout', 45.0)
        self.odom_timeout = self.positive('odom_timeout', 0.5)
        self.state = 'WAIT_READY'
        self.index = 0
        self.completed = 0
        self.odom = None
        self.cmd = None
        self.odom_time = self.cmd_time = self.cloud_time = 0.0
        self.position = [0.0, 0.0, 0.0]
        self.speed = self.yaw = self.yaw_rate = 0.0
        self.phase_start = self.leg_start = self.station_start = time.monotonic()
        self.stable_since = None
        self.ready_since = None
        self.reason = ''
        self.goal_pub = rospy.Publisher('/waypoint_generator/waypoints', Path, queue_size=1)
        self.yaw_pub = rospy.Publisher('/inspection/yaw', Float64, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher('/inspection/status', String, queue_size=1, latch=True)
        self.event_pub = rospy.Publisher('/inspection/events', String, queue_size=20, latch=True)
        self.marker_pub = rospy.Publisher('/inspection/markers', MarkerArray, queue_size=1, latch=True)
        directory = FilePath(os.environ.get('ROS_HOME', str(FilePath.home() / '.ros')))
        directory.mkdir(parents=True, exist_ok=True)
        self.log_path = directory / ('inspection-' + time.strftime('%Y%m%d-%H%M%S') + '.jsonl')
        self.log = self.log_path.open('a', buffering=1)
        self.subscribers = [
            rospy.Subscriber('/visual_slam/odom', Odometry, self.on_odom, queue_size=1),
            rospy.Subscriber('/planning/pos_cmd', PositionCommand, self.on_cmd, queue_size=1),
            rospy.Subscriber('/map_generator/global_cloud', PointCloud2, self.on_cloud, queue_size=1),
        ]
        self.start_requested = rospy.get_param('~auto_start', True)
        self.start_service = rospy.Service('/inspection/start', Trigger, self.on_start)
        self.timer = rospy.Timer(rospy.Duration(0.05), self.tick)
        self.last_display = 0.0
        self.event('mission_loaded')

    @staticmethod
    def positive(name, default):
        value = float(rospy.get_param('~' + name, default))
        if not math.isfinite(value) or value <= 0:
            raise ValueError(name + ' must be finite and positive')
        return value

    def on_odom(self, msg):
        with self.lock:
            age = (rospy.Time.now() - msg.header.stamp).to_sec()
            if age > self.odom_timeout or age < -self.odom_timeout:
                return
            p, v, q = msg.pose.pose.position, msg.twist.twist.linear, msg.pose.pose.orientation
            values = [p.x, p.y, p.z, v.x, v.y, v.z, q.x, q.y, q.z, q.w,
                      msg.twist.twist.angular.z]
            if not all(math.isfinite(x) for x in values):
                return
            self.odom = msg
            self.position = [p.x, p.y, p.z]
            self.speed = norm([v.x, v.y, v.z])
            self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                                  1 - 2 * (q.y * q.y + q.z * q.z))
            self.yaw_rate = msg.twist.twist.angular.z
            self.odom_time = time.monotonic()

    def on_cmd(self, msg):
        with self.lock:
            self.cmd, self.cmd_time = msg, time.monotonic()

    def on_cloud(self, msg):
        if msg.width * msg.height:
            with self.lock:
                self.cloud_time = time.monotonic()

    def on_start(self, request):
        with self.lock:
            if self.state not in ('WAIT_READY', 'COMPLETE', 'FAILED'):
                return TriggerResponse(False, 'Mission is already active')
            self.index = self.completed = 0
            self.reason = ''
            self.ready_since = None
            self.start_requested = True
            self.change('WAIT_READY')
            return TriggerResponse(True, 'Waiting for fresh odometry and map')

    def snapshot(self):
        station = self.stations[min(self.index, len(self.stations) - 1)]
        return dict(state=self.state, station=self.index + 1, total=len(self.stations),
                    completed=self.completed, target=station['position'],
                    target_yaw_deg=station['yaw_deg'], position=self.position,
                    speed=self.speed, yaw_deg=math.degrees(self.yaw),
                    yaw_rate_deg=math.degrees(self.yaw_rate), reason=self.reason)

    def event(self, name):
        data = self.snapshot()
        data.update(event=name, time=rospy.Time.now().to_sec(), monotonic=time.monotonic())
        encoded = json.dumps(data, allow_nan=False)
        self.log.write(encoded + '\n')
        self.event_pub.publish(String(data=encoded))
        rospy.loginfo('[Inspection] %s: station %d/%d, %s', name, self.index + 1,
                      len(self.stations), self.state)

    def change(self, state):
        self.state = state
        self.phase_start = time.monotonic()
        self.stable_since = None
        self.event('state_changed')

    def send_goal(self):
        self.yaw_pub.publish(Float64(data=float('nan')))  # Return yaw ownership to flight tracking.
        goal = Path()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = 'world'
        pose = PoseStamped()
        pose.header = goal.header
        pose.pose.position = Point(*self.stations[self.index]['position'])
        pose.pose.orientation.w = 1.0
        goal.poses = [pose]
        self.goal_pub.publish(goal)
        self.leg_start = time.monotonic()
        self.change('TRAVEL')
        self.event('goal_sent')

    def at_rest(self, now):
        target = self.stations[self.index]['position']
        if self.cmd is None or now - self.cmd_time > self.odom_timeout:
            return False
        p, v = self.cmd.position, self.cmd.velocity
        # Also require the execution server to be holding this station, not an intermediate stop.
        return (norm([a - b for a, b in zip(self.position, target)]) <= self.pos_tol
                and self.speed <= self.vel_tol
                and norm([p.x - target[0], p.y - target[1], p.z - target[2]]) <= self.pos_tol
                and norm([v.x, v.y, v.z]) < 0.05)

    def continuous(self, condition, duration, now):
        if not condition:
            self.stable_since = None
            return False
        if self.stable_since is None:
            self.stable_since = now
        return now - self.stable_since >= duration

    def fail(self, reason):
        self.reason = reason
        self.change('FAILED')
        self.event('mission_failed')
        rospy.logerr('[Inspection] %s; no further station will be dispatched', reason)

    def tick(self, event):
        with self.lock:
            self.step(time.monotonic())

    def step(self, now):
        if self.state == 'WAIT_READY':
            ready = (self.odom is not None and now - self.odom_time < self.odom_timeout
                     and now - self.cloud_time < 5.0 and self.goal_pub.get_num_connections() > 0
                     and self.yaw_pub.get_num_connections() > 0)
            if not ready:
                self.ready_since = None
            elif self.ready_since is None:
                self.ready_since = now
            elif self.start_requested and now - self.ready_since > 3.0:
                self.send_goal()
        elif self.state not in ('COMPLETE', 'FAILED'):
            if now - self.odom_time > self.odom_timeout:
                self.fail('Odometry unavailable or stale')
            elif self.state == 'TRAVEL':
                if now - self.leg_start > self.leg_timeout:
                    self.fail('Arrival timeout: target may be unreachable')
                elif self.continuous(self.at_rest(now), self.stable_time, now):
                    self.station_start = now
                    self.change('HOVER')
                    self.event('arrival_confirmed')
            else:
                if now - self.station_start > self.station_timeout:
                    self.fail('Station could not remain stable within timeout')
                elif not self.at_rest(now):
                    # Drift cancels all station timers. Never advance on elapsed time alone.
                    if self.state != 'RECOVER':
                        self.change('RECOVER')
                elif self.state == 'RECOVER':
                    if self.continuous(True, self.stable_time, now):
                        self.change('HOVER')
                elif self.state == 'HOVER':
                    if self.continuous(True, self.hover_time, now):
                        self.yaw_pub.publish(Float64(data=math.radians(self.stations[self.index]['yaw_deg'])))
                        self.change('ALIGN')
                elif self.state in ('ALIGN', 'STATION_HOLD'):
                    aligned = (abs(angle_error(math.radians(self.stations[self.index]['yaw_deg']),
                                               self.yaw)) <= self.yaw_tol
                               and abs(self.yaw_rate) <= self.rate_tol)
                    if self.state == 'ALIGN':
                        if self.continuous(aligned, self.stable_time, now):
                            self.change('STATION_HOLD')
                            self.event('heading_confirmed')
                    elif not aligned:
                        self.change('ALIGN')
                    elif self.continuous(True, self.hold_time, now):
                        self.completed += 1
                        self.event('station_completed')
                        if self.completed == len(self.stations):
                            self.change('COMPLETE')
                            self.event('mission_completed')
                        else:
                            self.index += 1
                            self.send_goal()
        if now - self.last_display >= 0.2:
            self.status_pub.publish(String(data=json.dumps(self.snapshot(), allow_nan=False)))
            self.draw()
            self.last_display = now

    def draw(self):
        markers = []
        for i, station in enumerate(self.stations):
            # P1 and P5 share a location. Draw one symbol to avoid z-fighting.
            repeated = next((j for j in range(i) if
                             self.stations[j]['position'] == station['position']), None)
            if repeated is not None:
                continue
            same_location = [j for j, item in enumerate(self.stations)
                             if item['position'] == station['position']]
            current_here = self.index in same_location and self.state != 'COMPLETE'
            marker = Marker()
            marker.header.frame_id = 'world'
            marker.header.stamp = rospy.Time.now()
            marker.ns, marker.id = 'inspection_stations', i
            marker.type, marker.action = Marker.SPHERE, Marker.ADD
            marker.pose.position = Point(*station['position'])
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.45
            marker.color.a = 0.9
            marker.color.g = 1.0 if i < self.completed else 0.7
            marker.color.r = 1.0 if current_here else 0.1
            marker.color.b = 0.2 if i <= self.index else 1.0
            markers.append(marker)
            label = Marker()
            label.header = marker.header
            label.ns, label.id = 'inspection_labels', i
            label.type = Marker.TEXT_VIEW_FACING
            label.pose.position = Point(station['position'][0], station['position'][1],
                                        station['position'][2] + 0.8 + 0.25 * (i == 4))
            label.pose.orientation.w = 1.0
            label.scale.z = 0.35
            label.color.r, label.color.g, label.color.b, label.color.a = 0.1, 0.1, 0.25, 1.0
            label.text = '/'.join('P%d' % (j + 1) for j in same_location)
            label.text += '  yaw=%g deg' % station['yaw_deg']
            markers.append(label)
            arrow = Marker()
            arrow.header = marker.header
            arrow.ns, arrow.id = 'inspection_headings', i
            arrow.type = Marker.ARROW
            yaw = math.radians(station['yaw_deg'])
            x, y, z = station['position']
            arrow.points = [Point(x, y, z + 0.3),
                            Point(x + math.cos(yaw), y + math.sin(yaw), z + 0.3)]
            arrow.pose.orientation.w = 1.0
            arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.08, 0.18, 0.25
            arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 0.5, 0.1, 0.8, 1.0
            markers.append(arrow)
        status = Marker()
        status.header.frame_id = 'world'
        status.ns, status.id = 'inspection_status', 0
        status.type = Marker.TEXT_VIEW_FACING
        status.pose.position = Point(-12.0, -10.0, 3.0)
        status.pose.orientation.w = 1.0
        status.scale.z = 0.65
        status.color.a = 1.0
        status.color.r = 0.8 if self.state == 'FAILED' else 0.1
        status.color.g = 0.1 if self.state == 'FAILED' else 0.5
        status.text = 'INSPECTION %d/%d  %s' % (self.completed, len(self.stations), self.state)
        if self.reason:
            status.text += '\n' + self.reason
        markers.append(status)
        self.marker_pub.publish(MarkerArray(markers=markers))


if __name__ == '__main__':
    rospy.init_node('inspection_mission')
    mission = InspectionMission()
    rospy.spin()
