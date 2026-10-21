#!/usr/bin/env python3
"""Exercise mission gating without starting a ROS master or moving a robot."""
import importlib.util
import threading
import unittest
from pathlib import Path
from unittest.mock import patch

import rospy
from quadrotor_msgs.msg import PositionCommand

spec = importlib.util.spec_from_file_location(
    'inspection', Path(__file__).resolve().parents[1] / 'scripts/inspection_mission.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
rospy.rostime.set_rostime_initialized(True)


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)

    def get_num_connections(self):
        return 1


class MissionFixture(module.InspectionMission):
    def __init__(self):
        self.lock = threading.RLock()
        self.stations = [dict(position=[-15., 0., 1.], yaw_deg=0.),
                         dict(position=[0., 15., 1.], yaw_deg=-90.)]
        self.pos_tol, self.vel_tol = 0.2, 0.1
        self.yaw_tol = self.rate_tol = 0.087266
        self.stable_time, self.hover_time, self.hold_time = 1., 3., 1.
        self.leg_timeout, self.station_timeout, self.odom_timeout = 120., 45., 0.5
        self.index = self.completed = 0
        self.state = 'TRAVEL'
        self.reason = ''
        self.position = [-15., 0., 1.]
        self.speed = self.yaw = self.yaw_rate = 0.
        self.odom = object()
        self.cmd = PositionCommand()
        self.cmd.position.x, self.cmd.position.z = -15., 1.
        self.odom_time = self.cmd_time = self.cloud_time = 0.
        self.leg_start = self.phase_start = self.station_start = 0.
        self.stable_since = self.ready_since = None
        self.last_display = 0.
        self.start_requested = True
        self.goal_pub, self.yaw_pub, self.status_pub = Publisher(), Publisher(), Publisher()
        self.events = []

    def event(self, name):
        self.events.append((name, self.index, self.state))

    def draw(self):
        pass

    def advance(self, now, fresh=True):
        if fresh:
            self.odom_time = self.cmd_time = self.cloud_time = now
        with patch.object(module.time, 'monotonic', return_value=now):
            self.step(now)


class MissionTests(unittest.TestCase):
    def test_fast_pass_does_not_count_as_arrival(self):
        m = MissionFixture()
        m.speed = 0.8
        m.advance(1.)
        m.advance(3.)
        self.assertEqual(m.state, 'TRAVEL')
        self.assertEqual(m.completed, 0)

    def test_stability_timer_resets_and_heading_gates_next_goal(self):
        m = MissionFixture()
        m.advance(1.)
        m.position[0] = -14.
        m.advance(1.8)
        m.position[0] = -15.
        m.advance(2.)
        m.advance(2.9)
        self.assertEqual(m.state, 'TRAVEL')
        m.advance(3.1)
        self.assertEqual(m.state, 'HOVER')
        m.advance(3.2)
        m.advance(6.3)
        self.assertEqual(m.state, 'ALIGN')
        m.yaw = 1.
        m.advance(7.)
        m.advance(9.)
        self.assertEqual(m.completed, 0)
        self.assertEqual(len(m.goal_pub.messages), 0)
        m.yaw = 0.
        m.advance(10.)
        m.advance(11.1)
        self.assertEqual(m.state, 'STATION_HOLD')
        m.advance(11.2)
        m.advance(12.3)
        self.assertEqual(m.completed, 1)
        self.assertEqual(m.index, 1)
        self.assertEqual(m.state, 'TRAVEL')
        self.assertEqual(m.goal_pub.messages[0].poses[0].pose.position.y, 15.)

    def test_stale_odometry_fails_without_dispatching_next_station(self):
        m = MissionFixture()
        m.advance(1., fresh=False)
        self.assertEqual(m.state, 'FAILED')
        self.assertEqual(m.goal_pub.messages, [])

    def test_unreachable_target_times_out_without_skipping(self):
        m = MissionFixture()
        m.position[0] = -18.
        m.advance(121.)
        self.assertEqual(m.state, 'FAILED')
        self.assertEqual(m.index, 0)
        self.assertEqual(m.completed, 0)

    def test_drift_during_hover_restarts_station_settling(self):
        m = MissionFixture()
        m.state = 'HOVER'
        m.station_start = 1.
        m.advance(1.)
        m.position[0] = -14.
        m.advance(3.9)
        self.assertEqual(m.state, 'RECOVER')
        m.position[0] = -15.
        m.advance(4.)
        m.advance(5.1)
        self.assertEqual(m.state, 'HOVER')
        self.assertEqual(m.completed, 0)

    def test_yaw_wrap_uses_shortest_angle(self):
        self.assertAlmostEqual(module.angle_error(-3.13, 3.13), 0.02318530718)


if __name__ == '__main__':
    unittest.main()
