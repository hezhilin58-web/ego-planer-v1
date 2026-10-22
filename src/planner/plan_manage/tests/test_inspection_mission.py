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
        self.stable_time, self.hover_time = 1., 3.
        self.leg_timeout, self.station_timeout, self.odom_timeout = 120., 45., 0.5
        self.recovery_height, self.descent_step, self.descent_speed = 0.21, 0.05, 0.1
        self.descent_tolerance, self.descent_timeout, self.descent_step_timeout = .02, 180., 15.
        self.home = [-18., 0., 0.]
        self.target = list(self.stations[0]['position'])
        self.descent_start = self.next_descent_time = 0.
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

    def test_arrival_immediately_requests_heading_without_pre_turn_dwell(self):
        m = MissionFixture()
        m.yaw = 1.
        m.advance(1.)
        self.assertEqual(m.state, 'ALIGN')
        self.assertEqual(len(m.yaw_pub.messages), 1)
        self.assertEqual(m.yaw_pub.messages[0].data, 0.)
        self.assertIn(('arrival_confirmed', 0, 'ALIGN'), m.events)

    def test_heading_must_settle_before_full_hover_then_next_goal(self):
        m = MissionFixture()
        m.advance(1.)
        m.yaw = 1.
        m.advance(2.)
        m.advance(6.)
        self.assertEqual(m.state, 'ALIGN')
        self.assertEqual(m.completed, 0)
        self.assertEqual(len(m.goal_pub.messages), 0)
        m.yaw = 0.
        m.advance(7.)
        m.advance(7.9)
        self.assertEqual(m.state, 'ALIGN')
        m.advance(8.1)
        self.assertEqual(m.state, 'HOVER')
        m.advance(8.2)
        m.advance(11.1)
        self.assertEqual(m.completed, 0)
        m.advance(11.3)
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
        self.assertEqual(m.state, 'ALIGN')
        m.advance(4.1)
        m.advance(5.1)
        self.assertEqual(m.state, 'HOVER')
        self.assertEqual(m.completed, 0)
        m.advance(5.2)
        m.advance(8.)
        self.assertEqual(m.completed, 0)
        m.advance(8.3)
        self.assertEqual(m.completed, 1)

    def test_heading_drift_discards_elapsed_hover(self):
        m = MissionFixture()
        m.state = 'HOVER'
        m.advance(1.)
        m.yaw = 1.
        m.advance(3.9)
        self.assertEqual(m.state, 'ALIGN')
        m.yaw = 0.
        m.advance(4.)
        m.advance(5.1)
        self.assertEqual(m.state, 'HOVER')
        m.advance(5.2)
        m.advance(8.)
        self.assertEqual(m.completed, 0)
        m.advance(8.3)
        self.assertEqual(m.completed, 1)

    def test_high_yaw_rate_blocks_hover_even_at_target_heading(self):
        m = MissionFixture()
        m.advance(1.)
        m.yaw_rate = 0.2
        m.advance(2.)
        m.advance(4.)
        self.assertEqual(m.state, 'ALIGN')
        self.assertEqual(m.completed, 0)

    def test_command_must_hold_the_station_before_turning(self):
        m = MissionFixture()
        m.cmd.velocity.x = 0.1
        m.advance(1.)
        self.assertEqual(m.state, 'TRAVEL')
        self.assertEqual(m.yaw_pub.messages, [])
        m.cmd.velocity.x = 0.
        m.cmd.position.x = -14.
        m.advance(2.)
        self.assertEqual(m.state, 'TRAVEL')
        m.cmd.position.x = -15.
        m.cmd_time = 0.
        m.odom_time = 3.
        m.advance(3., fresh=False)
        self.assertEqual(m.state, 'TRAVEL')

    def test_station_timeout_does_not_advance(self):
        m = MissionFixture()
        m.state = 'ALIGN'
        m.advance(46.)
        self.assertEqual(m.state, 'FAILED')
        self.assertEqual(m.completed, 0)
        self.assertEqual(m.goal_pub.messages, [])

    def test_final_station_starts_return_instead_of_finishing_mission(self):
        m = MissionFixture()
        m.stations = m.stations[:1]
        m.advance(1.)
        m.advance(1.1)
        m.advance(2.2)
        self.assertEqual(m.state, 'HOVER')
        m.advance(2.3)
        m.advance(5.4)
        self.assertEqual(m.state, 'RETURN_HOME')
        self.assertEqual(m.completed, 1)
        self.assertEqual(len(m.goal_pub.messages), 1)
        self.assertEqual(m.target, [-18., 0., 1.])
        self.assertFalse(any(e[0] == 'mission_completed' for e in m.events))

    def test_home_is_captured_from_actual_position_at_task_start(self):
        m = MissionFixture()
        m.state = 'WAIT_READY'
        m.home = None
        m.position = [3., -2., .1]
        m.advance(1.)
        m.advance(4.1)
        self.assertEqual(m.home, [3., -2., .1])
        self.assertEqual(m.state, 'TRAVEL')

    def return_fixture(self):
        m = MissionFixture()
        m.state = 'RETURN_HOME'
        m.completed = len(m.stations)
        m.target = [-18., 0., 1.]
        m.position = [-18., 0., 1.]
        m.cmd.position.x, m.cmd.position.z = -18., 1.
        return m

    def test_return_waits_for_actual_arrival_and_stability_before_descent(self):
        m = self.return_fixture()
        m.position[0] = -17.
        m.advance(1.)
        m.advance(3.)
        self.assertEqual(m.state, 'RETURN_HOME')
        self.assertEqual(m.goal_pub.messages, [])
        m.position[0] = -18.
        m.advance(4.)
        m.advance(4.9)
        self.assertEqual(m.state, 'RETURN_HOME')
        m.advance(5.1)
        self.assertEqual(m.state, 'DESCEND')
        self.assertAlmostEqual(m.target[2], .95)

    def test_descent_waits_for_measured_height_and_minimum_step_interval(self):
        m = self.return_fixture()
        m.advance(1.)
        m.advance(2.1)
        self.assertEqual(m.target[2], .95)
        m.advance(2.2)
        self.assertEqual(len(m.goal_pub.messages), 1)
        m.position[2] = m.cmd.position.z = .95
        m.advance(2.3)
        m.advance(2.58)
        self.assertEqual(len(m.goal_pub.messages), 1)
        m.advance(2.7)
        self.assertAlmostEqual(m.target[2], .9)
        self.assertEqual(m.completed, len(m.stations))

    def test_recovery_uses_tighter_height_tolerance_and_continuous_stability(self):
        m = self.return_fixture()
        m.state = 'DESCEND'
        m.target[2] = m.cmd.position.z = .21
        m.position[2] = .26
        m.advance(1.)
        m.advance(3.)
        self.assertEqual(m.state, 'DESCEND')
        m.position[2] = .21
        m.advance(4.)
        m.position[2] = .26
        m.advance(4.9)
        m.position[2] = .21
        m.advance(5.)
        m.advance(5.9)
        self.assertEqual(m.state, 'DESCEND')
        m.advance(6.1)
        self.assertEqual(m.state, 'COMPLETE')
        self.assertEqual(m.goal_pub.messages, [])
        self.assertTrue(any(e[0] == 'mission_completed' for e in m.events))

    def test_descent_target_is_clamped_to_point_two_one_metres(self):
        m = self.return_fixture()
        m.target[2] = .23
        m.send_descent_goal(1.)
        self.assertEqual(m.target[2], .21)

    def test_return_timeout_never_starts_descent(self):
        m = self.return_fixture()
        m.advance(121.)
        self.assertEqual(m.state, 'FAILED')
        self.assertEqual(m.goal_pub.messages, [])

    def test_short_final_remainder_is_merged_and_rate_interval_uses_real_step(self):
        m = self.return_fixture()
        m.target[2] = .29
        m.send_descent_goal(1.)
        self.assertEqual(m.target[2], .21)
        self.assertAlmostEqual(m.next_descent_time, 1.8)

    def test_stale_odom_during_descent_does_not_dispatch_lower_goal(self):
        m = self.return_fixture()
        m.state = 'DESCEND'
        m.advance(1., fresh=False)
        self.assertEqual(m.state, 'FAILED')
        self.assertEqual(m.goal_pub.messages, [])

    def test_blocked_descent_step_times_out_without_skipping(self):
        m = self.return_fixture()
        m.state = 'DESCEND'
        m.advance(16.)
        self.assertEqual(m.state, 'FAILED')
        self.assertEqual(m.goal_pub.messages, [])

    def test_yaw_wrap_uses_shortest_angle(self):
        self.assertAlmostEqual(module.angle_error(-3.13, 3.13), 0.02318530718)


if __name__ == '__main__':
    unittest.main()
