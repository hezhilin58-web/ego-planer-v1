#!/usr/bin/env python3
"""Run after starting run-local.sh sim on a dedicated ROS master."""
import json
import math
import time

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from ego_planner.msg import Bspline
from quadrotor_msgs.msg import PositionCommand

rospy.init_node('local_deployment_check', anonymous=True)
counts = dict(odom=0, cloud=0, bspline=0, command=0)
positions = []


def receive_odom(msg):
    counts['odom'] += 1
    p = msg.pose.pose.position
    positions.append((p.x, p.y, p.z))


def receive(name):
    def callback(msg):
        counts[name] += 1
    return callback


subscriptions = [
    rospy.Subscriber('/visual_slam/odom', Odometry, receive_odom),
    rospy.Subscriber('/pcl_render_node/cloud', PointCloud2, receive('cloud')),
    rospy.Subscriber('/planning/bspline', Bspline, receive('bspline')),
    rospy.Subscriber('/planning/pos_cmd', PositionCommand, receive('command')),
]
publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
deadline = time.monotonic() + 25
while time.monotonic() < deadline:
    if positions and counts['cloud'] and publisher.get_num_connections():
        break
    time.sleep(0.1)
if not positions or not counts['cloud']:
    raise RuntimeError(f'Simulation did not provide odometry and sensing: {counts}')
start = positions[-1]
goal = PoseStamped()
goal.header.frame_id = 'world'
goal.header.stamp = rospy.Time.now()
goal.pose.position.x = start[0] + 3.0
goal.pose.position.y = start[1]
goal.pose.position.z = 1.0
goal.pose.orientation.w = 1.0
publisher.publish(goal)
deadline = time.monotonic() + 20
while time.monotonic() < deadline:
    distance = math.dist(start, positions[-1])
    goal_error = math.dist((goal.pose.position.x, goal.pose.position.y, 1.0), positions[-1])
    if counts['bspline'] and counts['command'] and goal_error < 0.35:
        break
    time.sleep(0.1)
result = dict(messages=counts, start=start, end=positions[-1], displacement_m=distance,
              goal_error_m=goal_error)
print(json.dumps(result, indent=2))
if not (counts['bspline'] and counts['command'] and goal_error < 0.35):
    raise RuntimeError('Planning and simulated movement verification failed')
