#!/usr/bin/env python3
"""
tiago_patrol.py
Manda TIAGo avanti e indietro tra WP_SPAWN e WP_TABLE.
"""

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion
import tf.transformations

WAYPOINTS = [
    {"name": "WP_TABLE", "x": 4.0, "y": 0.0, "yaw": 0.0},
    {"name": "WP_SPAWN", "x": -6.0, "y": 0.0, "yaw": 3.14},
]

WAIT_AT_GOAL = 5  # secondi di attesa dopo aver raggiunto il goal


def yaw_to_quaternion(yaw):
    q = tf.transformations.quaternion_from_euler(0, 0, yaw)
    return Quaternion(*q)


def send_goal(client, wp):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = wp["x"]
    goal.target_pose.pose.position.y = wp["y"]
    goal.target_pose.pose.position.z = 0.0
    goal.target_pose.pose.orientation = yaw_to_quaternion(wp["yaw"])

    rospy.loginfo(f"Navigating to {wp['name']} ({wp['x']}, {wp['y']})")
    client.send_goal(goal)
    client.wait_for_result()

    state = client.get_state()
    if state == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo(f"Reached {wp['name']}, waiting {WAIT_AT_GOAL}s...")
        rospy.sleep(WAIT_AT_GOAL)
    else:
        rospy.logwarn(f"Failed to reach {wp['name']}, state={state}")


def main():
    rospy.init_node("tiago_patrol")
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server...")
    client.wait_for_server()
    rospy.loginfo("Connected to move_base")

    idx = 0
    while not rospy.is_shutdown():
        send_goal(client, WAYPOINTS[idx % len(WAYPOINTS)])
        idx += 1


if __name__ == "__main__":
    main()
