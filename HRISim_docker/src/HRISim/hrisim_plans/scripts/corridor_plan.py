#!/usr/bin/env python

import os
import sys
try:
    sys.path.insert(0, os.environ["PNP_HOME"] + '/scripts')
except:
    print("Please set PNP_HOME environment variable to PetriNetPlans folder.")
    sys.exit(1)

import rospy
import pickle
import math
import networkx as nx
import pnp_cmd_ros
from pnp_cmd_ros import *
import hrisim_util.ros_utils as ros_utils
from nav_msgs.msg import Odometry
from std_msgs.msg import String

ROBOT_CLOSEST_WP = None

def heuristic(a, b):
    pos = nx.get_node_attributes(G, 'pos')
    (x1, y1) = pos[a]
    (x2, y2) = pos[b]
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

def send_goal(p, next_dest, nextnext_dest=None):
    pos = nx.get_node_attributes(G, 'pos')
    x, y = pos[next_dest]
    if nextnext_dest is not None:
        x2, y2 = pos[nextnext_dest]
        angle = math.atan2(y2 - y, x2 - x)
    else:
        angle = 0
    p.exec_action('goto', f"{x}_{y}_{angle}_{TIME_THRESHOLD}")

def cb_robot_closest_wp(wp: String):
    global ROBOT_CLOSEST_WP
    ROBOT_CLOSEST_WP = wp.data

def Plan(p):
    while not ros_utils.wait_for_param("/pnp_ros/ready"):
        rospy.sleep(0.1)

    # Aspetta che le persone si posizionino
    rospy.logwarn("Waiting for people to reach positions...")
    rospy.sleep(5.0)

    # Calcola path WP_SPAWN -> WP_TABLE
    path = nx.astar_path(G, "WP_SPAWN", "WP_TABLE", heuristic=heuristic, weight='weight')
    rospy.logwarn(f"Path to WP_TABLE: {path}")

    # Naviga waypoint per waypoint
    for i, wp in enumerate(path):
        next_wp = path[i + 1] if i + 1 < len(path) else None
        send_goal(p, wp, next_wp)

        # Quando arriva a WP_CROSS, interagisce
        if wp == "WP_CROSS":
            rospy.logwarn("At WP_CROSS — triggering interaction")
            p.exec_action('moveHead', "0.7_0.0")

    rospy.logwarn("Reached WP_TABLE.")

if __name__ == "__main__":
    p = PNPCmd()

    g_path = ros_utils.wait_for_param("/peopleflow_pedsim_bridge/g_path")
    with open(g_path, 'rb') as f:
        G = pickle.load(f)

    TIME_THRESHOLD = ros_utils.wait_for_param("/hrisim/abort_time_threshold")

    rospy.Subscriber("/hrisim/robot_closest_wp", String, cb_robot_closest_wp)

    p.begin()
    Plan(p)
    p.end()
