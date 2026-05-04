import math
import os
import pickle
import sys
import rospy
try:
    sys.path.insert(0, os.environ["PNP_HOME"] + '/scripts')
except:
    print("Please set PNP_HOME environment variable to PetriNetPlans folder.")
    sys.exit(1)

import pnp_cmd_ros
from pnp_cmd_ros import *
from std_msgs.msg import String
import hrisim_util.ros_utils as ros_utils
import networkx as nx


# ── Waypoints del corridoio ──────────────────────────────────────────
#   WP_SPAWN  (-6.0, 0.0)  — posizione iniziale TIAGo  → PUNTO A
#   WP_CROSS  ( 0.0, -0.9) — incrocio davanti ai poster (waypoint intermedio)
#   WP_TABLE  ( 3.5, 0.0)  — destinazione finale        → PUNTO B (arretrato per TEB inflation_dist)

POINT_A = "WP_SPAWN"
POINT_B = "WP_TABLE"


def send_goal(p, next_dest, nextnext_dest=None):
    pos = nx.get_node_attributes(G, 'pos')
    x, y = pos[next_dest]
    if nextnext_dest is not None:
        x2, y2 = pos[nextnext_dest]
        angle = math.atan2(y2 - y, x2 - x)
        inputs = [x, y, angle, TIME_THRESHOLD]
    else:
        inputs = [x, y, 0, TIME_THRESHOLD]
    p.exec_action('goto', "_".join([str(i) for i in inputs]))


def heuristic(a, b):
    pos = nx.get_node_attributes(G, 'pos')
    (x1, y1) = pos[a]
    (x2, y2) = pos[b]
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5


def Plan(p):
    while not ros_utils.wait_for_param("/pnp_ros/ready"):
        rospy.sleep(0.1)

    # Necessario per sincronizzarsi con peopleflow_pedsim_bridge
    # e permettere lo spawn delle persone simulate
    ros_utils.wait_for_service('/hrisim/new_task')
    ros_utils.wait_for_service('/hrisim/finish_task')

    rospy.set_param('/hrisim/robot_busy', False)
    rospy.set_param("/peopleflow/robot_plan_on", True)

    while ROBOT_CLOSEST_WP is None:
        rospy.loginfo("[TIAGo_plan] Waiting for robot position...")
        rospy.sleep(0.1)

    rospy.loginfo(f"[TIAGo_plan] Starting continuous A↔B loop.")

    while not rospy.is_shutdown():
        for start, end in [(POINT_A, POINT_B), (POINT_B, POINT_A)]:
            rospy.loginfo(f"[TIAGo_plan] {start} → {end}")
            path = nx.astar_path(G, start, end, heuristic=heuristic, weight='weight')
            rospy.loginfo(f"[TIAGo_plan] Path: {path}")

            queue = list(path)
            while queue:
                current_wp = queue.pop(0)
                next_wp = queue[0] if queue else None
                send_goal(p, current_wp, next_wp)

            rospy.loginfo(f"[TIAGo_plan] Reached {end} ✓")
            rospy.sleep(1.0)  # breve pausa prima di ripartire

    rospy.set_param("/peopleflow/robot_plan_on", False)


def cb_robot_closest_wp(wp: String):
    global ROBOT_CLOSEST_WP
    ROBOT_CLOSEST_WP = wp.data


if __name__ == "__main__":
    ROBOT_CLOSEST_WP = None

    p = PNPCmd()

    g_path = ros_utils.wait_for_param("/peopleflow_pedsim_bridge/g_path")
    with open(g_path, 'rb') as f:
        G = pickle.load(f)

    TIME_THRESHOLD = ros_utils.wait_for_param("/hrisim/abort_time_threshold")

    rospy.Subscriber("/hrisim/robot_closest_wp", String, cb_robot_closest_wp)

    p.begin()
    Plan(p)
    p.end()
