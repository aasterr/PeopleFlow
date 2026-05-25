#!/usr/bin/env python
# -*- coding: utf-8 -*-

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
from std_msgs.msg import String, Int32, Bool
from std_srvs.srv import SetBool
from pedsim_msgs.msg import AgentStates
from geometry_msgs.msg import PoseWithCovarianceStamped
import hrisim_util.ros_utils as ros_utils
import networkx as nx

# ── Waypoint ─────────────────────────────────────────────────────────
POINT_A    = "WP_SPAWN"
POINT_B    = "WP_TABLE"
WP_OBS_FWD = "WP_OBS_FWD"
WP_OBS_BWD = "WP_OBS_BWD"
WP_CENTRE  = "WP_CENTRE"

# ── Geometria ────────────────────────────────────────────────────────
ROBOT_WIDTH       = 0.55
PERSONAL_SPACE    = 0.45
SAFETY_RADIUS     = PERSONAL_SPACE
GAP_MIN           = ROBOT_WIDTH + 2 * PERSONAL_SPACE  # 1.45 m
LOOK_AHEAD        = 3.0
LOOK_BEHIND       = 0.5
CORRIDOR_CENTER_Y = 0.0
CORRIDOR_WIDTH    = 2.5
_CROSS_CX         = 0.0
_CROSS_CY         = -0.9
_CROSS_RX         = 1.5
_CROSS_RY         = 1.5

# ── Comportamento ────────────────────────────────────────────────────
RECHECK_INTERVAL = 2.0
ACTION_POLICY    = "always_act"  # "always_act" | "never_act" | "alternate"

# ── Stato globale ────────────────────────────────────────────────────
LAST_AGENTS      = []
ROBOT_CLOSEST_WP = None
ROBOT_XY         = (None, None)
action_pub       = None
G                = None
TIME_THRESHOLD   = None
_injected_agents = set()

# ─────────────────────────────────────────────────────────────────────
# CALLBACKS
# ─────────────────────────────────────────────────────────────────────

def cb_agents(msg):
    global LAST_AGENTS
    LAST_AGENTS = [a for a in msg.agent_states if a.type == 1]

def cb_robot_closest_wp(msg):
    global ROBOT_CLOSEST_WP
    ROBOT_CLOSEST_WP = msg.data

def cb_robot_pose(msg):
    global ROBOT_XY
    ROBOT_XY = (msg.pose.pose.position.x, msg.pose.pose.position.y)

# ─────────────────────────────────────────────────────────────────────
# NAVIGAZIONE
# ─────────────────────────────────────────────────────────────────────

def heuristic(a, b):
    pos = nx.get_node_attributes(G, 'pos')
    x1, y1 = pos[a]
    x2, y2 = pos[b]
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

def send_goal(p, current_dest, next_dest=None, prev_dest=None):
    pos = nx.get_node_attributes(G, 'pos')
    x, y = pos[current_dest]
    if next_dest is not None:
        x2, y2 = pos[next_dest]
        angle = math.atan2(y2 - y, x2 - x)
    elif prev_dest is not None:
        xp, yp = pos[prev_dest]
        angle = math.atan2(y - yp, x - xp)
    else:
        angle = 0.0
    p.exec_action('goto', "_".join([str(v) for v in [x, y, angle, TIME_THRESHOLD]]))

def navigate(p, start, end):
    rospy.loginfo("[TIAGo] navigate: %s -> %s", start, end)
    path = nx.astar_path(G, start, end, heuristic=heuristic, weight='weight')
    path = path[1:]
    if not path:
        return
    for i, wp in enumerate(path):
        prev_wp = path[i-1] if i > 0 else start
        next_wp = path[i+1] if i < len(path)-1 else None
        send_goal(p, wp, next_dest=next_wp, prev_dest=prev_wp)
    rospy.loginfo("[TIAGo] Arrivato a %s.", end)

# ─────────────────────────────────────────────────────────────────────
# CHECK CONGESTIONE
# ─────────────────────────────────────────────────────────────────────

def check_congestion(direction):
    rx, ry = ROBOT_XY
    if rx is None:
        rospy.logwarn("[TIAGo] Posizione robot non disponibile, assumo LIBERO.")
        return False

    corridor_y_min = CORRIDOR_CENTER_Y - CORRIDOR_WIDTH / 2.0
    corridor_y_max = CORRIDOR_CENTER_Y + CORRIDOR_WIDTH / 2.0

    if direction == "FWD":
        x_min, x_max = rx - LOOK_BEHIND, rx + LOOK_AHEAD
    else:
        x_min, x_max = rx - LOOK_AHEAD, rx + LOOK_BEHIND

    relevant = [
        (a.id, a.pose.position.x, a.pose.position.y)
        for a in LAST_AGENTS
        if x_min <= a.pose.position.x <= x_max
        and corridor_y_min <= a.pose.position.y <= corridor_y_max
    ]

    if not relevant:
        rospy.loginfo("[TIAGo] Corridoio LIBERO (nessun agente)")
        return False

    obstacles = [("wall_low", corridor_y_min - 0.01, corridor_y_min)]
    for aid, px, py in relevant:
        obstacles.append(("p{}".format(aid), py - SAFETY_RADIUS, py + SAFETY_RADIUS))
    obstacles.append(("wall_high", corridor_y_max, corridor_y_max + 0.01))
    obstacles.sort(key=lambda o: o[1])

    best_gap = max(
        obstacles[i+1][1] - obstacles[i][2]
        for i in range(len(obstacles) - 1)
    )

    congested = best_gap < GAP_MIN
    rospy.loginfo("[TIAGo] [%s] %d agenti | gap_max=%.3f m -> %s",
                  direction, len(relevant), best_gap,
                  "BLOCCATO" if congested else "LIBERO")
    return congested

# ─────────────────────────────────────────────────────────────────────
# AGENTI
# ─────────────────────────────────────────────────────────────────────

def _in_cross_zone(ax, ay):
    return abs(ax - _CROSS_CX) <= _CROSS_RX and abs(ay - _CROSS_CY) <= _CROSS_RY

def inject_waypoint(agent_id, target_wp):
    rospy.set_param('/hrisim/override/{}/dest'.format(agent_id), target_wp)
    rospy.loginfo("[inject] Agente %s -> %s", agent_id, target_wp)

def inject_agents_by_zone():
    global _injected_agents
    _injected_agents.clear()
    moved = 0
    for agent in LAST_AGENTS:
        ax = agent.pose.position.x
        ay = agent.pose.position.y
        if not _in_cross_zone(ax, ay):
            continue
        inject_waypoint(str(agent.id), 'WP_CROSS_BACK')
        _injected_agents.add(str(agent.id))
        moved += 1
    rospy.loginfo("[inject] %d override impostati verso WP_CROSS_BACK", moved)
    
def reinject_agents_to_cross():
    for aid in _injected_agents:
        inject_waypoint(aid, 'WP_CROSS')
    rospy.loginfo("[inject] %d agenti reiniettati verso WP_CROSS", len(_injected_agents))

def wait_for_agent_in_cross(timeout=60.0):
    if not _injected_agents:
        rospy.loginfo("[TIAGo] Nessun agente iniettato, procedo subito")
        return True
    rospy.loginfo("[TIAGo] Aspetto che tornino in zona CROSS: %s", _injected_agents)
    deadline = rospy.Time.now() + rospy.Duration(timeout)
    rate = rospy.Rate(2)
    while not rospy.is_shutdown() and rospy.Time.now() < deadline:
        agents_by_id = {str(a.id): a for a in LAST_AGENTS}
        if all(
            str(aid) in agents_by_id and
            _in_cross_zone(agents_by_id[str(aid)].pose.position.x,
                           agents_by_id[str(aid)].pose.position.y)
            for aid in _injected_agents
        ):
            rospy.loginfo("[TIAGo] Tutti gli agenti %s tornati in zona CROSS", _injected_agents)
            return True
        rate.sleep()
    rospy.logwarn("[TIAGo] Timeout: procedo comunque")
    return False

# ─────────────────────────────────────────────────────────────────────
# AZIONE
# ─────────────────────────────────────────────────────────────────────

def choose_action(episode_num):
    if ACTION_POLICY == "always_act":
        return 1
    elif ACTION_POLICY == "never_act":
        return 0
    else:
        return episode_num % 2

def emit_action(action_val):
    action_pub.publish(Int32(action_val))
    rospy.loginfo("[TIAGo] A=%d pubblicata", action_val)
    if action_val == 1:
        try:
            rospy.wait_for_service('/pmb2/set_leds', timeout=2.0)
            set_leds = rospy.ServiceProxy('/pmb2/set_leds', SetBool)
            for _ in range(3):
                set_leds(True)
                rospy.sleep(0.5)
                set_leds(False)
                rospy.sleep(0.5)
            rospy.loginfo("[TIAGo] LED blink completato")
        except Exception as e:
            rospy.logwarn("[TIAGo] LED blink fallito: %s", e)
        inject_agents_by_zone()

# ─────────────────────────────────────────────────────────────────────
# EPISODIO
# ─────────────────────────────────────────────────────────────────────

def run_half(p, episode_num, start, obs_wp, end, direction,
             episode_start_pub, episode_end_pub, people_cleared_pub, transit_pub):
    rospy.loginfo("[TIAGo] ── EPISODIO %d | %s: %s -> %s ──", episode_num, direction, start, end)

    episode_start_pub.publish(Int32(episode_num))
    navigate(p, start, obs_wp)
    rospy.sleep(0.5)

    congested = check_congestion(direction)
    if congested:
        A = choose_action(episode_num)
        emit_action(A)
        last_action_time = rospy.Time.now()
        rospy.sleep(2.0)
        while not rospy.is_shutdown():
            rospy.sleep(RECHECK_INTERVAL)
            if not check_congestion(direction):
                rospy.loginfo("[TIAGo] Corridoio libero, procedo.")
                people_cleared_pub.publish(Bool(True))
                break
            if A == 1 and (rospy.Time.now() - last_action_time) > rospy.Duration(10.0):
                inject_agents_by_zone()
                last_action_time = rospy.Time.now()
    else:
        rospy.loginfo("[TIAGo] Corridoio libero, procedo direttamente.")

    transit_pub.publish(Bool(True))
    navigate(p, obs_wp, end)
    episode_end_pub.publish(Int32(episode_num))
    rospy.sleep(1.0)

# ─────────────────────────────────────────────────────────────────────
# PLAN
# ─────────────────────────────────────────────────────────────────────

def Plan(p):
    while not ros_utils.wait_for_param("/pnp_ros/ready"):
        rospy.sleep(0.1)
    ros_utils.wait_for_service('/hrisim/new_task')
    ros_utils.wait_for_service('/hrisim/finish_task')
    rospy.set_param('/hrisim/robot_busy', False)
    rospy.set_param("/peopleflow/robot_plan_on", True)

    while ROBOT_CLOSEST_WP is None:
        rospy.sleep(0.1)
    while not rospy.is_shutdown() and not LAST_AGENTS:
        rospy.sleep(0.2)

    rospy.loginfo("[TIAGo] Pronti | GAP_MIN=%.2f | LOOK_AHEAD=%.1f | POLICY=%s",
                  GAP_MIN, LOOK_AHEAD, ACTION_POLICY)

    episode_start_pub  = rospy.Publisher("/hrisim/episode_start",  Int32, queue_size=1)
    episode_end_pub    = rospy.Publisher("/hrisim/episode_end",    Int32, queue_size=1)
    people_cleared_pub = rospy.Publisher("/hrisim/people_cleared", Bool,  queue_size=1)
    transit_pub        = rospy.Publisher("/hrisim/robot_transit",  Bool,  queue_size=1)
    rospy.sleep(0.5)

    episode = 0
    while not rospy.is_shutdown():
        episode += 1
        run_half(p, episode, start=POINT_A, obs_wp=WP_OBS_FWD, end=POINT_B, direction="FWD",
                 episode_start_pub=episode_start_pub, episode_end_pub=episode_end_pub,
                 people_cleared_pub=people_cleared_pub, transit_pub=transit_pub)
        reinject_agents_to_cross()
        wait_for_agent_in_cross()

        episode += 1
        run_half(p, episode, start=POINT_B, obs_wp=WP_OBS_BWD, end=POINT_A, direction="BWD",
                 episode_start_pub=episode_start_pub, episode_end_pub=episode_end_pub,
                 people_cleared_pub=people_cleared_pub, transit_pub=transit_pub)
        reinject_agents_to_cross()
        wait_for_agent_in_cross()

    rospy.set_param("/peopleflow/robot_plan_on", False)

# ─────────────────────────────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    p = PNPCmd()

    g_path = ros_utils.wait_for_param("/peopleflow_pedsim_bridge/g_path")
    with open(g_path, 'rb') as f:
        G = pickle.load(f)

    TIME_THRESHOLD = ros_utils.wait_for_param("/hrisim/abort_time_threshold")

    rospy.Subscriber("/hrisim/robot_closest_wp",           String,                    cb_robot_closest_wp)
    rospy.Subscriber("/pedsim_simulator/simulated_agents", AgentStates,               cb_agents, queue_size=1)
    rospy.Subscriber("/robot_pose",                        PoseWithCovarianceStamped, cb_robot_pose, queue_size=1)

    action_pub = rospy.Publisher("/hrisim/robot_action", Int32, queue_size=1)

    p.begin()
    Plan(p)
    p.end()
