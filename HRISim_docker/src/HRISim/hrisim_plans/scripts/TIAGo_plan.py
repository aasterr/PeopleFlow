#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Loop: SPAWN -> WP_OBS_FWD -> TABLE -> WP_OBS_BWD -> SPAWN
A ogni WP_OBS: check congestione → se congesto emette azione → aspetta → procede.
L'azione HRI inietta un nuovo path direttamente nei rosparam degli agenti.
"""

import math
import os
import pickle
import sys
import threading

import rospy

try:
    sys.path.insert(0, os.environ["PNP_HOME"] + '/scripts')
except:
    print("Please set PNP_HOME environment variable to PetriNetPlans folder.")
    sys.exit(1)

import pnp_cmd_ros
from pnp_cmd_ros import *
from std_msgs.msg import String, Int32
from pedsim_msgs.msg import AgentStates
from geometry_msgs.msg import PoseWithCovarianceStamped
import actionlib
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
import hrisim_util.ros_utils as ros_utils
import networkx as nx

# ── Waypoint ─────────────────────────────────────────────────────────
POINT_A     = "WP_SPAWN"
POINT_B     = "WP_TABLE"
WP_OBS_FWD  = "WP_OBS_FWD"
WP_OBS_BWD  = "WP_OBS_BWD"
WP_CENTRE   = "WP_CENTRE"

# ── Geometria ────────────────────────────────────────────────────────
ROBOT_WIDTH       = 0.55
PERSONAL_SPACE    = 0.45
SAFETY_RADIUS     = PERSONAL_SPACE
GAP_MIN           = ROBOT_WIDTH + 2 * PERSONAL_SPACE   # 1.45 m
LOOK_AHEAD        = 3.0
LOOK_BEHIND       = 0.5
CORRIDOR_CENTER_Y = 0.0
CORRIDOR_WIDTH    = 2.5

# ── Comportamento ────────────────────────────────────────────────────
RECHECK_INTERVAL = 2.0
ACTION_POLICY    = "always_act"   # "always_act" | "never_act" | "alternate"

# ── Zone spostamento agenti ──────────────────────────────────────────
ZONE_RADIUS         = 1.5
_ZONE_CROSS_XY      = (0.0, -0.9)
_ZONE_POSTER_L_XY   = (-0.6, 0.5)
_ZONE_POSTER_R_XY   = (0.6, 0.5)

# ── Stato globale ────────────────────────────────────────────────────
LAST_AGENTS      = []
ROBOT_CLOSEST_WP = None
ROBOT_XY         = (None, None)
action_pub       = None
G                = None
TIME_THRESHOLD   = None

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
        rospy.loginfo("[TIAGo] Già a destinazione %s.", end)
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
        x_min = rx - LOOK_BEHIND
        x_max = rx + LOOK_AHEAD
    else:
        x_min = rx - LOOK_AHEAD
        x_max = rx + LOOK_BEHIND

    relevant = []
    for agent in LAST_AGENTS:
        px = agent.pose.position.x
        py = agent.pose.position.y
        if x_min <= px <= x_max and corridor_y_min <= py <= corridor_y_max:
            relevant.append((agent.id, px, py))

    rospy.loginfo("[TIAGo] [%s] %d agenti nella finestra", direction, len(relevant))

    if not relevant:
        rospy.loginfo("[TIAGo] Corridoio LIBERO (nessun agente)")
        return False

    obstacles = [("wall_low", corridor_y_min - 0.01, corridor_y_min)]
    for aid, px, py in relevant:
        obstacles.append(("p{}".format(aid), py - SAFETY_RADIUS, py + SAFETY_RADIUS))
    obstacles.append(("wall_high", corridor_y_max, corridor_y_max + 0.01))
    obstacles.sort(key=lambda o: o[1])

    best_gap = 0.0
    for i in range(len(obstacles) - 1):
        gap = obstacles[i + 1][1] - obstacles[i][2]
        if gap > best_gap:
            best_gap = gap

    congested = best_gap < GAP_MIN
    rospy.loginfo("[TIAGo] gap_max=%.3f m (soglia=%.3f) -> %s",
                  best_gap, GAP_MIN, "BLOCCATO" if congested else "LIBERO")
    return congested

# ─────────────────────────────────────────────────────────────────────
# INIEZIONE PATH AGENTI (nuova strategia)
# ─────────────────────────────────────────────────────────────────────

def _dist2d(x1, y1, x2, y2):
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

def _classify_agent(ax, ay):
    """Tutti gli agenti vanno sempre in CROSS_BACK."""
    return 'WP_CROSS_BACK'

def inject_waypoint(agent_id, target_wp):
    """
    Segnala al bridge di redirigere l'agente alla prossima chiamata
    del servizio /get_next_destination. One-shot, no race condition.
    """
    override_key = '/hrisim/override/{}/dest'.format(agent_id)
    rospy.set_param(override_key, target_wp)
    rospy.loginfo("[inject] Agente %s: override → %s", agent_id, target_wp)
    return True

    pos = nx.get_node_attributes(G, 'pos')
    closest = min(G.nodes, key=lambda wp: (pos[wp][0] - ax)**2 + (pos[wp][1] - ay)**2)

    if closest == target_wp:
        rospy.loginfo("[inject] Agente %s già a %s", agent_id, target_wp)
        return True

    try:
        path = nx.astar_path(G, closest, target_wp, heuristic=heuristic, weight='weight')
    except nx.NetworkXNoPath:
        rospy.logwarn("[inject] Agente %s: nessun path %s -> %s", agent_id, closest, target_wp)
        return False

    task_duration = {wp: 0 for wp in path}
    task_duration[target_wp] = duration

    agent_data['path'] = path
    agent_data['original_path'] = path[:]
    agent_data['taskDuration'] = task_duration
    agent_data['isStuck'] = False
    rospy.set_param(param_key, agent_data)
    rospy.loginfo("[inject] Agente %s: path iniettato %s -> %s", agent_id, closest, target_wp)
    return True

def inject_agents_by_zone():
    moved = 0
    for agent in LAST_AGENTS:
        aid = str(agent.id)   # assicurati sia stringa come nel rosparam
        ax = agent.pose.position.x
        ay = agent.pose.position.y
        target = _classify_agent(ax, ay)
        if target is None:
            continue
        inject_waypoint(aid, target)
        moved += 1
    rospy.loginfo("[inject] %d override impostati", moved)

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
            ac = actionlib.SimpleActionClient("/play_motion", PlayMotionAction)
            if ac.wait_for_server(timeout=rospy.Duration(2.0)):
                goal = PlayMotionGoal()
                goal.motion_name = "head_tour"
                goal.skip_planning = True
                ac.send_goal(goal)
                ac.wait_for_result(rospy.Duration(5.0))
                rospy.loginfo("[TIAGo] head_tour completato")
            else:
                rospy.logwarn("[TIAGo] /play_motion non disponibile")
        except Exception as e:
            rospy.logwarn("[TIAGo] head_tour fallito: %s", e)

        inject_agents_by_zone()

# ─────────────────────────────────────────────────────────────────────
# MEZZA TRATTA
# ─────────────────────────────────────────────────────────────────────

def run_half(p, episode_num, start, obs_wp, end, direction):
    rospy.loginfo("[TIAGo] ── %s: %s → %s ──", direction, start, end)

    navigate(p, start, obs_wp)
    rospy.sleep(0.5)

    congested = check_congestion(direction)

    if congested:
        rospy.loginfo("[TIAGo] Congesto! Emetto azione e aspetto...")
        A = choose_action(episode_num)
        emit_action(A)
        last_action_time = rospy.Time.now()

        # Breve attesa per far processare il nuovo path dal bridge
        rospy.sleep(2.0)

        while not rospy.is_shutdown():
            rospy.sleep(RECHECK_INTERVAL)
            if not check_congestion(direction):
                rospy.loginfo("[TIAGo] Corridoio libero! Procedo.")
                break
            if A == 1 and (rospy.Time.now() - last_action_time) > rospy.Duration(10.0):
                rospy.loginfo("[TIAGo] Ri-inietto path agenti")
                inject_agents_by_zone()
                last_action_time = rospy.Time.now()
            else:
                rospy.loginfo("[TIAGo] Ancora congesto, aspetto...")
    else:
        rospy.loginfo("[TIAGo] Corridoio libero, procedo direttamente.")

    navigate(p, obs_wp, end)

# ─────────────────────────────────────────────────────────────────────
# PLAN PRINCIPALE
# ─────────────────────────────────────────────────────────────────────

def Plan(p):
    while not ros_utils.wait_for_param("/pnp_ros/ready"):
        rospy.sleep(0.1)

    ros_utils.wait_for_service('/hrisim/new_task')
    ros_utils.wait_for_service('/hrisim/finish_task')
    rospy.set_param('/hrisim/robot_busy', False)
    rospy.set_param("/peopleflow/robot_plan_on", True)

    while ROBOT_CLOSEST_WP is None:
        rospy.loginfo("[TIAGo] Attendo posizione robot...")
        rospy.sleep(0.1)

    rospy.loginfo("[TIAGo] Attendo agenti pedsim...")
    while not rospy.is_shutdown() and not LAST_AGENTS:
        rospy.sleep(0.2)
    rospy.loginfo("[TIAGo] Agenti ricevuti: %d", len(LAST_AGENTS))

    rospy.loginfo("[TIAGo] GAP_MIN=%.2f | LOOK_AHEAD=%.1f | POLICY=%s | RECHECK=%.1fs",
                  GAP_MIN, LOOK_AHEAD, ACTION_POLICY, RECHECK_INTERVAL)

    episode = 0
    while not rospy.is_shutdown():
        episode += 1
        rospy.loginfo("[TIAGo] ══════ EPISODIO %d ══════", episode)

        run_half(p, episode, start=POINT_A, obs_wp=WP_OBS_FWD,
                 end=POINT_B, direction="FWD")
        run_half(p, episode, start=POINT_B, obs_wp=WP_OBS_BWD,
                 end=POINT_A, direction="BWD")

        rospy.sleep(1.0)

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

    rospy.Subscriber("/hrisim/robot_closest_wp",   String,                   cb_robot_closest_wp)
    rospy.Subscriber("/pedsim_simulator/simulated_agents", AgentStates,      cb_agents, queue_size=1)
    rospy.Subscriber("/robot_pose",                PoseWithCovarianceStamped, cb_robot_pose, queue_size=1)

    action_pub = rospy.Publisher("/hrisim/robot_action", Int32, queue_size=1)

    p.begin()
    Plan(p)
    p.end()
