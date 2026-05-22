#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Loop: SPAWN -> TABLE -> SPAWN -> TABLE -> ...
Ogni singola traversata (SPAWN->TABLE o TABLE->SPAWN) e' un EPISODIO autonomo.
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
from std_msgs.msg import String, Int32, Bool
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
_CROSS_CX = 0.0
_CROSS_CY = -0.9
_CROSS_RX = 1.5   # semi-larghezza asse X
_CROSS_RY = 1.5   # semi-altezza  asse Y

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
        rospy.loginfo("[TIAGo] Gia' a destinazione %s.", end)
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
# INIEZIONE PATH AGENTI
# ─────────────────────────────────────────────────────────────────────

def _dist2d(x1, y1, x2, y2):
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

def _in_cross_zone(ax, ay):
    """True se l'agente e' nel rettangolo CROSS."""
    return abs(ax - _CROSS_CX) <= _CROSS_RX and abs(ay - _CROSS_CY) <= _CROSS_RY

def inject_waypoint(agent_id, target_wp):
    """
    Segnala al bridge di redirigere l'agente alla prossima chiamata
    del servizio /get_next_destination. One-shot, no race condition.
    """
    override_key = '/hrisim/override/{}/dest'.format(agent_id)
    rospy.set_param(override_key, target_wp)
    rospy.loginfo("[inject] Agente %s: override -> %s", agent_id, target_wp)
    return True

def inject_agents_by_zone():
    """
    Inietta override WP_CROSS_BACK SOLO agli agenti attualmente
    dentro la zona CROSS. Gli agenti fuori zona non vengono toccati.
    """
    moved = 0
    for agent in LAST_AGENTS:
        ax = agent.pose.position.x
        ay = agent.pose.position.y
        if not _in_cross_zone(ax, ay):
            rospy.loginfo("[inject] Agente %s fuori zona CROSS (%.2f, %.2f), skip",
                          agent.id, ax, ay)
            continue
        inject_waypoint(str(agent.id), 'WP_CROSS_BACK')
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
# MEZZA TRATTA  (= un episodio)
# ─────────────────────────────────────────────────────────────────────

def run_half(p, episode_num, start, obs_wp, end, direction,
             episode_start_pub, episode_end_pub, people_cleared_pub, transit_pub):
    """
    Esegue una singola traversata e la tratta come episodio autonomo:
      1. Pubblica episode_start
      2. Naviga fino a obs_wp, controlla congestione, eventuale azione HRI
      3. Pubblica robot_transit e procede verso end
      4. Pubblica episode_end  ← gli ostacoli vengono rimossi qui da obstacle_policy
    """
    rospy.loginfo("[TIAGo] ══════ EPISODIO %d | %s: %s -> %s ══════",
                  episode_num, direction, start, end)

    episode_start_pub.publish(Int32(episode_num))

    navigate(p, start, obs_wp)
    rospy.sleep(0.5)

    congested = check_congestion(direction)
    if congested:
        rospy.loginfo("[TIAGo] Congesto! Emetto azione e aspetto...")
        A = choose_action(episode_num)
        emit_action(A)
        last_action_time = rospy.Time.now()
        rospy.sleep(2.0)
        while not rospy.is_shutdown():
            rospy.sleep(RECHECK_INTERVAL)
            if not check_congestion(direction):
                rospy.loginfo("[TIAGo] Corridoio libero! Procedo.")
                people_cleared_pub.publish(Bool(True))
                break
            if A == 1 and (rospy.Time.now() - last_action_time) > rospy.Duration(10.0):
                rospy.loginfo("[TIAGo] Ri-inietto path agenti")
                inject_agents_by_zone()
                last_action_time = rospy.Time.now()
            else:
                rospy.loginfo("[TIAGo] Ancora congesto, aspetto...")
    else:
        rospy.loginfo("[TIAGo] Corridoio libero, procedo direttamente.")

    transit_pub.publish(Bool(True))
    navigate(p, obs_wp, end)

    episode_end_pub.publish(Int32(episode_num))
    rospy.sleep(1.0)   # piccola pausa tra episodi per dare tempo al remove in Gazebo

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

    episode_start_pub  = rospy.Publisher("/hrisim/episode_start",  Int32, queue_size=1)
    episode_end_pub    = rospy.Publisher("/hrisim/episode_end",    Int32, queue_size=1)
    people_cleared_pub = rospy.Publisher("/hrisim/people_cleared", Bool,  queue_size=1)
    transit_pub        = rospy.Publisher("/hrisim/robot_transit",  Bool,  queue_size=1)
    rospy.sleep(0.5)

    episode = 0
    while not rospy.is_shutdown():
        # Traversata FWD: SPAWN -> TABLE  (episodio dispari)
        episode += 1
        run_half(p, episode,
                 start=POINT_A, obs_wp=WP_OBS_FWD, end=POINT_B, direction="FWD",
                 episode_start_pub=episode_start_pub, episode_end_pub=episode_end_pub,
                 people_cleared_pub=people_cleared_pub, transit_pub=transit_pub)

        # Traversata BWD: TABLE -> SPAWN  (episodio pari)
        episode += 1
        run_half(p, episode,
                 start=POINT_B, obs_wp=WP_OBS_BWD, end=POINT_A, direction="BWD",
                 episode_start_pub=episode_start_pub, episode_end_pub=episode_end_pub,
                 people_cleared_pub=people_cleared_pub, transit_pub=transit_pub)

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

    rospy.Subscriber("/hrisim/robot_closest_wp",              String,                    cb_robot_closest_wp)
    rospy.Subscriber("/pedsim_simulator/simulated_agents",    AgentStates,               cb_agents, queue_size=1)
    rospy.Subscriber("/robot_pose",                           PoseWithCovarianceStamped, cb_robot_pose, queue_size=1)

    action_pub = rospy.Publisher("/hrisim/robot_action", Int32, queue_size=1)

    p.begin()
    Plan(p)
    p.end()
