#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import os
import pickle
import random
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
from robot_srvs.srv import NewTask, FinishTask
import hrisim_util.constants as constants
from pedsim_msgs.msg import AgentStates
from geometry_msgs.msg import PoseWithCovarianceStamped
import hrisim_util.ros_utils as ros_utils
import networkx as nx

POINT_A    = "WP_SPAWN"
POINT_B    = "WP_TABLE"
WP_OBS_FWD = "WP_OBS_FWD"
WP_OBS_BWD = "WP_OBS_BWD"
WP_CENTRE  = "WP_CENTRE"

ROBOT_WIDTH       = 0.55
PERSONAL_SPACE    = 0.30
SAFETY_RADIUS     = PERSONAL_SPACE
GAP_MIN           = ROBOT_WIDTH + 2 * PERSONAL_SPACE
S_GAP_MIN         = ROBOT_WIDTH + 0.25
LOOK_AHEAD        = 3.0
LOOK_BEHIND       = 0.5
CORRIDOR_CENTER_Y = 0.0
CORRIDOR_WIDTH    = 2.5
_CROSS_CX         = 0.0
_CROSS_CY         = 0.0
_CROSS_RX         = 1.3
_CROSS_RY         = 1.5

RECHECK_INTERVAL   = 2.0
ACTION_POLICY      = "random"   # "random" | "always_act" | "never_act"
P_ACTION           = 0.65       # P(A=1 | congestione) con policy "random"
WAIT_CLEAR_TIMEOUT = 30.0       # max attesa (s) che il corridoio si liberi
ARRIVAL_TOLERANCE  = 0.8        # distanza massima (m) per considerare il robot arrivato

HOME = {
    '0': 'WP_POSTER_L',
    '1': 'WP_POSTER_R',
    '2': 'WP_CROSS',
    '3': 'WP_CROSS',
    '4': 'WP_CROSS',
}

LAST_AGENTS      = []
LAST_OBSTACLES   = []
ROBOT_CLOSEST_WP = None
ROBOT_XY         = (None, None)
action_pub       = None
G                = None
TIME_THRESHOLD   = None
_injected_agents = set()
CURRENT_O        = 0

def cb_agents(msg):
    global LAST_AGENTS
    LAST_AGENTS = [a for a in msg.agent_states if a.type == 1]
    
def cb_obstacles(msg):
    global LAST_OBSTACLES
    if msg.data:
        LAST_OBSTACLES = [tuple(map(float, p.split(","))) for p in msg.data.split(";")]
    else:
        LAST_OBSTACLES = []

def cb_robot_closest_wp(msg):
    global ROBOT_CLOSEST_WP
    ROBOT_CLOSEST_WP = msg.data

def cb_robot_pose(msg):
    global ROBOT_XY
    ROBOT_XY = (msg.pose.pose.position.x, msg.pose.pose.position.y)

def cb_O(msg):
    global CURRENT_O
    CURRENT_O = msg.data

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

def _reached(target_wp):
    rx, ry = ROBOT_XY
    if rx is None:
        rospy.logwarn("[TIAGo] Posizione robot non disponibile, assumo NON arrivato.")
        return False
    pos = nx.get_node_attributes(G, 'pos')
    tx, ty = pos[target_wp]
    dist = math.sqrt((rx - tx)**2 + (ry - ty)**2)
    rospy.loginfo("[TIAGo] Distanza da %s: %.2f m (tol=%.2f)", target_wp, dist, ARRIVAL_TOLERANCE)
    return dist <= ARRIVAL_TOLERANCE

def _agents_in_window(direction):
    """Agenti dentro la finestra di congestione (None se la posa del robot manca)."""
    rx, ry = ROBOT_XY
    if rx is None:
        return None
    y_min = CORRIDOR_CENTER_Y - CORRIDOR_WIDTH / 2.0
    y_max = CORRIDOR_CENTER_Y + CORRIDOR_WIDTH / 2.0
    if direction == "FWD":
        x_min, x_max = rx - LOOK_BEHIND, rx + LOOK_AHEAD
    else:
        x_min, x_max = rx - LOOK_AHEAD, rx + LOOK_BEHIND
    return [
        a for a in LAST_AGENTS
        if x_min <= a.pose.position.x <= x_max
        and y_min <= a.pose.position.y <= y_max
    ]
    
def _approaching_cross(ax, ay):
    return abs(ax - _CROSS_CX) <= _CROSS_RX and -4.0 <= ay < -_CROSS_RY

def check_congestion(direction):
    relevant = _agents_in_window(direction)
    if relevant is None:
        rospy.logwarn("[TIAGo] Posizione robot non disponibile, assumo LIBERO.")
        return False

    rx, ry = ROBOT_XY
    y_min = CORRIDOR_CENTER_Y - CORRIDOR_WIDTH / 2.0
    y_max = CORRIDOR_CENTER_Y + CORRIDOR_WIDTH / 2.0
    if direction == "FWD":
        x_min, x_max = rx - LOOK_BEHIND, rx + LOOK_AHEAD
    else:
        x_min, x_max = rx - LOOK_AHEAD, rx + LOOK_BEHIND

    obstacles = [("wall_low", y_min - 0.01, y_min)]
    for a in relevant:
        py = a.pose.position.y
        obstacles.append(("p{}".format(a.id), py - SAFETY_RADIUS, py + SAFETY_RADIUS))
    for (ox, oy) in LAST_OBSTACLES:
        if x_min <= ox <= x_max and y_min <= oy <= y_max:
            obstacles.append(("o{}".format(len(obstacles)), oy - SAFETY_RADIUS, oy + SAFETY_RADIUS))
    obstacles.append(("wall_high", y_max, y_max + 0.01))

    if len(obstacles) == 2:
        rospy.loginfo("[TIAGo] Corridoio LIBERO (nessun agente/ostacolo)")
        return False

    obstacles.sort(key=lambda o: o[1])
    best_gap = max(
        obstacles[i+1][1] - obstacles[i][2]
        for i in range(len(obstacles) - 1)
    )
    congested = best_gap < GAP_MIN
    rospy.loginfo("[TIAGo] [%s] %d agenti+ostacoli | gap_max=%.3f m -> %s",
                  direction, len(obstacles) - 2, best_gap,
                  "BLOCCATO" if congested else "LIBERO")
    return congested
    
def check_congestion_S(direction):
    relevant = _agents_in_window(direction)
    if relevant is None:
        return False

    rx, ry = ROBOT_XY
    y_min = CORRIDOR_CENTER_Y - CORRIDOR_WIDTH / 2.0
    y_max = CORRIDOR_CENTER_Y + CORRIDOR_WIDTH / 2.0
    if direction == "FWD":
        x_min, x_max = rx - LOOK_BEHIND, rx + LOOK_AHEAD
    else:
        x_min, x_max = rx - LOOK_AHEAD, rx + LOOK_BEHIND

    obstacles = [("wall_low", y_min - 0.01, y_min)]
    for a in relevant:
        py = a.pose.position.y
        obstacles.append(("p{}".format(a.id), py - SAFETY_RADIUS, py + SAFETY_RADIUS))
    for (ox, oy) in LAST_OBSTACLES:
        if x_min <= ox <= x_max and y_min <= oy <= y_max:
            obstacles.append(("o{}".format(len(obstacles)), oy - SAFETY_RADIUS, oy + SAFETY_RADIUS))
    obstacles.append(("wall_high", y_max, y_max + 0.01))

    if len(obstacles) == 2:
        return False

    obstacles.sort(key=lambda o: o[1])
    best_gap = max(
        obstacles[i+1][1] - obstacles[i][2]
        for i in range(len(obstacles) - 1)
    )
    return best_gap < S_GAP_MIN
    
def check_congestion_people(direction):
    relevant = _agents_in_window(direction)
    if relevant is None:
        return False
    if not relevant:
        return False

    y_min = CORRIDOR_CENTER_Y - CORRIDOR_WIDTH / 2.0
    y_max = CORRIDOR_CENTER_Y + CORRIDOR_WIDTH / 2.0

    obstacles = [("wall_low", y_min - 0.01, y_min)]
    for a in relevant:
        py = a.pose.position.y
        obstacles.append(("p{}".format(a.id), py - SAFETY_RADIUS, py + SAFETY_RADIUS))
    obstacles.append(("wall_high", y_max, y_max + 0.01))
    obstacles.sort(key=lambda o: o[1])

    best_gap = max(
        obstacles[i+1][1] - obstacles[i][2]
        for i in range(len(obstacles) - 1)
    )
    return best_gap < GAP_MIN

def _in_cross_zone(ax, ay):
    return abs(ax - _CROSS_CX) <= _CROSS_RX and abs(ay - _CROSS_CY) <= _CROSS_RY

def set_hold(agent_id, target_wp):
    rospy.set_param('/hrisim/hold/{}/dest'.format(agent_id), target_wp)
    rospy.loginfo("[inject] HOLD agente %s -> %s", agent_id, target_wp)

def clear_hold(agent_id):
    try:
        rospy.delete_param('/hrisim/hold/{}/dest'.format(agent_id))
    except KeyError:
        pass

def inject_congested_agents(direction):
    global _injected_agents
    _injected_agents.clear()
    window = _agents_in_window(direction) or []
    window_ids = {str(a.id) for a in window}
    rospy.logwarn("[inject] window_ids=%s", window_ids)
    rospy.logwarn("[inject] LAST_AGENTS ids=%s", [str(a.id) for a in LAST_AGENTS])
    for agent in LAST_AGENTS:
        aid = str(agent.id)
        ax, ay = agent.pose.position.x, agent.pose.position.y
        in_win  = aid in window_ids
        in_zone = _in_cross_zone(ax, ay)
        approaching = _approaching_cross(ax, ay)
        rospy.logwarn("[inject] agente %s | in_window=%s | in_zone=%s", aid, aid in window_ids, in_zone)
        if not (in_win or in_zone or approaching):
            continue
        if approaching and not (in_win or in_zone):
            evade_wp = 'WP_BOTTOM'
        else:
            evade_wp = random.choice(['WP_CROSS_BACK', 'WP_BOTTOM'])
        set_hold(aid, evade_wp)
        _injected_agents.add(aid)
    rospy.logwarn("[inject] _injected_agents=%s", _injected_agents)
    for aid in _injected_agents:
        val = rospy.get_param('/hrisim/hold/{}/dest'.format(aid), 'NON_TROVATO')
        rospy.logwarn("[inject] param hold agente %s = %s", aid, val)

def release_and_send_home():
    for aid in _injected_agents:
        clear_hold(aid)
        home = HOME.get(aid, 'WP_CROSS')
        rospy.set_param('/hrisim/hold/{}/dest'.format(aid), home)
        rospy.loginfo("[inject] Agente %s hold -> %s", aid, home)

def wait_for_agents_home(timeout=60.0):
    if not _injected_agents:
        rospy.loginfo("[TIAGo] Nessun agente da aspettare")
        return True
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
            rospy.loginfo("[TIAGo] Tutti gli agenti tornati")
            return True
        rate.sleep()
    rospy.logwarn("[TIAGo] Timeout wait_for_agents_home")
    return False
    
def clear_home_holds():
    for aid in _injected_agents:
        clear_hold(aid)
        rospy.loginfo("[inject] Hold home rimosso per agente %s", aid)

def choose_action():
    if ACTION_POLICY == "always_act":
        return 1
    elif ACTION_POLICY == "never_act":
        return 0
    else:
        return 1 if random.random() < P_ACTION else 0

def emit_action(action_val, direction):
    action_pub.publish(Int32(action_val))
    rospy.loginfo("[TIAGo] A=%d pubblicata", action_val)
    if action_val == 1:
        inject_congested_agents(direction)
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

def run_half(p, episode_num, start, obs_wp, end, direction,
             episode_start_pub, episode_end_pub, people_cleared_pub, transit_pub,  
             Pi_pub, Pe_pub, S_pub, T_pub, robot_arrived_pub, new_task_srv, finish_task_srv):
    rospy.loginfo("[TIAGo] ── EPISODIO %d | %s: %s -> %s ──", episode_num, direction, start, end)

    episode_start_pub.publish(Int32(episode_num))
    task_resp = new_task_srv(path=[start, end], final_destination=end)
    task_id   = task_resp.task_id
    navigate(p, start, obs_wp)
    rospy.sleep(0.5)

    congested = check_congestion(direction)
    Pi = 1 if check_congestion_people(direction) else 0
    Pi_pub.publish(Int32(Pi))

    A = choose_action() if congested else 0
    emit_action(A, direction)            
    if A == 1:
        rospy.sleep(2.0)                 

    Pe = 1 if check_congestion_people(direction) else 0
    Pe_pub.publish(Int32(Pe))            

    if congested:
        if A == 0:
            rospy.sleep(2.0)
        if check_congestion(direction):
            rospy.loginfo("[TIAGo] Ancora congested, aspetto 10s e provo comunque...")
            rospy.sleep(10.0)
        else:
            people_cleared_pub.publish(Bool(True))

    S_final = 0 if check_congestion_S(direction) else 1
    S_pub.publish(Int32(S_final))
    transit_pub.publish(Bool(True))
    navigate(p, obs_wp, end)

    T = 1 if _reached(end) else 0
    T_pub.publish(Int32(T))
    rospy.sleep(0.3)
    robot_arrived_pub.publish(Bool(True))
    rospy.loginfo("[TIAGo] Episodio %d | T=%d", episode_num, T)

    if T == 0:
        finish_task_srv(task_id=task_id, result=constants.TaskResult.FAILURE.value)
        release_and_send_home()
        wait_for_agents_home()
        clear_home_holds()
        _injected_agents.clear()
        rospy.logwarn("[TIAGo] Episodio %d FALLITO — recovery verso %s", episode_num, start)
        recovery_wp = ROBOT_CLOSEST_WP
        navigate(p, recovery_wp, start)
        episode_end_pub.publish(Int32(episode_num))
        rospy.loginfo("[TIAGo] Recovery completato, robot a %s", start)
        return False

    finish_task_srv(task_id=task_id, result=constants.TaskResult.SUCCESS.value)
    release_and_send_home()
    wait_for_agents_home()
    clear_home_holds()
    _injected_agents.clear()
    episode_end_pub.publish(Int32(episode_num))
    rospy.sleep(1.0)
    return True

def Plan(p):
    while not ros_utils.wait_for_param("/pnp_ros/ready"):
        rospy.sleep(0.1)
    ros_utils.wait_for_service('/hrisim/new_task')
    ros_utils.wait_for_service('/hrisim/finish_task')
    new_task_srv    = rospy.ServiceProxy('/hrisim/new_task',    NewTask)
    finish_task_srv = rospy.ServiceProxy('/hrisim/finish_task', FinishTask)
    rospy.set_param('/hrisim/robot_busy', False)
    rospy.set_param("/peopleflow/robot_plan_on", True)

    while ROBOT_CLOSEST_WP is None:
        rospy.sleep(0.1)
    while not rospy.is_shutdown() and not LAST_AGENTS:
        rospy.sleep(0.2)

    rospy.loginfo("[TIAGo] Pronti | GAP_MIN=%.2f | LOOK_AHEAD=%.1f | POLICY=%s | P_ACTION=%.2f",
                  GAP_MIN, LOOK_AHEAD, ACTION_POLICY, P_ACTION)

    episode_start_pub = rospy.Publisher("/hrisim/episode_start", Int32, queue_size=1, latch=True)
    episode_end_pub    = rospy.Publisher("/hrisim/episode_end",    Int32, queue_size=1)
    people_cleared_pub = rospy.Publisher("/hrisim/people_cleared", Bool,  queue_size=1)
    transit_pub        = rospy.Publisher("/hrisim/robot_transit",  Bool,  queue_size=1)
    robot_arrived_pub  = rospy.Publisher("/hrisim/robot_arrived", Bool, queue_size=1)
    S_pub              = rospy.Publisher("/hrisim/obs/S", Int32, queue_size=1)
    T_pub              = rospy.Publisher("/hrisim/obs/T", Int32, queue_size=1)
    Pi_pub = rospy.Publisher("/hrisim/obs/Pi", Int32, queue_size=1)
    Pe_pub = rospy.Publisher("/hrisim/obs/Pe", Int32, queue_size=1)
    rospy.sleep(0.5)

    current_pos = POINT_A  # posizione logica iniziale

    episode = 0
    while not rospy.is_shutdown():
        if current_pos == POINT_A:
            next_pos  = POINT_B
            obs_wp    = WP_OBS_FWD
            direction = "FWD"
        else:
            next_pos  = POINT_A
            obs_wp    = WP_OBS_BWD
            direction = "BWD"

        episode += 1
        success = run_half(
            p, episode,
            start=current_pos, obs_wp=obs_wp, end=next_pos, direction=direction,
            episode_start_pub=episode_start_pub, episode_end_pub=episode_end_pub,
            people_cleared_pub=people_cleared_pub, transit_pub=transit_pub,
            Pi_pub=Pi_pub, Pe_pub=Pe_pub,
            S_pub=S_pub, T_pub=T_pub, robot_arrived_pub=robot_arrived_pub,
            new_task_srv=new_task_srv, finish_task_srv=finish_task_srv
        )

        if success:
            current_pos = next_pos

    rospy.set_param("/peopleflow/robot_plan_on", False)

if __name__ == "__main__":
    p = PNPCmd()

    g_path = ros_utils.wait_for_param("/peopleflow_pedsim_bridge/g_path")
    with open(g_path, 'rb') as f:
        G = pickle.load(f)

    TIME_THRESHOLD = ros_utils.wait_for_param("/hrisim/abort_time_threshold")

    rospy.Subscriber("/hrisim/robot_closest_wp",           String,                    cb_robot_closest_wp)
    rospy.Subscriber("/pedsim_simulator/simulated_agents", AgentStates,               cb_agents, queue_size=1)
    rospy.Subscriber("/robot_pose",                        PoseWithCovarianceStamped, cb_robot_pose, queue_size=1)
    rospy.Subscriber("/hrisim/obs/O", Int32, cb_O)
    rospy.Subscriber("/hrisim/obstacles/positions", String, cb_obstacles)

    action_pub = rospy.Publisher("/hrisim/robot_action", Int32, queue_size=1)

    p.begin()
    Plan(p)
    p.end()
