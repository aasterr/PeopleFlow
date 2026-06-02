#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
obstacle_policy.py
==================
Gestisce il confounder O (singolo nodo binario nel DAG causale).

EPISODIO = singola traversata (SPAWN->TABLE oppure TABLE->SPAWN).

  A episode_start:
    - Campiona N ~ Uniform(0, N_max) (intero)
    - Spawna N oggetti in posizioni random nella zona di congestione
    - O = 1 se N > 0, O = 0 se N == 0

  A episode_end:
    - Loga il valore finale di O e N
    - Rimuove tutti gli ostacoli
    - Resetta lo stato

Parametri ROS (privati, passare con _param:=val):
  ~n_max      int    [4]     numero massimo di oggetti spawnabili
  ~zone_cx    float  [0.0]   centro x zona di congestione  (= WP_CENTRE x)
  ~zone_cy    float  [0.0]   centro y zona di congestione  (= WP_CENTRE y)
  ~zone_rx    float  [1.35]  semi-larghezza zona sull'asse X  (meta' corridoio 2.7m)
  ~zone_ry    float  [0.8]   semi-altezza zona sull'asse Y  (da -0.8 a +0.8)

  ~min_dist   float  [0.8]   distanza minima da un agente per spawmare un ostacolo

Topic:
  SUB /hrisim/episode_start  (Int32)
  SUB /hrisim/episode_end    (Int32)
  SUB /pedsim_simulator/simulated_agents (AgentStates)
  PUB /hrisim/obstacles/spawn   (String)  →  "ID:x:y"
  PUB /hrisim/obstacles/remove  (String)  →  "ALL"
"""

import rospy
import random
import math
from std_msgs.msg import Int32, String
from pedsim_msgs.msg import AgentStates

# ── Publisher globali ────────────────────────────────────────────────
spawn_pub  = None
remove_pub = None

# ── Parametri (aggiornati in main) ───────────────────────────────────
N_MAX    = 8
MIN_DIST = 0.6  # distanza minima da agenti per spawn ostacolo

# ── Snapshot agenti ──────────────────────────────────────────────────
_last_agents = {}   # {agent_id (int) -> (x, y)}

# ── Stato episodio ───────────────────────────────────────────────────
_n_spawned = 0   # numero di oggetti spawnati nell'episodio corrente
_spawned_positions = []   # lista di (x, y) degli ostacoli già spawnati

# ─────────────────────────────────────────────────────────────────────
# HELPERS
# ─────────────────────────────────────────────────────────────────────

def _too_close_to_agent(x, y, min_dist=None):
    d = min_dist or MIN_DIST
    for (ax, ay) in _last_agents.values():
        if math.sqrt((x - ax) ** 2 + (y - ay) ** 2) < d:
            return True
    return False

def _too_close_to_obstacle(x, y, min_dist=0.5):
    """Evita che due ostacoli si sovrappongano."""
    for (ox, oy) in _spawned_positions:
        if math.sqrt((x - ox) ** 2 + (y - oy) ** 2) < min_dist:
            return True
    return False
    
def _get_fresh_agents(timeout=0.5):
    """
    Aspetta che lo snapshot sia stato aggiornato almeno una volta
    entro il timeout, sfruttando il subscriber permanente cb_agents.
    """
    deadline = rospy.Time.now() + rospy.Duration(timeout)
    rate = rospy.Rate(20)
    while rospy.Time.now() < deadline:
        if _last_agents:  # snapshot già ricevuto almeno una volta
            return
        rate.sleep()
    rospy.logwarn("[ObstaclePolicy] Snapshot agenti non disponibile entro %.1fs", timeout)

    sub = rospy.Subscriber(
        "/pedsim_simulator/simulated_agents", AgentStates, _cb)
    deadline = rospy.Time.now() + rospy.Duration(timeout)
    rate = rospy.Rate(20)
    while not received[0] and rospy.Time.now() < deadline:
        rate.sleep()
    sub.unregister()

    if not received[0]:
        rospy.logwarn("[ObstaclePolicy] Snapshot agenti non aggiornato entro %.1fs", timeout)

def _random_position_in_zone(max_attempts=50):
    cx = rospy.get_param("~zone_cx",  0.0)
    cy = rospy.get_param("~zone_cy",  0.0)
    rx = rospy.get_param("~zone_rx",  1.35)
    ry = rospy.get_param("~zone_ry",  0.8)
    for _ in range(max_attempts):
        x = random.uniform(cx - rx, cx + rx)
        y = random.uniform(cy - ry, cy + ry)
        if _too_close_to_agent(x, y):
            continue
        if _too_close_to_obstacle(x, y):
            continue
        return x, y
    rospy.logwarn("[ObstaclePolicy] Nessuna posizione libera dopo %d tentativi.", max_attempts)
    return None

def _spawn(obs_id, x, y):
    payload = "{}:{:.3f}:{:.3f}".format(obs_id, x, y)
    spawn_pub.publish(String(payload))
    rospy.loginfo("[ObstaclePolicy] Spawn: %s", payload)

def _remove_all():
    remove_pub.publish(String("ALL"))
    rospy.loginfo("[ObstaclePolicy] Remove ALL")

def _reset_episode():
    global _n_spawned, _spawned_positions
    _n_spawned = 0
    _spawned_positions = []

def _compute_O():
    return 1 if _n_spawned > 0 else 0

# ─────────────────────────────────────────────────────────────────────
# CALLBACKS
# ─────────────────────────────────────────────────────────────────────

def cb_agents(msg):
    """Aggiorna snapshot posizioni di tutti gli agenti continuamente."""
    global _last_agents
    for agent in msg.agent_states:
        _last_agents[agent.id] = (
            agent.pose.position.x,
            agent.pose.position.y
        )

def cb_episode_start(msg):
    global _n_spawned, _spawned_positions
    episode_num = msg.data

    # Snapshot fresco prima di campionare
    _get_fresh_agents(timeout=0.5)

    n = random.randint(0, N_MAX)
    rospy.loginfo(
        "[ObstaclePolicy] ── Episodio %d START | N_intended=%d ──",
        episode_num, n
    )

    actually_spawned = 0
    for i in range(n):
        pos = _random_position_in_zone()
        if pos is None:
            continue
        obs_id = "O_{}".format(i)
        _spawn(obs_id, pos[0], pos[1])
        _spawned_positions.append(pos)   # traccia posizione per controllo inter-ostacolo
        actually_spawned += 1

    _n_spawned = actually_spawned
    rospy.loginfo("[ObstaclePolicy] Spawnati %d/%d | O=%d", actually_spawned, n, _compute_O())

def cb_episode_end(msg):
    """
    Fine episodio: loga O finale, rimuove tutti gli ostacoli, resetta stato.
    """
    o_final = _compute_O()

    rospy.loginfo(
        "[ObstaclePolicy] ── Episodio %d END | O=%d N=%d ──",
        msg.data, o_final, _n_spawned
    )

    # Valore finale del confounder — decommentare quando il data logger e' pronto
    # rospy.set_param('/hrisim/episode_O', o_final)
    # rospy.set_param('/hrisim/episode_N', _n_spawned)

    _remove_all()
    _reset_episode()

# ─────────────────────────────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    rospy.init_node("obstacle_policy_node")

    N_MAX    = int(rospy.get_param("~n_max",    N_MAX))
    MIN_DIST = float(rospy.get_param("~min_dist", MIN_DIST))

    rospy.loginfo("[ObstaclePolicy] Avviato | n_max=%d | min_dist=%.2f", N_MAX, MIN_DIST)
    rospy.loginfo(
        "[ObstaclePolicy] Zona congestione: cx=%.1f cy=%.1f rx=%.1f ry=%.1f",
        rospy.get_param("~zone_cx",  0.0),
        rospy.get_param("~zone_cy",  0.0),
        rospy.get_param("~zone_rx",  1.35),
        rospy.get_param("~zone_ry",  0.8)
    )

    spawn_pub  = rospy.Publisher("/hrisim/obstacles/spawn",  String, queue_size=10)
    remove_pub = rospy.Publisher("/hrisim/obstacles/remove", String, queue_size=5)
    rospy.sleep(0.5)

    rospy.Subscriber("/hrisim/episode_start",              Int32,       cb_episode_start)
    rospy.Subscriber("/hrisim/episode_end",                Int32,       cb_episode_end)
    rospy.Subscriber("/pedsim_simulator/simulated_agents", AgentStates, cb_agents)

    rospy.spin()
