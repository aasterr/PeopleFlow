#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
obstacle_policy.py
==================
Gestisce la policy probabilistica del confounder O (O1 + O2).

EPISODIO = singola traversata (SPAWN->TABLE oppure TABLE->SPAWN).
  - O1 spawna a episode_start, viene rimosso a episode_end.
  - O2 spawna al robot_transit, viene rimosso a episode_end.
  - Il reset dello stato avviene solo a episode_end (dopo remove).

O1 — ostacolo fisso, effetto indiretto su A:
  - Spawna nel corridoio a inizio episodio con probabilita' p1
  - Restringe spazio fisico -> Social Force Model -> influenza congestione percepita -> P(A=1)

O2 — zaini/oggetti, effetto diretto su S e T:
  - Quando A=1 viene emesso, cattura TUTTI gli agenti presenti nella zona CROSS
    (rettangolo configurabile attorno a WP_CROSS = (0.0, -0.9))
    NON filtra per ID: qualsiasi agente fisicamente presente in zona puo' lasciare un oggetto
  - Spawna un oggetto nella posizione salvata quando il robot inizia il transito
    (/hrisim/robot_transit), con probabilita' p2 per agente
  - Rimossi tutti a fine episodio

O = 1 se almeno uno tra O1 e O2 e' presente nell'episodio.

Parametri ROS (privati, passare con _param:=val):
  ~p1         float  [0.5]   prob ostacolo fisso O1
  ~p2         float  [0.5]   prob zaino O2 per agente in zona CROSS
  ~o1_x       float  [0.0]   posizione x ostacolo O1
  ~o1_y       float  [1.0]   posizione y ostacolo O1
  ~cross_cx   float  [0.0]   centro x zona CROSS  (= WP_CROSS x)
  ~cross_cy   float  [-0.9]  centro y zona CROSS  (= WP_CROSS y)
  ~cross_rx   float  [1.5]   semi-larghezza zona CROSS sull'asse X
  ~cross_ry   float  [1.5]   semi-altezza  zona CROSS sull'asse Y

Avvio esempio:
  python obstacle_policy.py _p1:=0.5 _p2:=0.5 _o1_x:=0.0 _o1_y:=0.8
"""

import rospy
import random
from std_msgs.msg import Bool, Int32, String
from pedsim_msgs.msg import AgentStates

# ── Publisher globali ────────────────────────────────────────────────
spawn_pub  = None
remove_pub = None

# ── Parametri (aggiornati in main) ───────────────────────────────────
P1 = 0.5
P2 = 0.5

# ── Stato episodio ───────────────────────────────────────────────────
_o1_active   = False   # True se O1 e' stato spawnato in questo episodio
_o2_spawned  = []      # lista ID oggetti spawnati es. ["O2_3", "O2_4"]
_o2_pending  = {}      # {agent_id (int) -> (x, y)} — posizioni al momento di A=1
_action_done = False   # True se A=1 e' gia' stato registrato in questo episodio

# ── Snapshot agenti (TUTTI, aggiornato continuamente) ────────────────
_last_agents = {}      # {agent_id (int) -> (x, y)}

# ─────────────────────────────────────────────────────────────────────
# GEOMETRIA ZONA CROSS
# ─────────────────────────────────────────────────────────────────────

def _in_cross_zone(x, y):
    """
    Restituisce True se il punto (x, y) e' dentro il rettangolo CROSS.
    Parametri letti da rosparam (modificabili a runtime senza riavvio).
    """
    cx = rospy.get_param("~cross_cx",  0.0)
    cy = rospy.get_param("~cross_cy", -0.9)
    rx = rospy.get_param("~cross_rx",  1.5)
    ry = rospy.get_param("~cross_ry",  1.5)
    return abs(x - cx) <= rx and abs(y - cy) <= ry

# ─────────────────────────────────────────────────────────────────────
# HELPERS
# ─────────────────────────────────────────────────────────────────────

def _spawn(obs_id, x, y):
    payload = "{}:{:.3f}:{:.3f}".format(obs_id, x, y)
    spawn_pub.publish(String(payload))
    rospy.loginfo("[ObstaclePolicy] Spawn richiesto: %s", payload)

def _remove_all():
    remove_pub.publish(String("ALL"))
    rospy.loginfo("[ObstaclePolicy] Remove ALL")

def _reset_episode():
    global _o1_active, _o2_spawned, _o2_pending, _action_done
    _o1_active   = False
    _o2_spawned  = []
    _o2_pending  = {}
    _action_done = False

def _compute_O():
    return 1 if (_o1_active or len(_o2_spawned) > 0) else 0

# ─────────────────────────────────────────────────────────────────────
# CALLBACKS
# ─────────────────────────────────────────────────────────────────────

def cb_agents(msg):
    """
    Aggiorna snapshot posizioni di TUTTI gli agenti continuamente.
    Nessun filtro per ID: l'ID e' un intero come arriva dal topic.
    """
    global _last_agents
    for agent in msg.agent_states:
        _last_agents[agent.id] = (
            agent.pose.position.x,
            agent.pose.position.y
        )

def cb_episode_start(msg):
    """
    Inizio episodio (= inizio singola traversata): spawn eventuale O1.
    NON fa remove qui — gli ostacoli del precedente episodio sono gia'
    stati rimossi da cb_episode_end. Il reset stato e' gia' avvenuto li'.
    """
    global _o1_active
    episode_num = msg.data

    # Piccolo sleep per dare tempo ai publisher di essere pronti
    # (race condition nota tra avvio nodo e primo episodio)
    rospy.sleep(0.3)

    o1 = 1 if random.random() < P1 else 0
    rospy.loginfo("[ObstaclePolicy] ── Episodio %d START | O1=%d (p1=%.2f) ──",
                  episode_num, o1, P1)
    if o1 == 1:
        o1_x = rospy.get_param("~o1_x", 0.0)
        o1_y = rospy.get_param("~o1_y", 1.0)
        _spawn("O1", o1_x, o1_y)
        _o1_active = True

def cb_robot_action(msg):
    """
    Quando A=1 viene emesso, cattura le posizioni correnti di TUTTI
    gli agenti che si trovano nella zona CROSS come candidati per O2.
    Nessun ID hard-coded: e' la posizione fisica che conta.
    """
    global _o2_pending, _action_done

    if msg.data != 1:
        return
    if _action_done:
        return  # gia' registrato in questo episodio

    _action_done = True
    _o2_pending  = {}

    for aid, (x, y) in _last_agents.items():
        if _in_cross_zone(x, y):
            _o2_pending[aid] = (x, y)
            rospy.loginfo("[ObstaclePolicy] Agente %d in zona CROSS: (%.2f, %.2f)", aid, x, y)

    rospy.loginfo("[ObstaclePolicy] %d agenti candidati per O2", len(_o2_pending))

    if not _o2_pending:
        rospy.logwarn("[ObstaclePolicy] Nessun agente in zona CROSS al momento di A=1.")

def cb_robot_transit(msg):
    """
    Quando il robot inizia il transito, spawna O2 per ogni agente
    in _o2_pending con probabilita' p2.
    Chiamato SOLO dopo A=1 (altrimenti _o2_pending e' vuoto).
    """
    global _o2_spawned

    if not msg.data:
        return

    if not _o2_pending:
        rospy.loginfo("[ObstaclePolicy] Nessuna posizione O2 in attesa (A=0 o zona vuota), skip.")
        return

    for aid, (x, y) in list(_o2_pending.items()):
        roll = random.random()
        rospy.loginfo("[ObstaclePolicy] O2 agente %d: roll=%.2f (p2=%.2f)", aid, roll, P2)
        if roll < P2:
            obs_id = "O2_{}".format(aid)
            _spawn(obs_id, x, y)
            _o2_spawned.append(obs_id)

    rospy.loginfo("[ObstaclePolicy] O2 spawnati: %s | O=%d", _o2_spawned, _compute_O())

def cb_episode_end(msg):
    """
    Fine episodio (= fine singola traversata):
    log valori finali O/O1/O2, rimuovi tutti gli ostacoli, reset stato.
    """
    o_final  = _compute_O()
    o1_final = 1 if _o1_active else 0
    o2_final = 1 if len(_o2_spawned) > 0 else 0

    rospy.loginfo("[ObstaclePolicy] ── Episodio %d END | O=%d O1=%d O2=%d (oggetti: %s) ──",
                  msg.data, o_final, o1_final, o2_final, _o2_spawned)

    # Valori finali del confounder — commentati finche' il data logger non e' pronto
    # rospy.set_param('/hrisim/episode_O',  o_final)
    # rospy.set_param('/hrisim/episode_O1', o1_final)
    # rospy.set_param('/hrisim/episode_O2', o2_final)

    _remove_all()
    _reset_episode()

# ─────────────────────────────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    rospy.init_node("obstacle_policy_node")

    P1 = rospy.get_param("~p1", P1)
    P2 = rospy.get_param("~p2", P2)
    rospy.loginfo("[ObstaclePolicy] Avviato | p1=%.2f | p2=%.2f", P1, P2)
    rospy.loginfo("[ObstaclePolicy] Zona CROSS: cx=%.1f cy=%.1f rx=%.1f ry=%.1f",
                  rospy.get_param("~cross_cx",  0.0),
                  rospy.get_param("~cross_cy", -0.9),
                  rospy.get_param("~cross_rx",  1.5),
                  rospy.get_param("~cross_ry",  1.5))

    spawn_pub  = rospy.Publisher("/hrisim/obstacles/spawn",  String, queue_size=5)
    remove_pub = rospy.Publisher("/hrisim/obstacles/remove", String, queue_size=5)
    rospy.sleep(0.5)

    rospy.Subscriber("/hrisim/episode_start",              Int32,       cb_episode_start)
    rospy.Subscriber("/hrisim/episode_end",                Int32,       cb_episode_end)
    rospy.Subscriber("/hrisim/robot_action",               Int32,       cb_robot_action)
    rospy.Subscriber("/hrisim/robot_transit",              Bool,        cb_robot_transit)
    rospy.Subscriber("/pedsim_simulator/simulated_agents", AgentStates, cb_agents)

    rospy.spin()
