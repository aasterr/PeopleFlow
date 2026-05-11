#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
obstacle_policy.py
==================
Nodo ROS indipendente che gestisce la policy probabilistica del confounder O.

Logica:
  - All'inizio di ogni episodio (segnale /hrisim/episode_start) decide con
    probabilità P_OBSTACLE se spawnare l'ostacolo fisico nel corridoio.
  - L'ostacolo contribuisce a Pi (congestione percepita dal robot) e
    rimane fisicamente presente causando T=0 se il robot non riesce a passare.
  - A fine episodio (segnale /hrisim/episode_end) rimuove l'ostacolo e
    resetta lo stato.

Dipendenze:
  - DynamicObstacle.py (espone /hrisim/obstacles/spawn e /hrisim/obstacles/remove)
  - TIAGo_plan.py (pubblica /hrisim/episode_start e /hrisim/episode_end)
"""

import rospy
from std_msgs.msg import Bool, Int32
from std_srvs.srv import Empty as EmptySrv
import random

# ── Parametri ────────────────────────────────────────────────────────
P_OBSTACLE = 0.5   # probabilità che O=1 in un episodio

# ── Stato globale ────────────────────────────────────────────────────
_obstacle_active = False

# ─────────────────────────────────────────────────────────────────────
# HELPERS
# ─────────────────────────────────────────────────────────────────────

def _call_service(srv_name):
    """Chiama un servizio Empty, logga errori senza crashare."""
    try:
        rospy.wait_for_service(srv_name, timeout=3.0)
        proxy = rospy.ServiceProxy(srv_name, EmptySrv)
        proxy()
        return True
    except Exception as e:
        rospy.logwarn("[ObstaclePolicy] Servizio %s non disponibile: %s", srv_name, e)
        return False

def _set_O(value):
    """Aggiorna il rosparam e pubblica O sul topic."""
    rospy.set_param('/hrisim/robot_obs', value)
    obs_pub.publish(Bool(value))
    rospy.loginfo("[ObstaclePolicy] O=%d", int(value))

# ─────────────────────────────────────────────────────────────────────
# CALLBACKS
# ─────────────────────────────────────────────────────────────────────

def cb_episode_start(msg):
    """
    Ricevuto all'inizio di ogni episodio da TIAGo_plan.
    Decide se spawnare O con probabilità P_OBSTACLE.
    """
    global _obstacle_active
    episode_num = msg.data

    O = 1 if random.random() < P_OBSTACLE else 0
    rospy.loginfo("[ObstaclePolicy] ── Episodio %d: O=%d (p=%.2f) ──",
                  episode_num, O, P_OBSTACLE)

    if O == 1:
        success = _call_service('/hrisim/obstacles/spawn')
        if success:
            _obstacle_active = True
            _set_O(True)
        else:
            rospy.logwarn("[ObstaclePolicy] Spawn fallito, O forzato a 0")
            _set_O(False)
    else:
        _obstacle_active = False
        _set_O(False)


def cb_episode_end(msg):
    """
    Ricevuto a fine episodio da TIAGo_plan.
    Rimuove l'ostacolo se presente e resetta lo stato.
    """
    global _obstacle_active

    if _obstacle_active:
        rospy.loginfo("[ObstaclePolicy] Fine episodio: rimuovo ostacolo")
        _call_service('/hrisim/obstacles/remove')
        _obstacle_active = False
        _set_O(False)
    else:
        rospy.loginfo("[ObstaclePolicy] Fine episodio: nessun ostacolo da rimuovere")

# ─────────────────────────────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    rospy.init_node("obstacle_policy_node")

    P_OBSTACLE = rospy.get_param("~p_obstacle", P_OBSTACLE)
    rospy.loginfo("[ObstaclePolicy] Avviato | P_OBSTACLE=%.2f", P_OBSTACLE)

    obs_pub = rospy.Publisher("/hrisim/robot_obs", Bool, queue_size=1)

    rospy.Subscriber("/hrisim/episode_start", Int32, cb_episode_start)
    rospy.Subscriber("/hrisim/episode_end",   Int32, cb_episode_end)

    rospy.spin()
