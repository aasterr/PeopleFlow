#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
record.py
=========
Registra un bag ROS per ogni episodio (episode_start -> episode_end).

Nodi DAG coperti:
  Pi, Pe  <- /pedsim_simulator/simulated_agents + /robot_pose
  A       <- /hrisim/robot_action
  O       <- /hrisim/obs/O
  S       <- /hrisim/obs/S
  T       <- /hrisim/obs/T
"""

import os
import signal
import subprocess
import rospy
from std_msgs.msg import Int32

BAG_DIR = "/home/hrisim/shared/hrisim_bags"

TOPICS = [
    # Navigazione
    "/map",
    "/tf",
    "/tf_static",
    "/robot_pose",
    "/mobile_base_controller/odom",
    "/move_base/goal",
    # Episodio
    "/hrisim/episode_start",
    "/hrisim/episode_end",
    # Nodi DAG
    "/hrisim/robot_action",
    "/hrisim/obs/O",
    "/hrisim/obs/S",
    "/hrisim/obs/T",
    # Persone
    "/pedsim_simulator/simulated_agents",
    # Ausiliari
    "/hrisim/robot_closest_wp",
    "/hrisim/robot_transit",
    "/hrisim/people_cleared",
    "/hrisim/robot_tasks_info",
]

_bag_process = None
_current_episode = -1


def _bag_path(episode_id):
    os.makedirs(BAG_DIR, exist_ok=True)
    return os.path.join(BAG_DIR, "episode_{:04d}".format(episode_id))


def _start_recording(episode_id):
    global _bag_process, _current_episode
    if _bag_process is not None:
        rospy.logwarn("[record] Recorder già attivo, fermo il precedente.")
        _stop_recording()
    path = _bag_path(episode_id)
    cmd = ["rosbag", "record", "-O", path] + TOPICS
    rospy.loginfo("[record] Avvio bag: %s.bag", path)
    _bag_process = subprocess.Popen(cmd)
    _current_episode = episode_id


def _stop_recording():
    global _bag_process
    if _bag_process is None:
        return
    _bag_process.send_signal(signal.SIGINT)
    try:
        _bag_process.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        rospy.logwarn("[record] Timeout — SIGKILL.")
        _bag_process.kill()
    rospy.loginfo("[record] Bag chiuso (episodio %d).", _current_episode)
    _bag_process = None


def cb_episode_start(msg):
    rospy.loginfo("[record] episode_start %d", msg.data)
    _start_recording(msg.data)


def cb_episode_end(msg):
    rospy.loginfo("[record] episode_end %d", msg.data)
    _stop_recording()


if __name__ == "__main__":
    rospy.init_node("hrisim_recording")
    rospy.loginfo("[record] Nodo pronto, in attesa degli episodi...")
    rospy.Subscriber("/hrisim/episode_start", Int32, cb_episode_start, queue_size=1)
    rospy.Subscriber("/hrisim/episode_end",   Int32, cb_episode_end,   queue_size=1)
    rospy.on_shutdown(_stop_recording)
    rospy.spin()
