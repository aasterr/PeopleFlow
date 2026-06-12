#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import signal
import subprocess
import threading
import time

import rospy
from std_msgs.msg import Int32

BAG_DIR = "/home/hrisim/shared/hrisim_bags"

# Timeout (s) per la chiusura del bag. Il flush su volume condiviso e' lento:
# meglio aspettare che corrompere il bag.
SIGINT_TIMEOUT   = 20.0
SIGTERM_TIMEOUT  = 10.0
FINALIZE_TIMEOUT = 10.0   # attesa rinomina .bag.active -> .bag dopo l'uscita

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
    "/hrisim/obs/Pi",
    "/hrisim/obs/Pe",
    # Ausiliari
    "/hrisim/obstacles/positions",
    "/hrisim/robot_closest_wp",
    "/hrisim/robot_tasks_info",
]

_lock = threading.Lock()
_bag_process = None
_current_episode = -1
_current_path = None      # path SENZA estensione (come passato a -O)


def _bag_path(episode_id):
    os.makedirs(BAG_DIR, exist_ok=True)
    return os.path.join(BAG_DIR, "episode_{:04d}".format(episode_id))


def _wait_finalized(path_noext, timeout):
    active = path_noext + ".bag.active"
    deadline = time.time() + timeout
    while time.time() < deadline:
        if not os.path.exists(active):
            return True
        time.sleep(0.2)
    return False


def _stop_locked():
    global _bag_process, _current_path
    if _bag_process is None:
        return

    proc = _bag_process
    path = _current_path

    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=SIGINT_TIMEOUT)
    except subprocess.TimeoutExpired:
        rospy.logwarn("[record] SIGINT non basta dopo %.0fs, provo SIGTERM.",
                      SIGINT_TIMEOUT)
        proc.terminate()
        try:
            proc.wait(timeout=SIGTERM_TIMEOUT)
        except subprocess.TimeoutExpired:
            rospy.logerr("[record] SIGTERM fallito, SIGKILL. "
                         "Il bag restera' .active: serve 'rosbag reindex'.")
            proc.kill()
            proc.wait()

    if path is not None:
        if _wait_finalized(path, FINALIZE_TIMEOUT):
            rospy.loginfo("[record] Bag chiuso correttamente (episodio %d): %s.bag",
                          _current_episode, path)
        else:
            rospy.logerr("[record] %s.bag.active NON finalizzato! "
                         "Recuperalo con: rosbag reindex %s.bag.active",
                         path, path)

    _bag_process = None
    _current_path = None


def _start_locked(episode_id):
    global _bag_process, _current_episode, _current_path

    if _bag_process is not None:
        rospy.logwarn("[record] Recorder ancora attivo all'arrivo di "
                      "episode_start %d: lo chiudo prima.", episode_id)
        _stop_locked()

    path = _bag_path(episode_id)
    cmd = ["rosbag", "record", "--lz4", "-O", path] + TOPICS
    rospy.loginfo("[record] Avvio bag: %s.bag", path)
    _bag_process = subprocess.Popen(cmd)
    _current_episode = episode_id
    _current_path = path


def cb_episode_start(msg):
    rospy.loginfo("[record] episode_start %d", msg.data)
    with _lock:
        _start_locked(msg.data)


def cb_episode_end(msg):
    rospy.loginfo("[record] episode_end %d", msg.data)
    with _lock:
        _stop_locked()


def _on_shutdown():
    with _lock:
        _stop_locked()


if __name__ == "__main__":
    rospy.init_node("hrisim_recording")
    rospy.loginfo("[record] Nodo pronto, in attesa degli episodi...")
    rospy.Subscriber("/hrisim/episode_start", Int32, cb_episode_start, queue_size=1)
    rospy.Subscriber("/hrisim/episode_end",   Int32, cb_episode_end,   queue_size=1)
    rospy.on_shutdown(_on_shutdown)
    rospy.spin()
