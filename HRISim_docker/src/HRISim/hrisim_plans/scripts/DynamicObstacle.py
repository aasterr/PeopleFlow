#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
DynamicObstacle.py
==================
Spawna e rimuove ostacoli in Gazebo tramite topic ROS.

Spawn:  /hrisim/obstacles/spawn  (String)  →  "ID❌y"
Remove: /hrisim/obstacles/remove (String)  →  "ID" | "ALL"

Esempi:
  spawn  "O1:0.0:1.2"
  spawn  "O2_4:-0.3:0.8"
  remove "O1"
  remove "ALL"
"""

import rospy
import os
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse


class DynamicObstacleSpawner:
    def __init__(self):
        rospy.init_node("dynamic_obstacle_node")

        # Dizionario ID -> model_name in Gazebo
        self._active = {}

        # Modello SDF
        self.model_path = os.path.expanduser(
            "/home/hrisim/.gazebo/models/Suitcase1H/model.sdf")
        with open(self.model_path, "r") as f:
            self.model_xml = f.read()

        # Gazebo services
        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        rospy.wait_for_service("/gazebo/delete_model")
        self.spawn_srv  = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        self.delete_srv = rospy.ServiceProxy("/gazebo/delete_model",    DeleteModel)

        # Topic
        rospy.Subscriber("/hrisim/obstacles/spawn",  String, self.cb_spawn)
        rospy.Subscriber("/hrisim/obstacles/remove", String, self.cb_remove)

        rospy.loginfo("[DynamicObstacle] Pronto.")

    # ── SPAWN ────────────────────────────────────────────────────────

    def cb_spawn(self, msg):
        """
        Payload atteso: "ID❌y"
        Es: "O1:0.0:1.2"  |  "O2_4:-0.3:0.8"
        """
        try:
            parts = msg.data.split(":")
            obs_id = parts[0]
            x = float(parts[1])
            y = float(parts[2])
        except (ValueError, IndexError):
            rospy.logerr("[DynamicObstacle] Payload spawn malformato: '%s'", msg.data)
            return

        if obs_id in self._active:
            rospy.logwarn("[DynamicObstacle] '%s' già presente, skip.", obs_id)
            return

        model_name = "obstacle_{}".format(obs_id)
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = 0.0

        try:
            self.spawn_srv(model_name, self.model_xml, "", pose, "world")
            self._active[obs_id] = model_name
            rospy.loginfo("[DynamicObstacle] Spawnato '%s' in (%.2f, %.2f)", obs_id, x, y)
        except rospy.ServiceException as e:
            rospy.logerr("[DynamicObstacle] Spawn '%s' fallito: %s", obs_id, e)

    # ── REMOVE ───────────────────────────────────────────────────────

    def cb_remove(self, msg):
        """
        Payload: "ID" per rimuovere uno specifico | "ALL" per rimuovere tutti.
        """
        obs_id = msg.data.strip()

        if obs_id == "ALL":
            for oid in list(self._active.keys()):
                self._remove_one(oid)
        elif obs_id in self._active:
            self._remove_one(obs_id)
        else:
            rospy.logwarn("[DynamicObstacle] '%s' non trovato tra gli attivi.", obs_id)

    def _remove_one(self, obs_id):
        model_name = self._active.get(obs_id)
        if model_name is None:
            return
        try:
            self.delete_srv(model_name)
            del self._active[obs_id]
            rospy.loginfo("[DynamicObstacle] Rimosso '%s'", obs_id)
        except rospy.ServiceException as e:
            rospy.logerr("[DynamicObstacle] Delete '%s' fallito: %s", obs_id, e)


if __name__ == "__main__":
    node = DynamicObstacleSpawner()
    rospy.spin()
