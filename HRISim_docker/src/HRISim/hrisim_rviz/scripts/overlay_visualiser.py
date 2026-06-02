#!/usr/bin/env python
import rospy
from jsk_rviz_plugins.msg import OverlayText
from std_msgs.msg import ColorRGBA, Int32
from robot_msgs.msg import TasksInfo

def cb_tasks(msg: TasksInfo):
    global TASKS
    TASKS = (int(msg.num_tasks), int(msg.num_success), int(msg.num_failure))

def cb_episode_start(msg: Int32):
    global EPISODE
    EPISODE = msg.data

def cb_action(msg: Int32):
    global ACTION
    ACTION = msg.data

def create_overlay_text():
    text = OverlayText()
    text.width = 400
    text.height = 150
    text.left = 10
    text.top = 10
    text.text_size = 13
    text.line_width = 2
    text.font = "DejaVu Sans Mono"
    text.fg_color = ColorRGBA(1.0, 1.0, 1.0, 1.0)

    ep_str  = f"Episode: {EPISODE}" if EPISODE is not None else "Episode: -"
    act_str = f"Action:  {ACTION}"  if ACTION  is not None else "Action:  -"

    if TASKS is not None:
        total   = TASKS[0]
        success = TASKS[1]
        failure = TASKS[2]
        rate    = (success / total * 100) if total > 0 else 0.0
        task_str = (
            f"Tasks:   {total} total\n"
            f"Success: {success} ({rate:.1f}%)\n"
            f"Failure: {failure}"
        )
    else:
        task_str = "Tasks: -"

    text.text = "\n".join([ep_str, act_str, "─" * 30, task_str])
    return text

if __name__ == '__main__':
    rospy.init_node('overlay_visualiser')
    rate = rospy.Rate(1)

    EPISODE = None
    ACTION  = None
    TASKS   = None

    rospy.Subscriber('/hrisim/robot_tasks_info', TasksInfo, cb_tasks)
    rospy.Subscriber('/hrisim/episode_start',    Int32,     cb_episode_start)
    rospy.Subscriber('/hrisim/robot_action',     Int32,     cb_action)

    text_pub = rospy.Publisher('/hrisim/robot/info/main', OverlayText, queue_size=10)

    while not rospy.is_shutdown():
        text_pub.publish(create_overlay_text())
        rate.sleep()
