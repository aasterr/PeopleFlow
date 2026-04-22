#!/usr/bin/env python

import rospy
import networkx as nx 
import random
import pickle
from pedsim_srvs.srv import GetNextDestination, GetNextDestinationResponse
from peopleflow_msgs.msg import Time as pT
from peopleflow_util.Agent import Agent 
import hrisim_util.ros_utils as ros_utils
import hrisim_util.constants as constants
import traceback
import time
from std_srvs.srv import Empty
from robot_srvs.srv import VisualisePath

def seconds_to_hhmmss(seconds):
    return time.strftime("%H:%M:%S", time.gmtime(seconds))

class PedsimBridge():
    def __init__(self):
        self.timeDefined = False
        rospy.Subscriber("/peopleflow/time", pT, self.cb_time)
        
        while not self.timeDefined: rospy.sleep(0.1)
        
        rospy.Service('get_next_destination', GetNextDestination, self.handle_get_next_destination)
        rospy.loginfo('ROS service /get_next_destination advertised')
        
    def cb_time(self, t: pT):
        self.timeOfDay = t.time_of_the_day.data
        self.elapsedTimeString = t.hhmmss.data
        self.elapsedTime = t.elapsed
        self.timeDefined = True
        
    def load_agents(self, req):
        """Load agents from the ROS parameter server."""
        agent_id = str(req.agent_id)
        agents_param = rospy.get_param(f'/peopleflow/agents/{agent_id}', None)
    
        if agent_id in AGENTS_CROSS:
            schedule = SCHEDULE_CROSS
        elif agent_id in AGENTS_POSTER:
            schedule = SCHEDULE_POSTER
        else:
            schedule = SCHEDULE

        if agents_param is not None:
            a = Agent.from_dict(agents_param, schedule, G, ALLOW_TASK, MAX_TASKTIME)
        else:
            a = Agent(agent_id, schedule, G, ALLOW_TASK, MAX_TASKTIME)

        a.x = req.origin.x
        a.y = req.origin.y
        a.isStuck = req.is_stuck
        return a
    
    def save_agents(self, agent):
        """Save agents to the ROS parameter server."""
        rospy.set_param(f'/peopleflow/agents/{agent.id}', agent.to_dict())
             
    def handle_get_next_destination(self, req):
        try:
            # Load agent from rosparam
            agent = self.load_agents(req)
                        
            # New goal logic                
            if agent.isStuck or agent.isFree:
                next_destination = agent.selectDestination(self.timeOfDay, req.destinations)
                agent.setTask(next_destination, agent.getTaskDuration())
            
            elif not agent.isFree: 
                pass
                                                                            
            else:
                rospy.logerr("THERE IS A CASE I DID NOT COVER:")
                rospy.logerr(f"TOD {self.timeOfDay}")
                rospy.logerr(f"elapsedTime {self.elapsedTime}")
                rospy.logerr(f"Agent {agent.id}")
                rospy.logerr(f"isFree {agent.isFree}")
                rospy.logerr(f"isStuck {agent.isStuck}")
                rospy.logerr(f"closestWP {agent.closestWP}")
                
            
            # Response
            wpname, wp = agent.nextWP         
            agent.nextDestRadius = WPS[wpname]["r"] if wpname in WPS else 1.0
            response = GetNextDestinationResponse(destination_id=wpname, 
                                                  destination=wp, 
                                                  destination_radius=WPS[wpname]["r"] if wpname in WPS else 1.0,
                                                  task_duration=agent.taskDuration[wpname])
            # Save agents to rosparam
            self.save_agents(agent)
            return response
        
        except Exception as e:
            rospy.logerr(f"Time: {self.timeOfDay} - {self.elapsedTimeString}")
            rospy.logerr(f"Agent {agent.id} generated error: {str(e)}")
            rospy.logerr(f"Traceback: {traceback.format_exc()}")


  
if __name__ == '__main__':
    rospy.init_node('peopleflow_pedsim_bridge')
    rate = rospy.Rate(10)  # 10 Hz
    
    SCHEDULE = ros_utils.wait_for_param("/peopleflow/schedule")

    SCHEDULE_CROSS = {time_name: {
        'dests': {k: v for k, v in time_data['dests'].items() if k in ['WP_CROSS']},
        'duration': time_data['duration']
    } for time_name, time_data in SCHEDULE.items()}

    SCHEDULE_POSTER = {time_name: {
        'dests': {k: v for k, v in time_data['dests'].items() if k in ['WP_POSTER_L', 'WP_POSTER_R']},
        'duration': time_data['duration']
    } for time_name, time_data in SCHEDULE.items()}

    AGENTS_CROSS = ['4', '5', '6', '7']
    AGENTS_POSTER = ['0', '1', '2', '3']
    WPS = ros_utils.wait_for_param("/peopleflow/wps")
    ALLOW_TASK = rospy.get_param("~allow_task", False)
    MAX_TASKTIME = int(rospy.get_param("~max_tasktime"))
    g_path = str(rospy.get_param("~g_path"))
    with open(g_path, 'rb') as f:
        G = pickle.load(f)
        ros_utils.load_graph_to_rosparam(G, "/peopleflow/G")
        
        # Create a handle for the Trigger service
        ros_utils.wait_for_service('/graph/path/show')
        graph_path_show = rospy.ServiceProxy('/graph/path/show', VisualisePath)        # Call the service
        graph_path_show("")
                
    pedsimBridge = PedsimBridge()
    rospy.logwarn("Pedsim Bridge started!")
                
    rospy.spin()
