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
        if agents_param is not None:
            a = Agent.from_dict(agents_param, SCHEDULE, G, ALLOW_TASK, MAX_TASKTIME)
        else:
            a = Agent(agent_id, SCHEDULE, G, ALLOW_TASK, MAX_TASKTIME)
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
                next_destination = AGENTSPLAN[int(agent.id)]['tasks'][self.timeOfDay]['destinations'].pop(0)
                agent.setTask(next_destination, AGENTSPLAN[int(agent.id)]['tasks'][self.timeOfDay]['durations'].pop(0))
            
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
        
    agentsplan_path = rospy.get_param("~agent_task_list", False)
    with open(agentsplan_path, 'rb') as f:
        AGENTSPLAN = pickle.load(f)
                
    pedsimBridge = PedsimBridge()
    rospy.logwarn("Pedsim Bridge started!")
                
    rospy.spin()