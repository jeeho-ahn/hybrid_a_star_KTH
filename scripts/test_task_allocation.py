#!/usr/bin/env python
import os
import sys

# Get the current directory
current_directory = os.path.dirname(os.path.abspath(__file__))
# Add the current directory to the Python path
sys.path.insert(0, current_directory)

import rospy
from nav_msgs.msg import Path as navPath
import numpy as np
#from multipath_hungarian.srv import allocationReqSrv
from agent import agent
from task import pickup_task
from marker_publisher import MarkerPublisher
from transformation_tools import fit_angle_rad
from communications import add_cube_srv_req, req_plan_pose2D, req_allocation_srv, publish_paths, generate_cost_matrix

def initialize_path_publishers(nRobot:int):

    path_publishers=[]

    #todo: handle empty path
    for p in range(nRobot):
        path_publishers.append(rospy.Publisher('/planned_path/'+str(p), navPath, queue_size=10))

    return path_publishers


if __name__ == '__main__':
    rospy.init_node('test_task_allocation')  # Replace 'your_node_name' with the desired name for your node

    #generate cost matrix
    nRobot = 4
    nTask = 4
    nWays = 4
    cost_mat = np.zeros((nRobot, nTask*nWays), dtype=float)

    ang_res = 72
    block_size = 0.2 #width or height in m

    agents_list = [agent(4,0,fit_angle_rad(1.5708,ang_res)), agent(2,6,fit_angle_rad(-1.5708,ang_res)), 
                   agent(0,0,fit_angle_rad(1.5708,ang_res)), agent(0,6,fit_angle_rad(-1.5708,ang_res))]
    
    #pick-up tasks
    ptask_list = [pickup_task(1.5,3,ang_res), pickup_task(3.5,3,ang_res), pickup_task(2.5,3.5,ang_res), pickup_task(0.5,3.5,ang_res)]

    #visualize blocks
    blocks_pub = MarkerPublisher()
    add_cube_srv_req(ptask_list, block_size)

    #array to save paths in (any faster way than dicts of dicts?)
    path_db = {}
    path_publishers = initialize_path_publishers(nRobot)

    #fill cost matrix by calling path plan request for all options
    cost_mat = generate_cost_matrix(nRobot,nTask,nWays,path_db,ptask_list,agents_list)
    print(cost_mat)    

    #request best allocation to multiple-choice hungarian algorithm node
    rob, taskw = req_allocation_srv(nRobot,nTask,nWays,cost_mat)

    #visualize blocks
    blocks_pub.publish_marker(ptask_list)
    publish_paths(rob,taskw,path_publishers,path_db)


    while not rospy.is_shutdown(): 
        rospy.spin()

    