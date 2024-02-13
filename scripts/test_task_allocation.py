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
from task import pickup_task, delivery_task
from marker_publisher import MarkerPublisher, LinePublisher
from transformation_tools import fit_angle_rad
from communications import add_cube_srv_req, req_plan_pose2D, req_allocation_srv, publish_paths, generate_cost_matrix

from geometry_msgs.msg import Point
import math


test_data = 'ex6'

def initialize_path_publishers(nRobot:int):

    path_publishers=[]

    #todo: handle empty path
    for p in range(nRobot):
        path_publishers.append(rospy.Publisher('/planned_path/'+str(p), navPath, queue_size=10))

    return path_publishers


def initialize_line_publishers(nRobot:int):

    line_publishers = []
    for p in range(nRobot):
        line_publishers.append(LinePublisher('/planned_line/'+str(p)))

    return line_publishers


def init_agents():

    if(test_data == 'ex3'):
        #ex3
        x = [4,2,0,0]
        y = [0,6,0,6]
        th = [1.5708,-1.5708,1.5708,-1.5708]

    elif(test_data == 'ex6'):
        #ex6
        x = [4, 0.5, 1, 3]
        y = [0, 6,  0,  6]
        th = [1.5708,-1.5708,1.5708,-1.5708]

    return [agent(x[0],y[0],fit_angle_rad(th[0],ang_res)), agent(x[1],y[1],fit_angle_rad(th[1],ang_res)), 
                   agent(x[2],y[2],fit_angle_rad(th[2],ang_res)), agent(x[3],y[3],fit_angle_rad(th[3],ang_res))]

def init_pickup_tasks(angle_res = 72):

    if(test_data == 'ex3'):
        #ex3
        x = [1.5,3.5,2.5,0.5]
        y = [3,3,3.5,3.5]

    elif(test_data == 'ex6'):
        #ex6
        x = [2, 1  ,   1.5, 2]
        y = [3, 3.5,   3.5,   3.5]

    return [pickup_task(x[0],y[0],angle_res), pickup_task(x[1],y[1],angle_res), pickup_task(x[2],y[2],angle_res), pickup_task(x[3],y[3],angle_res)]

def init_delivery_tasks():

    if(test_data == 'ex3'):
        #ex3
        x = [4,0,3,0]
        y = [6,6,0,0]

    elif(test_data == 'ex6'):
        x = [0,     2,4,0]
        y = [0.5,   0,6,6]

    return [delivery_task(x[0],y[0]),delivery_task(x[1],y[1]),delivery_task(x[2],y[2]),delivery_task(x[3],y[3])]

if __name__ == '__main__':
    rospy.init_node('test_task_allocation')  # Replace 'your_node_name' with the desired name for your node

    #generate cost matrix
    nRobot = 4
    nTask = 4
    nWays = 4
    cost_mat = np.zeros((nRobot, nTask*nWays), dtype=float)

    #angle search resolution in hybrid A*
    ang_res = 72
    #width or height in m
    block_size = 0.2

    #init agents
    agents_list = init_agents()
    
    #pick-up tasks
    ptask_list = init_pickup_tasks(ang_res)

    #visualize blocks
    blocks_pub = MarkerPublisher()
    #wait for publisher attachment
    rospy.sleep(1)
    #add cubes as obstacles
    add_cube_srv_req(ptask_list, block_size)
    #visualize blocks
    blocks_pub.publish_marker(ptask_list)

    #array to save paths in (any faster way than dicts of dicts?)
    path_db = {}
    path_publishers = initialize_path_publishers(nRobot)

    #fill cost matrix by calling path plan request for all options
    cost_mat_pickup = generate_cost_matrix(nRobot,nTask,nWays,path_db,ptask_list,agents_list,'pickup')
    
    #manual cost matrix for testing
    #cost_mat_pickup = np.array([[2.06592959, 2.31725315, 0.75628018, 0.56570147, 1.68693795, 2.73927156, 1.06795312, 0.2674108,  1.84689335, 2.57348792, 0.94897204, 0.38991786, 1.99365025, 2.43648131, 0.84726601, 0.50126426],
    #                    [1.35742768, 3.14159265, 1.35742768, 10000,        1.13026599, 2.80413658, 1.67046498, 0.23645614, 1.25333759, 2.96792146, 1.52796539, 0.11990399, 1.387174,   3.14159265, 1.387174,   100000 ],
    #                    [0.84726601, 0.50126426, 1.99365025, 2.43648131, 0.60054113, 0.76101275, 2.32527651, 2.12338068, 0.67085747, 0.67085747, 2.20786666, 2.20786666, 0.75628018, 0.56570147, 2.06592959, 2.31725315],
    #                    [1.99365025, 0.50126426, 0.84726601, 2.43648131, 1.70989227, 0.30739747, 0.98742332, 2.64604098, 1.89853483, 0.44441921, 0.86052532, 2.46029443, 2.06592959, 0.56570147, 0.75628018, 2.31725315]])
    
    print(cost_mat_pickup)

    #initialize delivery tasks
    dtask_list = init_delivery_tasks()

    #delivery cost evaluation
    dAlloc_mode = 'line'

    cost_mat_delivery = np.zeros((0,0),dtype=float)
    empyt_dict = {}
    if dAlloc_mode == 'line':
        cost_mat_delivery = generate_cost_matrix(nRobot,nTask,nWays,empyt_dict,ptask_list,dtask_list,'delivery')
        print(cost_mat_delivery)


    #best allocation for delivery
    deliv, taskwd = req_allocation_srv(nRobot,nTask,nWays,cost_mat_delivery)

    print(deliv)
    print(taskwd)

    #mask pickup allocation
    for row in deliv:
        for col in range(nTask*nWays):
            if col not in taskwd:
                cost_mat_pickup[row,col] = 100000000

    print(cost_mat_pickup)


    #request best allocation to multiple-choice hungarian algorithm node
    rob, taskw = req_allocation_srv(nRobot,nTask,nWays,cost_mat_pickup)
    publish_paths(rob,taskw,path_publishers,path_db)


    #publish delivery allocation line
    line_pub_list = initialize_line_publishers(nRobot)
    rospy.sleep(1)
    for li in range(len(line_pub_list)):
        #p1: delivery point
        p1 = Point()
        p1 = dtask_list[li].get_center().to_ros_pose().position
        #p2: pickup point
        ##pickup number
        pn = math.floor(taskwd[li] / nWays)
        p2 = ptask_list[pn].get_center().to_ros_pose().position

        
        #pub
        line_pub_list[li].draw_line(p1,p2)


    while not rospy.is_shutdown(): 
        rospy.spin()

    