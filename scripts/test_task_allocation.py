#!/usr/bin/env python

import rospy
from hybrid_astar.srv import planReqSrv
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from std_msgs.msg import Int32
import math
import numpy as np

from multipath_hungarian.srv import allocationReqSrv
from std_msgs.msg import Float32MultiArray

def euler_to_quaternion(yaw, pitch, roll):
    cy = math.cos(yaw / 2.0)
    sy = math.sin(yaw / 2.0)
    cp = math.cos(pitch / 2.0)
    sp = math.sin(pitch / 2.0)
    cr = math.cos(roll / 2.0)
    sr = math.sin(roll / 2.0)

    w = cy * cp * cr + sy * sp * sr
    x = cy * cp * sr - sy * sp * cr
    y = sy * cp * sr + cy * sp * cr
    z = sy * cp * cr - cy * sp * sr

    return w, x, y, z


def euler_rad_to_quaternion2d(angle_in_rad):
    w,x,y,z = euler_to_quaternion(angle_in_rad,0,0)

    out_pose = Pose()
    out_pose.orientation.w = w
    out_pose.orientation.x = x
    out_pose.orientation.y = y
    out_pose.orientation.z = z

    return out_pose.orientation


def euler_deg_to_quaternion2d(angle_in_deg):
    angle_rad = math.radians(angle_in_deg)

    return euler_rad_to_quaternion2d(angle_rad)


def sq(d):
    return d*d;

def normalize_q(q_in):
    norm = math.sqrt(sq(q_in.w) + sq(q_in.x) + sq(q_in.y) + sq(q_in.z))

    q_in.x = q_in.x/norm
    q_in.y = q_in.y/norm
    q_in.z = q_in.z/norm
    q_in.w = q_in.w/norm

    return q_in

def service_client(start_pose_req, goal_pose_req):
    rospy.wait_for_service('/plan_req_test')  # Replace 'your_service_name' with the actual name of your service
    try:
        plan_service = rospy.ServiceProxy('/plan_req_test', planReqSrv)  # Replace 'your_service_name' with the actual name of your service
        response = plan_service(start_pose_req, goal_pose_req)
        return response
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")
        return -1
    
class pose2D:
    def __init__(self,x_in,y_in,th_in) -> None:
        self.x = x_in
        self.y = y_in
        self.th = th_in

    def to_ros_pose(self):
        out_pose = Pose()
        out_pose.position.x = self.x
        out_pose.position.y = self.y
        out_pose.position.z = 0
        out_pose.orientation = euler_rad_to_quaternion2d(self.th)
        return out_pose

class agent:
    def __init__(self, start_x, start_y, start_th) -> None:
        self.start_pose = pose2D(start_x,start_y,start_th)

    def get_start_pose(self) -> pose2D:
        return self.start_pose

class task:
    def __init__(self, pos_x_in, pos_y_in, arrival_gap:float=0) -> None:
        self.x = pos_x_in
        self.y = pos_y_in

        self.poses = [0,1.5708,3.1415,-1.5708]

    def get_goal_poses(self) -> list:
        #todo: use loop
        return [pose2D(self.x,self.y,self.poses[0]),
                pose2D(self.x,self.y,self.poses[1]),
                pose2D(self.x,self.y,self.poses[2]),
                pose2D(self.x,self.y,self.poses[3])]
    


def req_plan_pose2D(start_req:pose2D,goal_req:pose2D):
    start_pose = PoseWithCovarianceStamped()
    goal_pose = PoseStamped()

    start_pose.pose.pose = start_req.to_ros_pose()
    goal_pose.pose = goal_req.to_ros_pose()

    srv_res = service_client(start_pose, goal_pose)
    return srv_res.trav_dist.data



if __name__ == '__main__':
    rospy.init_node('test_task_allocation')  # Replace 'your_node_name' with the desired name for your node

    # Example start and goal poses
    #start_pose = PoseWithCovarianceStamped()
    #goal_pose = PoseStamped()

    # Example start and goal poses with specific values
    #start_pose = PoseWithCovarianceStamped()
    #start_pose.pose.pose.position.x = 10.5
    #start_pose.pose.pose.position.y = 8.58
    #start_pose.pose.pose.orientation.z = 0.927
    #start_pose.pose.pose.orientation = normalize_q(start_pose.pose.pose.orientation)
    #start_pose.pose.pose.orientation = euler_deg_to_quaternion2d(123.476)
    # Add other pose information if needed

    #goal_pose = PoseStamped()
    #goal_pose.pose.position.x = 8.22
    #goal_pose.pose.position.y = 35.9
    #goal_pose.pose.orientation.z = 0.623
    #goal_pose.pose.orientation = normalize_q(goal_pose.pose.orientation)
    #goal_pose.pose.orientation = euler_deg_to_quaternion2d(173.047)


    agents_list = [agent(4,0,1.5708), agent(2,6,-1.5708), 
                   agent(0,0,1.5708), agent(0,6,-1.5708)]
    

    task_list = [task(1.5,3), task(3.5,3), task(2.5,3.5), task(0.5,3.5)]


    #generate cost matrix
    nRobot = 4
    nTask = 4
    nWays = 4
    cost_mat = np.zeros((nRobot, nTask*nWays), dtype=float)

    for robot in range(nRobot):
        for task in range(nTask):
            #init task
            pot_goals = task_list[task].get_goal_poses() #list of pose2D

            for way in range(nWays):
                cost_single = req_plan_pose2D(agents_list[robot].get_start_pose()
                                              ,pot_goals[way])
                print(cost_single)
                task_ind = task*nWays
                if(cost_single > 0):                    
                    cost_mat[robot,task_ind + way] = cost_single
                else:
                    #large cost
                    cost_mat[robot,task_ind + way] = 1000000

    #if result != -1:
    #    rospy.loginfo(f"Service call successful. Result: {result}")
    #else:
    #    rospy.logwarn("Service call failed.")
                    
    print(cost_mat)


    #request allocation
    rospy.wait_for_service('/mc_hungarian/allocReq')
    try:
        service_proxy = rospy.ServiceProxy('/mc_hungarian/allocReq', allocationReqSrv)
        req = allocationReqSrv()
        
        # Call the service with flattened array
        response = service_proxy(nRobot,nTask,nWays,cost_mat.astype(np.float32).flatten())
        alloc_mat = np.array(response.allocation_matrix).reshape((nRobot, nTask*nWays))
        print(alloc_mat)
    except rospy.ServiceException as e:
        print("Service call failed:", e)

    rob, taskw = np.where(alloc_mat)

    print(rob)
    print(taskw)

    rospy.spin()
