#!/usr/bin/env python

import rospy
from hybrid_astar.srv import planReqSrv
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from std_msgs.msg import Int32
import math

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

def euler_deg_to_quaternion2d(angle_in_deg):
    angle_rad = math.radians(angle_in_deg)

    w,x,y,z = euler_to_quaternion(angle_rad,0,0)

    out_pose = Pose()
    out_pose.orientation.w = w
    out_pose.orientation.x = x
    out_pose.orientation.y = y
    out_pose.orientation.z = z

    return out_pose.orientation


def sq(d):
    return d*d;

def normalize_q(q_in):
    norm = math.sqrt(sq(q_in.w) + sq(q_in.x) + sq(q_in.y) + sq(q_in.z))

    q_in.x = q_in.x/norm
    q_in.y = q_in.y/norm
    q_in.z = q_in.z/norm
    q_in.w = q_in.w/norm

    return q_in

def service_client(start_pose, goal_pose):
    rospy.wait_for_service('/plan_req_test')  # Replace 'your_service_name' with the actual name of your service
    try:
        plan_service = rospy.ServiceProxy('/plan_req_test', planReqSrv)  # Replace 'your_service_name' with the actual name of your service
        response = plan_service(start_pose, goal_pose)
        return response.result
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")
        return -1

if __name__ == '__main__':
    rospy.init_node('your_node_name')  # Replace 'your_node_name' with the desired name for your node

    # Example start and goal poses
    start_pose = PoseWithCovarianceStamped()
    goal_pose = PoseStamped()

    # Example start and goal poses with specific values
    start_pose = PoseWithCovarianceStamped()
    start_pose.pose.pose.position.x = 21.1918
    start_pose.pose.pose.position.y = 20.4737
    #start_pose.pose.pose.orientation.z = 0.927
    #start_pose.pose.pose.orientation = normalize_q(start_pose.pose.pose.orientation)
    start_pose.pose.pose.orientation = euler_deg_to_quaternion2d(107.928)
    # Add other pose information if needed

    goal_pose = PoseStamped()
    goal_pose.pose.position.x = 27.359
    goal_pose.pose.position.y = 72.1764
    #goal_pose.pose.orientation.z = 0.623
    #goal_pose.pose.orientation = normalize_q(goal_pose.pose.orientation)
    goal_pose.pose.orientation = euler_deg_to_quaternion2d(351.617)

    result = service_client(start_pose, goal_pose)

    if result != -1:
        rospy.loginfo(f"Service call successful. Result: {result}")
    else:
        rospy.logwarn("Service call failed.")

    rospy.spin()
