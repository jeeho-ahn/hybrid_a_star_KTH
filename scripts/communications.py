
import rospy
import numpy as np
from hybrid_astar.srv import addCubesSrv, planReqSrv
from transformation_tools import pose2D
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from multipath_hungarian.srv import allocationReqSrv


  #request to add blocks as obstacles
def add_cube_srv_req(ptask_list:list, block_size:float=0.2):
    rospy.wait_for_service("/add_cube_obstacles")
    try:
        service_proxy = rospy.ServiceProxy('/add_cube_obstacles', addCubesSrv)

        poses = []
        for n in range(len(ptask_list)):
            o = ptask_list[n].get_center().to_ros_pose()

            poses.append(o)
        #req.pose_in = poses
        response = service_proxy(block_size,poses)
        
    except rospy.ServiceException as e:
        print("Service call failed:", e)

def plan_service_client(start_pose_req, goal_pose_req):
    rospy.wait_for_service('/plan_req_test') 
    try:
        plan_service = rospy.ServiceProxy('/plan_req_test', planReqSrv)
        response = plan_service(start_pose_req, goal_pose_req)
        return response
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")
        return -1
    

def req_plan_pose2D(start_req:pose2D,goal_req:pose2D):
    start_pose = PoseWithCovarianceStamped()
    goal_pose = PoseStamped()

    start_pose.pose.pose = start_req.to_ros_pose()
    goal_pose.pose = goal_req.to_ros_pose()

    srv_res = plan_service_client(start_pose, goal_pose)
    return srv_res.trav_dist.data, srv_res.planned_path


def req_allocation_srv(nRobot:int,nTask:int,nWays:int,cost_mat):
    #request allocation
    rospy.wait_for_service('/mc_hungarian/allocReq')
    try:
        service_proxy = rospy.ServiceProxy('/mc_hungarian/allocReq', allocationReqSrv)
        #req = allocationReqSrv()
        
        # Call the service with flattened array
        response = service_proxy(nRobot,nTask,nWays,cost_mat.astype(np.float32).flatten())
        alloc_mat = np.array(response.allocation_matrix).reshape((nRobot, nTask*nWays))
        print(alloc_mat)
    except rospy.ServiceException as e:
        print("Service call failed:", e)

    return np.where(alloc_mat)


def publish_paths(rob,taskw,path_publishers,path_db):
    for p in range(len(rob)):
        path_to_pub = path_db[rob[p]][taskw[p]]
        nSub = path_publishers[p].get_num_connections()
        if(nSub):
            path_publishers[p].publish(path_to_pub)

def generate_cost_matrix(nRobot:int,nTask:int,nWays:int,path_db:dict,task_list:list,agents_list:list,alloc_type='pickup', print_cost=True):
    cost_mat = np.zeros((nRobot, nTask*nWays), dtype=float)

    for robot in range(nRobot):
        #robot path dict
        path_db[robot] = {}
        for task in range(nTask):
            #init task
            pot_goals = task_list[task].get_goal_poses() #list of pose2D

            #dict to be stored in row dict for paths
            temp_task_dict = {}

            for way in range(nWays):
                if(alloc_type == 'pickup'):
                    cost_single, planned_path = req_plan_pose2D(agents_list[robot].get_start_pose()
                                                ,pot_goals[way])
                    if(print_cost):
                        print(cost_single)
                    task_ind = task*nWays
                    if(cost_single > 0):                    
                        cost_mat[robot,task_ind + way] = cost_single
                        path_db[robot][task_ind + way] = planned_path
                    else:
                        #large cost
                        cost_mat[robot,task_ind + way] = 10000000

                elif(alloc_type == 'delivery'):
                    
                    angle_cost = agents_list[robot].get_angle_cost(pot_goals[way])
                    if(print_cost):
                        print(angle_cost)
                    task_ind = task*nWays
                    cost_mat[robot,task_ind + way] = angle_cost


    return cost_mat