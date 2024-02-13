from transformation_tools import fit_angle_rad, pose2D, normalize
import math



class pickup_task:
    """
    Pick-up task object
    """
    def __init__(self, pos_x_in, pos_y_in, angle_res:int=72, obj_size_in:float=0.2, arrival_offset:float=0.55) -> None:
        self.x = pos_x_in
        self.y = pos_y_in

        self.size = obj_size_in
        self.arr_off = arrival_offset

        self.poses = [0,fit_angle_rad(1.5708,angle_res),fit_angle_rad(3.1415,angle_res),fit_angle_rad(-1.5708,angle_res)]


    def get_goal_poses(self) -> list:
        #todo: use loop
        margin = (self.size/2) + self.arr_off
        return [pose2D(self.x - margin,self.y,self.poses[0]),
                pose2D(self.x,self.y - margin,self.poses[1]),
                pose2D(self.x + margin,self.y,self.poses[2]),
                pose2D(self.x,self.y+margin,self.poses[3])]
    
    def get_center(self) -> pose2D:
        return pose2D(self.x, self.y, self.poses[0])
    

class delivery_task:
    """
    Delivery task object
    """
    def __init__(self, pos_x_in, pos_y_in, obj_size_in:float=0.2, arrival_offset:float=0.55) -> None:
        self.x = pos_x_in
        self.y = pos_y_in

        self.size = obj_size_in
        self.arr_off = arrival_offset

    def get_angle_cost(self, start_pose:pose2D) -> float:
        #push vector
        push_vec = (math.cos(start_pose.th),math.sin(start_pose.th))

        #straight line vector
        line_vec_not_normalized = (self.x - start_pose.x, self.y - start_pose.y)
        #normalize
        line_vec = normalize(line_vec_not_normalized)

        #return 1 - dot product
        #return 1.0 - math.fabs(push_vec[0]*line_vec[0] + push_vec[1]*line_vec[1])


        #return acos(dot)
        return math.acos(push_vec[0]*line_vec[0] + push_vec[1]*line_vec[1])


    def get_center(self) -> pose2D:
        return pose2D(self.x, self.y, 0)