from transformation_tools import fit_angle_rad, pose2D

class pickup_task:
    """
    Pick-up task object
    """
    def __init__(self, pos_x_in, pos_y_in, angle_res:int=72, size_in:float=0.2, arrival_offset:float=0.55) -> None:
        self.x = pos_x_in
        self.y = pos_y_in

        self.size = size_in
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