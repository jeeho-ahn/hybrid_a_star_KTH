from transformation_tools import pose2D

class agent:
    def __init__(self, start_x, start_y, start_th) -> None:
        self.start_pose = pose2D(start_x,start_y,start_th)

    def get_start_pose(self) -> pose2D:
        return self.start_pose