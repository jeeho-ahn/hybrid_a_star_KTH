import math
from geometry_msgs.msg import Pose

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

def fit_angle_rad(ang_in,ang_resol):
    """
    Round angle to the discrete angle steps defined in Hybrid A*
    """
    ang_step = 2*math.pi/float(ang_resol)
    return round(ang_in/ang_step) * ang_step

def normalize(v):
    magnitude = math.sqrt(v[0]**2 + v[1]**2)
    if magnitude == 0:
        return (0, 0)  # Avoid division by zero
    normalized_vector = (v[0] / magnitude, v[1] / magnitude)
    return normalized_vector