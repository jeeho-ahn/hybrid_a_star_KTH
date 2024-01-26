#include<cube_block.h>

// Convert quaternion to Euler angles (roll, pitch, yaw)
Eigen::Vector3d quaternionToEulerAngles(geometry_msgs::Quaternion q) {
    Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
    Eigen::Vector3d euler = quat.toRotationMatrix().eulerAngles(2, 1, 0); // ZYX order
    return euler;
}

Eigen::Vector2f rotateVector(Eigen::Vector2f vec_in, float angle) {
    Eigen::Vector2f out_vec;
    out_vec.x() = vec_in.x() * std::cos(angle) - vec_in.y() * std::sin(angle);
    out_vec.y() = vec_in.x() * std::sin(angle) + vec_in.y() * std::cos(angle);

    return out_vec;
}

geometry_msgs::Quaternion yawToQuaternion(double yaw) {
    geometry_msgs::Quaternion out_q;
    out_q.w = std::cos(yaw / 2.0);
    out_q.x = 0.0;
    out_q.y = 0.0;
    out_q.z = std::sin(yaw / 2.0);

    return out_q;
}

Eigen::Vector3f jeeho::cube::get_pos_eigen()
{
  return Eigen::Vector3f(x,y,size/2);
}

jeeho::cube::cube()
{

}

jeeho::cube::cube(geometry_msgs::Pose pose_in)
{
  x = pose_in.position.x;
  y = pose_in.position.y;
  //2D orientation
  th = quaternionToEulerAngles(pose_in.orientation)[0];

  std::cout << quaternionToEulerAngles(pose_in.orientation) << std::endl;
}

std::shared_ptr<std::vector<geometry_msgs::PoseStamped>> jeeho::cube::pose_candidates()
{
  Eigen::Vector2f uvec = Eigen::Vector2f(1,0);
  std::vector<Eigen::Vector2f> unit_vecs(4);
  std::vector<geometry_msgs::PoseStamped> pose_cands(4);
  for (int i=0;i<4;i++) {
    unit_vecs[i] = rotateVector(uvec,push_dir[i]);
    auto land_pos = get_pos_eigen();
    auto disp_vec = unit_vecs[i] * size;
    land_pos.x() = land_pos.x() - disp_vec.x();
    land_pos.y() = land_pos.y() - disp_vec.y();
    pose_cands[i].pose.position.x = land_pos.x();
    pose_cands[i].pose.position.y = land_pos.y();
    pose_cands[i].pose.orientation = yawToQuaternion(push_dir[i]);
  }


  return std::make_shared<std::vector<geometry_msgs::PoseStamped>>(pose_cands);
}
