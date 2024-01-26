#ifndef CUBE_BLOCK
#define CUBE_BLOCK

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <memory>


namespace jeeho
{
    class cube
    {
    public:
      float x;
      float y;
      float th;
      float push_dir[4] = {0,1.5708,3.14159,4.71239};
      float size = 0.2f;
      cube();
      cube(geometry_msgs::Pose pose_in);
      std::shared_ptr<std::vector<geometry_msgs::PoseStamped>> pose_candidates();
      Eigen::Vector3f get_pos_eigen();
    };
}

#endif
