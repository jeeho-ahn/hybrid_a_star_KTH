// Inside your pick point subscriber node

#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "visualization_msgs/Marker.h"
#include <iostream>
#include <hybrid_astar/cubeReqSrv.h>
#include <std_srvs/Trigger.h>

ros::Publisher* marker_pub_ptr = nullptr;
ros::ServiceClient* srvCubeReq_ptr = nullptr;
ros::ServiceClient* srvPushCostReq_ptr = nullptr;

void pickPointCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    // Process the pick point data
    double x = msg->point.x;
    double y = msg->point.y;
    double z = msg->point.z;

    // Publish the cube marker at the pick point
    visualization_msgs::Marker cube_marker;
    cube_marker.header.frame_id = "map";  // Replace with the appropriate frame
    cube_marker.type = visualization_msgs::Marker::CUBE;
    cube_marker.pose.position.x = x;
    cube_marker.pose.position.y = y;
    cube_marker.pose.position.z = z;
    cube_marker.scale.x = 0.2;
    cube_marker.scale.y = 0.2;
    cube_marker.scale.z = 0.2;
    cube_marker.color.r = 1.0;
    cube_marker.color.g = 0.0;
    cube_marker.color.b = 0.0;
    cube_marker.color.a = 1.0;

    marker_pub_ptr->publish(cube_marker);

    //request add cube
    hybrid_astar::cubeReqSrv cube_req;
    cube_req.request.cmd.data="replace";
    cube_req.request.pose_in.position=msg->point;
    cube_req.request.pose_in.orientation = geometry_msgs::Quaternion();
    srvCubeReq_ptr->call(cube_req);

    std_srvs::Trigger treq;
    srvPushCostReq_ptr->call(treq);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pick_point_subscriber");
    ros::NodeHandle nh;

    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("cube_marker", 1);
    marker_pub_ptr = &marker_pub;

    ros::ServiceClient srvCubeReq = nh.serviceClient<hybrid_astar::cubeReqSrv>("/cube_req");
    srvCubeReq_ptr = &srvCubeReq;

    ros::ServiceClient srvPushCostReq = nh.serviceClient<std_srvs::Trigger>("/cube_pickup_cost_req");
    srvPushCostReq_ptr = &srvPushCostReq;

    std::cout << "Starting cube drawing" << std::endl;

    // Replace "/rviz/pick_point" with the actual pick point topic
    std::string pickPointTopic = "/clicked_point";

    ros::Subscriber pickPointSub = nh.subscribe(pickPointTopic, 1, pickPointCallback);

    ros::spin();

    return 0;
}
