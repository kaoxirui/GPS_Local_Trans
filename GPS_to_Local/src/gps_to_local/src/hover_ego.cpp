#include<iostream>
#include<ros/ros.h>
#include<geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include"gps_to_local/waypoints.h"
#include"gps_to_local/fcc.h"
#include"gps_to_local/vectormessage.h"

ros::Publisher pub_target;
// int count_points=0;

bool check_target(geometry_msgs::PoseStamped set_pose, geometry_msgs::PoseStamped current_pose){
    double dx = set_pose.pose.position.x - current_pose.pose.position.x;
    double dy = set_pose.pose.position.y - current_pose.pose.position.y;
    double dz = set_pose.pose.position.z - current_pose.pose.position.z;

    if (dx * dx + dy * dy + dz * dz < 1.2)
        return true;
    else
        return false;
}

geometry_msgs::PoseStamped current_pose;
void odom_call_back(const nav_msgs::Odometry odomMsg){
    current_pose.pose.position.x=odomMsg.pose.pose.position.x;
    current_pose.pose.position.y=odomMsg.pose.pose.position.y;
    current_pose.pose.position.z=odomMsg.pose.pose.position.z;
}

void waypoints_callback(const gps_to_local::vectormessageConstPtr &ego_wayoints){
    const std::vector<geometry_msgs::PoseStamped>& poses=ego_wayoints->data;
    size_t index=0;
    while(index<poses.size()){
        if(check_target(poses[index],current_pose)){
            index++;
            if(index<poses.size()){
                ROS_INFO("Reached waypoint %d,moving to the next target",index);
                pub_target.publish(poses[index]);
            }else{
                ROS_INFO("Reached final waypoint");
            }
        }
    }
}

int main(int argc,char** argv){
    ros::init(argc,argv,"hover_ego");
    ros::NodeHandle nh;
    ros::Subscriber sub_odom=nh.subscribe<nav_msgs::Odometry>("/Odometry",10,odom_call_back);
    ros::Subscriber sub_waypoint=nh.subscribe("/publish_points",10,waypoints_callback);
    pub_target=nh.advertise<geometry_msgs::PoseStamped>("/local_planner/goal_position",10);

    ros::spin();

}