#include<iostream>
#include<ros/ros.h>
#include<GeographicLib/Geocentric.hpp>
#include<GeographicLib/LocalCartesian.hpp>
#include"gps_to_local/waypoints.h"
#include"gps_to_local/fcc.h"
#include<cmath>
#include<vector>
#include<fstream>
#include<string>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc,char** argv){

    ros::init(argc,argv,"pub_loca");
    ros::NodeHandle nh;
    ros::Publisher pub=nh.advertise<geometry_msgs::PoseStamped>("/pose_cmd",10);

    geometry_msgs::PoseStamped localposition;
    localposition.pose.position.x=64858.6;
    localposition.pose.position.y=-105729;
    localposition.pose.position.z=-1106.7;

    ros::Rate r(20);
    while (ros::ok())
    {
        pub.publish(localposition);
        r.sleep();
        ros::spinOnce();
    }
}