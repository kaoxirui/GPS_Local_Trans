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

int main(int argc,char** argv){
    gps_to_local::waypoints watpoint;
    gps_to_local::fcc start_position;

    watpoint.id=1;
    watpoint.latitude=31.3456789;
    watpoint.longitude=121.2345678;
    watpoint.altitude=300;

    start_position.latitude=30.7654321;
    start_position.longitude=120.1234567;
    start_position.altitude=200;
    // start_position.latitude=0;
    // start_position.longitude=0;
    // start_position.altitude=0;
    start_position.roll=0;
    start_position.pitch=0;
    start_position.orientation_uav=0;

    ros::init(argc,argv,"pub");
    ros::NodeHandle nh;
    ros::Publisher pub_1=nh.advertise<gps_to_local::waypoints>("/waypoints_messages",10);
    ros::Publisher pub_2=nh.advertise<gps_to_local::fcc>("/fcc_messages",10);

    ros::Rate r(20);
    while (ros::ok())
    {
        pub_2.publish(start_position);
        pub_1.publish(watpoint);
        r.sleep();
        ros::spinOnce();
    }
    
}