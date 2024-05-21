#include<iostream>
#include<ros/ros.h>
#include<GeographicLib/Geocentric.hpp>
#include<GeographicLib/LocalCartesian.hpp>
#include"gps_to_local/waypoints.h"
#include"gps_to_local/fcc.h"
#include"gps_to_local/vectormessage.h"
#include<cmath>
#include<vector>
#include<fstream>
#include<string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>


bool flag=false;
bool init=false;
gps_to_local::fcc origin_points;
gps_to_local::vectormessage waypoints_data;
std::vector<geometry_msgs::PoseStamped>wayponits;
int count=0;//航点个数
ros::Publisher pub_ego;

double toRadians(double degrees){
    return degrees*M_PI/180.0;
}


void NEUtoXYZ(double north_,double east_,double up_,
                double roll,double pitch,double yaw,
                double& x,double& y,double& z){
    // 将角度转为弧度
    double rollRad = toRadians(roll);
    double pitchRad = toRadians(pitch);
    double yawRad = toRadians(yaw);
    tf2::Matrix3x3 R_ENU_to_local;
    R_ENU_to_local.setEulerZYX(yawRad-3.14159265359/2,pitchRad,rollRad);
    x = R_ENU_to_local[0][0] * east_ + 
          R_ENU_to_local[0][1] * north_ +
          R_ENU_to_local[0][2] * up_;
    y = R_ENU_to_local[1][0] * east_+
          R_ENU_to_local[1][1] * north_+
          R_ENU_to_local[1][2] * up_;
    z = R_ENU_to_local[2][0] * east_ +
          R_ENU_to_local[2][1] * north_ +
          R_ENU_to_local[2][2] * up_;
    // // 计算旋转矩阵元素
    // double cosRoll = cos(rollRad);
    // double sinRoll = sin(rollRad);
    // double cosPitch = cos(pitchRad);
    // double sinPitch = sin(pitchRad);
    // double cosYaw = cos(yawRad);
    // double sinYaw = sin(yawRad);
    // // 应用旋转矩阵
    // x = east_ * cosYaw * cosPitch + north_ * (cosYaw * sinPitch * sinRoll - sinYaw * cosRoll) + up_ * (cosYaw * sinPitch * cosRoll + sinYaw * sinRoll);
    // y = east_ * sinYaw * cosPitch + north_ * (sinYaw * sinPitch * sinRoll + cosYaw * cosRoll) + up_ * (sinYaw * sinPitch * cosRoll - cosYaw * sinRoll);
    // z = -east_ * sinPitch + north_ * cosPitch * sinRoll + up_ * cosPitch * cosRoll;
    // std::cout<<x<<std::endl;
    // std::cout<<y<<std::endl;
    // std::cout<<z<<std::endl;
    return;
}


void Originpoint_CallBack(const gps_to_local::fcc::ConstPtr& originmsg){
    if(!flag){
        origin_points=*originmsg;
        ROS_INFO("Originpoint_CallBack reveived message!!");
        std::cout<<"origin"<<origin_points.latitude<<" "<<origin_points.longitude<<" "<<origin_points.altitude<<std::endl;
        flag=true;
    }
    //return;
} 
  



void WGS_TO_ENU(double lat, double lon, double h, double lat0, double lon0, double h0, double &enu_x,double &enu_y,double &enu_z)
{
	double a, b, f, e_sq, pi;
    pi = 3.14159265359;
	a = 6378137;
	b = 6356752.3142;
	f = (a - b) / a;
	e_sq = f * (2 - f);
	// 站点（非原点）
	double lamb, phi, s, N, sin_lambda, cos_lambda, sin_phi, cos_phi, x, y, z;
	lamb = lat/180.0*pi;  
	phi = lon/180.0*pi;
	s = sin(lamb);
	N = a / sqrt(1 - e_sq * s * s);
 
	sin_lambda = sin(lamb);
	cos_lambda = cos(lamb);
	sin_phi = sin(phi);
	cos_phi = cos(phi);
 
	x = (h + N) * cos_lambda * cos_phi;
	y = (h + N) * cos_lambda * sin_phi;
	z = (h + (1 - e_sq) * N) * sin_lambda;
	// 原点坐标转换
	double lamb0, phi0, s0, N0, sin_lambda0, cos_lambda0, sin_phi0, cos_phi0, x0, y0, z0;
	lamb0 = lat0/180.0*pi;
	phi0 = lon0/180.0*pi;
	s0 = sin(lamb0);
	N0 = a / sqrt(1 - e_sq * s0 * s0);
 
	sin_lambda0 = sin(lamb0);
	cos_lambda0 = cos(lamb0);
	sin_phi0 = sin(phi0);
	cos_phi0 = cos(phi0);
 
	x0 = (h0 + N0) * cos_lambda0 * cos_phi0;
	y0 = (h0 + N0) * cos_lambda0 * sin_phi0;
	z0 = (h0 + (1 - e_sq) * N0) * sin_lambda0;
	// ECEF 转 ENU
	double xd, yd, zd, t;
	xd = x - x0;
	yd = y - y0;
	zd = z - z0;
	t = -cos_phi0 * xd - sin_phi0 * yd;
 
	enu_x = -sin_phi0 * xd + cos_phi0 * yd;
	enu_y = t * sin_lambda0 + cos_lambda0 * zd;
	enu_z = cos_lambda0 * cos_phi0 * xd + cos_lambda0 * sin_phi0 * yd + sin_lambda0 * zd;
    return;
}
// //判断是不是到位置了
// bool check_target(geometry_msgs::PoseStamped set_pose, geometry_msgs::PoseStamped current_pose_){
//     double dx = set_pose.pose.position.x - current_pose_.pose.position.x;
//     double dy = set_pose.pose.position.y - current_pose_.pose.position.y;
//     double dz = set_pose.pose.position.z - current_pose_.pose.position.z;

//     if (dx * dx + dy * dy + dz * dz < 1)
//         return true;
//     else
//         return false;
// }


// //订阅当前的里程计中的位置信息
// geometry_msgs::PoseStamped current_pose;
// void odom_call_back(const nav_msgs::Odometry odomMsg){
//     current_pose.pose.position.x=odomMsg.pose.pose.position.x;
//     current_pose.pose.position.y=odomMsg.pose.pose.position.y;
//     current_pose.pose.position.z=odomMsg.pose.pose.position.z;
// }


//GeographicLib::LocalCartesian geoConverter;
void gps_to_localCallBack(const gps_to_local::waypoints::ConstPtr& msg){
    gps_to_local::waypoints current_point=*msg;
    //转为东北天
    double east,north,up;
    
    // if(flag&&!init){
    //     geoConverter.Reset(origin_points.latitude,origin_points.longitude,origin_points.altitude);
    //     init=true;
    // }
    //     geoConverter.Forward(current_point.latitude,current_point.longitude,current_point.altitude,east,north,up);
    //void WGS_TO_ENU(double lat, double lon, double h, double lat0, double lon0, double h0, double &enu_x,double &enu_y,double &enu_z);

    WGS_TO_ENU(current_point.latitude,current_point.longitude,current_point.altitude,
                 origin_points.latitude,origin_points.longitude,origin_points.altitude,east,north,up);
    // 输出东北天坐标
    std::cout << "East: " << east << std::endl;
    std::cout << "North: " << north << std::endl;
    std::cout << "Up: " << up << std::endl;
    //局部坐标
    double waypoints_local_x,waypoints_local_y,waypoints_local_z;
    NEUtoXYZ(north,east,up,
        origin_points.roll,origin_points.pitch,static_cast<double>(origin_points.orientation_uav),
        waypoints_local_x,waypoints_local_y,waypoints_local_z);

    //将局部坐标存储在wayponits数组中
    geometry_msgs::PoseStamped waypoints_to_use;
    waypoints_to_use.pose.position.x=waypoints_local_x;
    waypoints_to_use.pose.position.y=waypoints_local_y;
    waypoints_to_use.pose.position.z=waypoints_local_z;
    wayponits.push_back(waypoints_to_use);
    if(wayponits.size()==count+1){
        waypoints_data.data=wayponits;
        pub_ego.publish(waypoints_data);
        ros::spinOnce();
    }

    double id=static_cast<double>(current_point.id);
    std::ofstream outfile;
    outfile.open("/home/kao/GPS_to_Local/src/gps_to_local/src/output.txt",std::ios_base::app);
    if(outfile.is_open()){
        outfile<<"id: "<<id<<std::endl;
        outfile<<"position:["<<waypoints_local_x<<","<<waypoints_local_y<<","<<waypoints_local_z<<"]"<<std::endl;
        outfile<<std::endl;
    }else {
        ROS_ERROR("Output file is not open, unable to write coordinates.");
    }
    outfile.close();
}
int main(int argc,char** argv){
    ros::init(argc,argv,"gps_to_local");
    ros::NodeHandle nh;
    
    //订阅起始点经纬高
    ros::Subscriber sub_2=nh.subscribe<gps_to_local::fcc>("/fcc_messages",10,Originpoint_CallBack);
    //订阅航点经纬高信息
    ros::Subscriber sub_1=nh.subscribe<gps_to_local::waypoints>("/waypoints_messages",10,gps_to_localCallBack);
    //订阅里程计信息
    //ros::Subscriber sub_odom=nh.subscribe<nav_msgs::Odometry>("/Odometry",10,odom_call_back);
    //发布航点信息
    pub_ego=nh.advertise<gps_to_local::vectormessage>("/publish_points",10);
    gps_to_local::fccConstPtr msg=ros::topic::waitForMessage<gps_to_local::fcc>("/fcc_messages",ros::Duration(5.0));
    if(msg){
        Originpoint_CallBack(msg);
    }else{
        ROS_INFO("TIME OUT");
    }
    ros::spin();

    return 0;
}