#include<iostream>
#include<ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include"gps_to_local/waypoints.h"
#include"gps_to_local/fcc.h"
#include<cmath>
#include<vector>
#include<GeographicLib/Geocentric.hpp>
#include<GeographicLib/LocalCartesian.hpp>

bool flag = false;

double lat0,lon0,alt0;
double roll,pitch,yaw;
gps_to_local::fcc origin_position;
tf2::Quaternion enu_q;
double local_x,local_y,local_z;//起始点局部坐标
double origin_position_east, origin_position_north, origin_position_up;//起始点的东北天坐标

ros::Publisher pub_gps;

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
    // 计算旋转矩阵元素
    double cosRoll = cos(rollRad);
    double sinRoll = sin(rollRad);
    double cosPitch = cos(pitchRad);
    double sinPitch = sin(pitchRad);
    double cosYaw = cos(yawRad);
    double sinYaw = sin(yawRad);
    // 应用旋转矩阵
    x = east_ * cosYaw * cosPitch + north_ * (cosYaw * sinPitch * sinRoll - sinYaw * cosRoll) + up_ * (cosYaw * sinPitch * cosRoll + sinYaw * sinRoll);
    y = east_ * sinYaw * cosPitch + north_ * (sinYaw * sinPitch * sinRoll + cosYaw * cosRoll) + up_ * (sinYaw * sinPitch * cosRoll - cosYaw * sinRoll);
    z = -east_ * sinPitch + north_ * cosPitch * sinRoll + up_ * cosPitch * cosRoll;

}

void Originpoint_CallBack(const gps_to_local::fcc::ConstPtr& originmsg){//得到起始点局部坐标
    if(!flag){
        origin_position=*originmsg;
        lat0=origin_position.latitude;
        lon0=origin_position.longitude;
        alt0=origin_position.altitude;

        roll=origin_position.roll;
        pitch=origin_position.pitch;
        yaw=origin_position.orientation_uav;

        //转东北天坐标
        GeographicLib::Geocentric earth(
            GeographicLib::Constants::WGS84_a(),
            GeographicLib::Constants::WGS84_f()
        );
        // 将经纬高坐标转换为 ECEF 坐标
        double ECEF_x, ECEF_y, ECEF_z;
        earth.Forward(origin_position.latitude, origin_position.longitude, origin_position.altitude, ECEF_x, ECEF_y, ECEF_z);
        // 创建 LocalCartesian 对象
        GeographicLib::LocalCartesian proj(
            origin_position.latitude, origin_position.longitude, origin_position.altitude, earth
        );
            // 将 ECEF 坐标转换为东北天坐标
        proj.Reverse(ECEF_x, ECEF_y, ECEF_z, origin_position_north, origin_position_east, origin_position_up);

        // 输出东北天坐标
        std::cout << "North_origin_position: " << origin_position_north << std::endl;
        std::cout << "East_origin_position: " << origin_position_east << std::endl;
        std::cout << "Up_origin_position: " << origin_position_up << std::endl;
        
        NEUtoXYZ(origin_position_north,origin_position_east,origin_position_up,
                origin_position.roll,origin_position.pitch,static_cast<double>(origin_position.orientation_uav),
                local_x,local_y,local_z);
    }
    flag=true;
    return;
}



void local_to_gpsCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg){
    geometry_msgs::PoseStamped local_msg;
    double enu_x,enu_y,enu_z;
    local_msg=*msg;
    local_msg.pose.position.x+=local_x;
    local_msg.pose.position.y+=local_y;
    local_msg.pose.position.z+=local_z;

    //将局部坐标转换为东北天坐标
    enu_q.setRPY(origin_position.roll,origin_position.pitch,origin_position.orientation_uav);

    tf2::Matrix3x3 rot_matrix(enu_q);

    double x = local_msg.pose.position.x;
    double y = local_msg.pose.position.y;
    double z = local_msg.pose.position.z;
    enu_x = origin_position_east + rot_matrix[0][0] * x + rot_matrix[0][1] * y + rot_matrix[0][2] * z;
    enu_y = origin_position_north + rot_matrix[1][0] * x + rot_matrix[1][1] * y + rot_matrix[1][2] * z;
    enu_z = origin_position_up + rot_matrix[2][0] * x + rot_matrix[2][1] * y + rot_matrix[2][2] * z;
    //东北天转gps
    GeographicLib::LocalCartesian proj;
    proj.Reset(origin_position_east,origin_position_north,origin_position_up);
    // 计算相对位置
    double dx = enu_x - origin_position_east;
    double dy = enu_y - origin_position_north;
    double dz = enu_z - origin_position_up;

    double lat,lon,alt;
    sensor_msgs::NavSatFix gps_message;
    // 转换为经纬高坐标
    proj.Reverse(dy, dx, dz, lat, lon, alt);
    gps_message.header=local_msg.header;
    gps_message.latitude=lat;
    gps_message.longitude=lon;
    gps_message.altitude=alt;
    pub_gps.publish(gps_message);
}

int main(int argc,char** argv){
    ros::init(argc,argv,"local_to_gps");
    ros::NodeHandle nh;
    ros::Subscriber sub_target=nh.subscribe<geometry_msgs::PoseStamped>("/pose_cmd",10,local_to_gpsCallBack);
    ros::Subscriber sub=nh.subscribe<gps_to_local::fcc>("/fcc_messages",10,Originpoint_CallBack);
    pub_gps=nh.advertise<sensor_msgs::NavSatFix>("/target_gps",10);
    ros::spin();

    return 0;
}