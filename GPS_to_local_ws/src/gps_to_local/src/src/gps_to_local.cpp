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



bool flag=false;
gps_to_local::fcc origin_points;

double toRadians(double degrees){
    return degrees*M_PI/180.0;
}

std::vector<std::vector<double>>origin_vec;
std::vector<std::vector<double>>way_points_vec;
std::vector<std::vector<double>>relative_waypoints_vec;

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


void Originpoint_CallBack(const gps_to_local::fcc::ConstPtr& originmsg){
    if(!flag){
        origin_points=*originmsg;
        GeographicLib::Geocentric earth(
            GeographicLib::Constants::WGS84_a(),
            GeographicLib::Constants::WGS84_f()
        );
        double local_x,local_y,local_z;
        // 将经纬高坐标转换为 ECEF 坐标
        double ECEF_x, ECEF_y, ECEF_z;
        earth.Forward(origin_points.latitude, origin_points.longitude, origin_points.altitude, ECEF_x, ECEF_y, ECEF_z);
        // 创建 LocalCartesian 对象
        GeographicLib::LocalCartesian proj(
            origin_points.latitude, origin_points.longitude, origin_points.altitude, earth
        );
            // 将 ECEF 坐标转换为东北天坐标
        double origin_points_north, origin_points_east, origin_points_up;
        proj.Reverse(ECEF_x, ECEF_y, ECEF_z, origin_points_north, origin_points_east, origin_points_up);

        // 输出东北天坐标
        std::cout << "North_origin_points: " << origin_points_north << std::endl;
        std::cout << "East_origin_points: " << origin_points_east << std::endl;
        std::cout << "Up_origin_points: " << origin_points_up << std::endl;
        NEUtoXYZ(origin_points_north,origin_points_east,origin_points_up,
                origin_points.roll,origin_points.pitch,static_cast<double>(origin_points.orientation_uav),
                local_x,local_y,local_z);
        std::vector<double>vec_temp;
        vec_temp.push_back(local_x);
        vec_temp.push_back(local_y);
        vec_temp.push_back(local_z);

        origin_vec.push_back(vec_temp);
    }
    flag=true;
    return;
}   

void gps_to_localCallBack(const gps_to_local::waypoints::ConstPtr& msg){
    gps_to_local::waypoints current_point=*msg;
    
    GeographicLib::Geocentric earth(
        GeographicLib::Constants::WGS84_a(),
        GeographicLib::Constants::WGS84_f()
    );

    // 将经纬高坐标转换为 ECEF 坐标
    double x, y, z;
    earth.Forward(current_point.latitude, current_point.longitude, current_point.altitude, x, y, z);
    // 创建 LocalCartesian 对象
    GeographicLib::LocalCartesian proj(
        current_point.latitude, current_point.longitude, current_point.altitude, earth
    );
        // 将 ECEF 坐标转换为东北天坐标
    double north, east, up;
    proj.Reverse(x, y, z, north, east, up);

    // 输出东北天坐标
    std::cout << "North: " << north << std::endl;
    std::cout << "East: " << east << std::endl;
    std::cout << "Up: " << up << std::endl;
    //局部坐标
    double waypoints_local_x,waypoints_local_y,waypoints_local_z;
    NEUtoXYZ(north,east,up,
        origin_points.roll,origin_points.pitch,static_cast<double>(origin_points.orientation_uav),
        waypoints_local_x,waypoints_local_y,waypoints_local_z);
    std::vector<double>vec_temp_waypoints;
    vec_temp_waypoints.push_back(waypoints_local_x);
    vec_temp_waypoints.push_back(waypoints_local_y);
    vec_temp_waypoints.push_back(waypoints_local_z);

    way_points_vec.push_back(vec_temp_waypoints);
    return;
}
int main(int argc,char** argv){
    ros::init(argc,argv,"gps_to_local");
    ros::NodeHandle nh;
    ros::Subscriber sub_1=nh.subscribe<gps_to_local::waypoints>("/waypoints_messages",10,gps_to_localCallBack);
    ros::Subscriber sub_2=nh.subscribe<gps_to_local::fcc>("/fcc_messages",10,Originpoint_CallBack);
    ros::spin();
    std::cout<<"computing relative cordinates...."<<std::endl;
    for (size_t i = 0; i < way_points_vec.size(); i++)
    {
        std::vector<double>relative_point_temp;
        relative_point_temp[0]=way_points_vec[i][0]-origin_vec[0][0];
        relative_point_temp[1]=way_points_vec[i][1]-origin_vec[0][1];
        relative_point_temp[2]=way_points_vec[i][2]-origin_vec[0][2];
        relative_waypoints_vec.push_back(relative_point_temp);       
    }
    std::cout<<"the result is: "<<std::endl;
    for(const auto& vec:relative_waypoints_vec){
        std::cout<<"[";
        for(double value:vec){
            std::cout<<value<<",";
        }
        std::cout<<"]"<<std::endl;
    }

    std::ofstream outfile("possition.txt");
    // 检查文件是否打开成功
    if (outfile.is_open()) {
        // 遍历 vector 并写入文件
        for(const auto& vec:relative_waypoints_vec){
            for(double value:vec){
                outfile << value << " ";
            }
            outfile<<"\n";
    }
        // 关闭文件
        outfile.close();
        std::cout << "Data written to file successfully." << std::endl;
    } else {
        std::cerr << "Unable to open file for writing." << std::endl;
    }


    return 0;
}