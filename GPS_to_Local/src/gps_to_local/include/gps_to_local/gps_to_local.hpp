#ifndef _GPS_TO_LOCAL
#define _GPS_TO_LOCAL
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "gps_to_local/fcc.h"
#include "gps_to_local/vectormessage.h"
#include "gps_to_local/waypoints.h"

class Gps_Local_Trans {
 public:
  Gps_Local_Trans();

  void EnuToXyz(double north_, double east_, double up_, double roll,
                double pitch, double yaw, double& x, double& y, double& z);

  void WgsToEnu(double lat, double lon, double h, double lat0, double lon0,
                double h0, double& enu_x, double& enu_y, double& enu_z);

  void OriginPointCallBack(const gps_to_local::fcc::ConstPtr& originmsg);

  void GpsToLocalCallBack(const gps_to_local::waypoints::ConstPtr& msg);

 private:
  ros::NodeHandle nh;
  ros::Subscriber sub_1;   //订阅航点经纬高
  ros::Subscriber sub_2;   //订阅起始点经纬高
  ros::Publisher pub_ego;  //发布航点信息

  bool flag;

  gps_to_local::fcc origin_points;
  gps_to_local::vectormessage waypoints_data;
  std::vector<geometry_msgs::PoseStamped> wayponits;
  int count = 0;  //航点个数
};

#endif