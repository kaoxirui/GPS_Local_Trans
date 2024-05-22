#ifndef _LOCAL_TO_GPS
#define _LOCAL_TO_GPS

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cmath>
#include <iostream>
#include <vector>

#include "gps_to_local/fcc.h"
#include "gps_to_local/waypoints.h"

class Local_To_Gps_Trans {
 public:
  Local_To_Gps_Trans();
  void OriginCallBack(const gps_to_local::fcc::ConstPtr &originmsg);

  void EnuToWgs(double &lat, double &lon, double &h, double lat0, double lon0,
                double h0, double xEast, double yNorth, double zUp);

  void LocalToGpsCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg);

 private:
  ros::NodeHandle nh;
  ros::Subscriber sub_target;
  ros::Subscriber sub_ori;
  ros::Publisher pub_gps;
  bool flag = false;
  double lat0, lon0, alt0;
  double roll, pitch, yaw;
  gps_to_local::fcc origin_position;
};

#endif
