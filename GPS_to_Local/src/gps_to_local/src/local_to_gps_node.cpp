#include "gps_to_local/local_to_gps.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "local_to_gps");
  ros::NodeHandle nh;
  Local_To_Gps_Trans Local_Trans;
  gps_to_local::fccConstPtr msg = ros::topic::waitForMessage<gps_to_local::fcc>(
      "/fcc_messages", ros::Duration(5.0));
  if (msg) {
    Local_Trans.OriginCallBack(msg);
  } else {
    ROS_INFO("TIME OUT");
  }
  ros::spin();

  return 0;
}