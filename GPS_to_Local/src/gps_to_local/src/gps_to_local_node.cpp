#include"gps_to_local/gps_to_local.hpp"
//按照源文件方式配置，这个可执行文件依赖另一个cpp文件
int main(int argc,char** argv){
    ros::init(argc,argv,"gps_to_local");
    ros::NodeHandle nh;

    Gps_Local_Trans Trans;

    gps_to_local::fccConstPtr msg=ros::topic::waitForMessage<gps_to_local::fcc>("/fcc_messages",ros::Duration(5.0));
    if(msg){
         Trans.Originpoint_CallBack(msg);
    }else{
        ROS_INFO("TIME OUT");
    }
    ros::spin();

    return 0;
}