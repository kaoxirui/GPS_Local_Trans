#include "gps_to_local/gps_to_local.hpp"

Gps_Local_Trans::Gps_Local_Trans() : nh("~"), flag(false) {
    /*
      this 表示当前 Gps_Local_Trans 类的实例。它用于将 Originpoint_CallBack
  成员函数作为回调函数传递给 nh.subscribe 方法。

  在这种情况下,this 指针是必需的,因为 Originpoint_CallBack
  是一个非静态成员函数,需要访问类的非静态成员变量。传递 this 指针可以让
  Originpoint_CallBack 函数访问到当前 Gps_Local_Trans 类实例的成员变量。

  如果 Originpoint_CallBack 是一个静态成员函数,就不需要传递 this
  指针了,因为静态成员函数不需要 this 指针就可以访问类的成员变量和成员函数。
  */
    //订阅航点经纬高信息
    sub_1 = nh.subscribe<gps_to_local::waypoints>("/waypoints_messages", 10,
                                                  &Gps_Local_Trans::GpsToLocalCallBack, this);

    //订阅起始点经纬高
    sub_2 = nh.subscribe<gps_to_local::fcc>("/fcc_messages", 10,
                                            &Gps_Local_Trans::OriginPointCallBack, this);
    //发布航点信息
    pub_ego = nh.advertise<gps_to_local::vectormessage>("/publish_points", 10);
}

void Gps_Local_Trans::EnuToXyz(double north_, double east_, double up_, double roll, double pitch,
                               double yaw, double &x, double &y, double &z) {
    // 将角度转为弧度
    double rollRad = roll;
    double pitchRad = pitch;
    double yawRad = yaw;
    tf2::Matrix3x3 R_ENU_to_local;
    //R_ENU_to_local.setEulerZYX(yawRad - 3.14159265359 / 2, pitchRad, -rollRad);
    R_ENU_to_local.setEulerZYX(-yaw + 3.14159265359 / 2, -pitch, roll);
    tf2::Matrix3x3 R_final = R_ENU_to_local.inverse();
    x = R_final[0][0] * east_ + R_final[0][1] * north_ + R_final[0][2] * up_;
    y = R_final[1][0] * east_ + R_final[1][1] * north_ + R_final[1][2] * up_;
    z = R_final[2][0] * east_ + R_final[2][1] * north_ + R_final[2][2] * up_;

    return;
}

void Gps_Local_Trans::OriginPointCallBack(const gps_to_local::fcc::ConstPtr &originmsg) {
    if (!flag) {
        origin_points = *originmsg;
        ROS_INFO("Originpoint_CallBack reveived message!!");
        std::cout << "origin" << origin_points.latitude << " " << origin_points.longitude << " "
                  << origin_points.altitude << std::endl;
        flag = true;
    }
    // return;
}

void Gps_Local_Trans::WgsToEnu(double lat, double lon, double h, double lat0, double lon0,
                               double h0, double &enu_x, double &enu_y, double &enu_z) {
    double a, b, f, e_sq, pi;
    pi = 3.14159265359;
    a = 6378137;
    b = 6356752.3142;
    f = (a - b) / a;
    e_sq = f * (2 - f);
    // 站点（非原点）
    double lamb, phi, s, N, sin_lambda, cos_lambda, sin_phi, cos_phi, x, y, z;
    lamb = lat / 180.0 * pi;
    phi = lon / 180.0 * pi;
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
    lamb0 = lat0 / 180.0 * pi;
    phi0 = lon0 / 180.0 * pi;
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

void Gps_Local_Trans::GpsToLocalCallBack(const gps_to_local::waypoints::ConstPtr &msg) {
    gps_to_local::waypoints current_point = *msg;
    //转为东北天
    double east, north, up;

    WgsToEnu(current_point.latitude, current_point.longitude, current_point.altitude,
             origin_points.latitude, origin_points.longitude, origin_points.altitude, east, north,
             up);
    // 输出东北天坐标
    std::cout << "East: " << east << std::endl;
    std::cout << "North: " << north << std::endl;
    std::cout << "Up: " << up << std::endl;
    //局部坐标
    double waypoints_local_x, waypoints_local_y, waypoints_local_z;
    EnuToXyz(north, east, up, origin_points.roll, origin_points.pitch,
             static_cast<double>(origin_points.orientation_uav), waypoints_local_x,
             waypoints_local_y, waypoints_local_z);

    //将局部坐标存储在wayponits数组中
    geometry_msgs::PoseStamped waypoints_to_use;
    waypoints_to_use.pose.position.x = waypoints_local_x;
    waypoints_to_use.pose.position.y = waypoints_local_y;
    waypoints_to_use.pose.position.z = waypoints_local_z;
    wayponits.push_back(waypoints_to_use);
    if (wayponits.size() == count + 1) {
        waypoints_data.data = wayponits;
        pub_ego.publish(waypoints_data);
        ros::spinOnce();
    }

    double id = static_cast<double>(current_point.id);
    std::ofstream outfile;
    outfile.open("/home/kao/GPS_to_Local/src/gps_to_local/src/output.txt", std::ios_base::app);
    if (outfile.is_open()) {
        outfile << "id: " << id << std::endl;
        outfile << "position:[" << waypoints_local_x << "," << waypoints_local_y << ","
                << waypoints_local_z << "]" << std::endl;
        outfile << std::endl;
    } else {
        ROS_ERROR("Output file is not open, unable to write coordinates.");
    }
    outfile.close();
}