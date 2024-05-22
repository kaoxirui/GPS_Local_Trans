#include "gps_to_local/local_to_gps.hpp"

Local_To_Gps_Trans::Local_To_Gps_Trans() : nh("~"), flag(false) {
    sub_target = nh.subscribe<geometry_msgs::PoseStamped>(
        "/pose_tf", 10, &Local_To_Gps_Trans::LocalToGpsCallBack, this);
    sub_ori = nh.subscribe<gps_to_local::fcc>("/fcc_messages", 10,
                                              &Local_To_Gps_Trans::OriginCallBack, this);
    pub_gps = nh.advertise<gps_to_local::fcc>("/target_gps", 10);
}

void Local_To_Gps_Trans::OriginCallBack(
    const gps_to_local::fcc::ConstPtr &originmsg) { //得到起始点局部坐标
    if (!flag) {
        origin_position = *originmsg;
        lat0 = origin_position.latitude;  //纬度
        lon0 = origin_position.longitude; //经
        alt0 = origin_position.altitude;
        roll = origin_position.roll;
        pitch = origin_position.pitch;
        yaw = origin_position.orientation_uav;
    }
    flag = true;
    return;
}

void Local_To_Gps_Trans::EnuToWgs(double &lat, double &lon, double &h, double lat0, double lon0,
                                  double h0, double xEast, double yNorth, double zUp) {
    double a, b, f, e_sq, pi;
    pi = 3.14159265359;
    a = 6378137;
    b = 6356752.3142;
    f = (a - b) / a;
    e_sq = f * (2 - f);
    pi = 3.14159265359;
    double lamb, phi, s, N, sin_lambda, cos_lambda, sin_phi, cos_phi;
    lamb = pi / 180 * (lat0);
    phi = pi / 180 * (lon0);
    s = sin(lamb);
    N = a / sqrt(1 - e_sq * s * s);

    sin_lambda = sin(lamb);
    cos_lambda = cos(lamb);
    sin_phi = sin(phi);
    cos_phi = cos(phi);

    double x0, y0, z0, t, zd, xd, yd, x, y, z;
    x0 = (h0 + N) * cos_lambda * cos_phi;
    y0 = (h0 + N) * cos_lambda * sin_phi;
    z0 = (h0 + (1 - e_sq) * N) * sin_lambda;

    t = cos_lambda * zUp - sin_lambda * yNorth;

    zd = sin_lambda * zUp + cos_lambda * yNorth;
    xd = cos_phi * t - sin_phi * xEast;
    yd = sin_phi * t + cos_phi * xEast;

    x = xd + x0;
    y = yd + y0;
    z = zd + z0;

    double x2, y2, z2;
    x2 = x * x;
    y2 = y * y;
    z2 = z * z;

    double e, b2, e2, ep, r, r2, E2, F, G, c, s2, P, Q, ro, tmp, U, V, zo, height, temp;
    e = sqrt(1 - (b / a) * (b / a));
    b2 = b * b;
    e2 = e * e;
    ep = e * (a / b);
    r = sqrt(x2 + y2);
    r2 = r * r;
    E2 = a * a - b * b;
    F = 54 * b2 * z2;
    G = r2 + (1 - e2) * z2 - e2 * E2;
    c = (e2 * e2 * F * r2) / (G * G * G);
    s2 = pow((1 + c + sqrt(c * c + 2 * c)), (1 / 3));
    P = F / (3 * (s2 + 1 / s2 + 1) * (s2 + 1 / s2 + 1) * G * G);
    Q = sqrt(1 + 2 * e2 * e2 * P);
    ro = -(P * e2 * r) / (1 + Q)
         + sqrt((a * a / 2) * (1 + 1 / Q) - (P * (1 - e2) * z2) / (Q * (1 + Q)) - P * r2 / 2);
    tmp = (r - e2 * ro) * (r - e2 * ro);
    U = sqrt(tmp + z2);
    V = sqrt(tmp + (1 - e2) * z2);
    zo = (b2 * z) / (a * V);

    height = U * (1 - b2 / (a * V));

    lat = atan((z + ep * ep * zo) / r);

    temp = atan(y / x);

    double longitude;
    if (x >= 0)
        longitude = temp;
    else {
        if ((x < 0) && (y >= 0))
            longitude = pi + temp;
        else
            longitude = temp - pi;
    }

    lat = lat / (pi / 180);
    lon = longitude / (pi / 180);
    h = height;
}

ros::Time last_publish_time = ros::Time::now();
const double publish_rate_ = 20.0; // 发布频率为 20 Hz

void Local_To_Gps_Trans::LocalToGpsCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    if ((ros::Time::now() - last_publish_time).toSec() >= (1.0 / publish_rate_)) {
        geometry_msgs::PoseStamped local_msg;
        //求姿态
        //四元数
        double oritetion_x = local_msg.pose.orientation.x;
        double oritetion_y = local_msg.pose.orientation.y;
        double oritetion_z = local_msg.pose.orientation.z;
        double oritetion_w = local_msg.pose.orientation.w;

        // tf2将四元数转换为欧拉角
        // 使用 tf2 库将四元数转换为 roll、pitch、yaw
        double roll_uav, pitch_uav, yaw_uav;
        tf2::Quaternion q(oritetion_x, oritetion_y, oritetion_z, oritetion_w);
        tf2::Matrix3x3 m(q);
        m.getRPY(roll_uav, pitch_uav, yaw_uav);
        // 将角度转换为度
        // roll_uav = roll_uav * 180.0 / M_PI;//多余了，本身就是弧度
        // pitch_uav = pitch_uav * 180.0 / M_PI;
        // yaw_uav = yaw_uav * 180.0 / M_PI;

        local_msg = *msg;
        tf2::Matrix3x3 R_local_to_ENU;

        yaw = yaw * 180 / M_PI;
        pitch = pitch * 180 / M_PI;
        roll = roll * 180 / M_PI;

        R_local_to_ENU.setEulerZYX(-yaw + 3.14159265359 / 2, -pitch, -roll);
        tf2::Matrix3x3 R_final = R_local_to_ENU;
        double ans_x, ans_y, ans_z;
        ans_x = R_final[0][0] * local_msg.pose.position.x
                + R_final[0][1] * local_msg.pose.position.y
                + R_final[0][2] * local_msg.pose.position.z;
        ans_y = R_final[1][0] * local_msg.pose.position.x
                + R_final[1][1] * local_msg.pose.position.y
                + R_final[1][2] * local_msg.pose.position.z;
        ans_z = R_final[2][0] * local_msg.pose.position.x
                + R_final[2][1] * local_msg.pose.position.y
                + R_final[2][2] * local_msg.pose.position.z;
        std::cout << "East:" << ans_x << "   North:" << ans_y << "   Up:" << ans_z << std::endl;
        double lat, lon, alt;
        // ENU_to_GPS
        EnuToWgs(lat, lon, alt, lat0, lon0, alt0, ans_x, ans_y, ans_z);
        gps_to_local::fcc gps_message;
        // 转换为经纬高坐标
        gps_message.header_ros = local_msg.header;
        gps_message.latitude = lat;
        gps_message.longitude = lon;
        gps_message.altitude = alt;
        gps_message.roll = roll_uav;
        gps_message.pitch = pitch_uav;
        gps_message.orientation_uav = yaw_uav;
        std::cout << gps_message.latitude << " " << gps_message.longitude << " "
                  << gps_message.altitude << " " << std::endl;

        pub_gps.publish(gps_message);
        last_publish_time = ros::Time::now();
    }
}
