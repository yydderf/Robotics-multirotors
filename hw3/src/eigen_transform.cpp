#include "ros/ros.h"
#include <Eigen/Dense>
#include <cmath>

#define _USE_MATH_DEFINES
#define SLERP_S 0.667

Eigen::Vector3d rad2deg(Eigen::Vector3d degrees)
{
    Eigen::Vector3d radians;
    for (int i = 0; i < degrees.size(); i++) {
        radians[i] = degrees[i] / M_PI * 180;
    }
    return radians;
}

Eigen::Vector3d deg2rad(Eigen::Vector3d radians)
{
    Eigen::Vector3d degrees;
    for (int i = 0; i < radians.size(); i++) {
        degrees[i] = radians[i] / 180 * M_PI;
    }
    return degrees;
}

Eigen::Quaterniond Euler2Quaternion(Eigen::Vector3d euler)
{
    Eigen::Quaterniond Q;
    Q = Eigen::AngleAxisd(euler.z(), Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(euler.y(), Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(euler.x(), Eigen::Vector3d::UnitX());
    return Q;
}

Eigen::Vector3d Quaternion2Euler(Eigen::Quaterniond Q)
{
    Eigen::Vector3d Euler(0, 0, 0);
    Euler.x() = atan2(2 * (Q.w() * Q.x() + Q.y() * Q.z()), 1 - 2 * (Q.x() * Q.x() + Q.y() * Q.y()));
    Euler.y() = asin(2 * (Q.w() * Q.y() - Q.z() * Q.x()));
    Euler.z() = atan2(2 * (Q.w() * Q.z() + Q.x() * Q.y()), 1 - 2 * (Q.y() * Q.y() + Q.z() * Q.z()));
    return Euler;
}

void rotByQuaternion(Eigen::Vector3d& v, Eigen::Quaterniond q, bool isFrame = false)
{
    double inv = isFrame ? -1 : 1;
    double s = q.w();
    Eigen::Vector3d u(inv * q.x(), inv * q.y(), inv * q.z());

    v = 2.0 * u.dot(v) * u +
        (s * s - u.dot(u)) * v + 
        2.0 * s * u.cross(v);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "eigen_transform");
    ros::NodeHandle nh;
    ros::Rate loop_rate(1);

    Eigen::Vector3d world_point(1, 0, 0);
    Eigen::Vector3d tmp_point = world_point;
    Eigen::Vector3d euler_angle_deg(0, 0, 90);

    int cnt = 0;
    std::cout << "Current point position: " << world_point.transpose() << "\n\n";

    while (ros::ok()) {
        Eigen::Vector3d euler_angle_rad = deg2rad(euler_angle_deg);
        Eigen::Quaterniond Q_Total = Euler2Quaternion(euler_angle_rad);
        Eigen::Quaterniond Q_tmp = Eigen::Quaterniond::Identity().slerp(SLERP_S, Q_Total);

        Eigen::Vector3d euler_tmp_rad = Quaternion2Euler(Q_tmp);
        Eigen::Vector3d euler_tmp_deg = rad2deg(euler_tmp_rad);
        std::cout << "Apply rotation roll(X): " << euler_tmp_deg.x()
                  << ", pitch(Y): " << euler_tmp_deg.y()
                  << ", yaw(Z): " << euler_tmp_deg.z() << std::endl;

        rotByQuaternion(world_point, Q_tmp, false);

        cnt++;
        std::cout << "Current point position: " << world_point.transpose() << "\n\n";

        Eigen::Vector3d point_diff = world_point - tmp_point;
        if (point_diff.norm() < 0.1) {
            std::cout << "Current point rotate " << cnt << " times to origin position!" << std::endl;
            break;
        }
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
