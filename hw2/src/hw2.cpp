#include "ros/ros.h"

#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>

#include <math.h>

#define KP_x 1
#define KP_z 1.5
#define TOL  0.01

using namespace std;

turtlesim::Pose leader;
turtlesim::Pose follower1;
turtlesim::Pose follower2;

geometry_msgs::Point leader_goal;
geometry_msgs::Point follower1_goal;
geometry_msgs::Point follower2_goal;

geometry_msgs::Twist leader_twist;
geometry_msgs::Twist follower1_twist;
geometry_msgs::Twist follower2_twist;

ros::Publisher leader_pub;
ros::Publisher follower1_pub;
ros::Publisher follower2_pub;

struct XY {
    float x;
    float y;
} pos_err_I;

void leader_cb(const turtlesim::Pose::ConstPtr& msg)
{
	leader = *msg;
}

void follower1_cb(const turtlesim::Pose::ConstPtr& msg)
{
	follower1 = *msg;
}

void follower2_cb(const turtlesim::Pose::ConstPtr& msg)
{
	follower2 = *msg;
}

void leaderToWorld2D(geometry_msgs::Point &follower_goal, turtlesim::Pose &leader)
{
    float tmp_x = follower_goal.x;
    float tmp_y = follower_goal.y;
    float theta = -leader.theta;

    follower_goal.x = leader.x + tmp_x * cos(theta) + tmp_y * sin(theta);
    follower_goal.y = leader.y - tmp_x * sin(theta) + tmp_y * cos(theta);
}

void worldToBody2D(float &x, float &y, float theta)
{
    float tmp_x = x;
    float tmp_y = y;

    x = tmp_x * cos(theta) + tmp_y * sin(theta);
    y = -tmp_x * sin(theta) + tmp_y * cos(theta);
}

void positionControl(geometry_msgs::Point &goal, turtlesim::Pose &obj, geometry_msgs::Twist &vel_msg, string name)
{
    pos_err_I.x = goal.x - obj.x;
    pos_err_I.y = goal.y - obj.y;

    if (abs(pos_err_I.x) < obj.x * TOL && abs(pos_err_I.y) < obj.y * TOL) {
        vel_msg.linear.x = 0;
        vel_msg.angular.z = 0;
    	return;
    }

    worldToBody2D(pos_err_I.x, pos_err_I.y, obj.theta);

    float error_norm = sqrt(pow(pos_err_I.x, 2) + pow(pos_err_I.y, 2));
    float error_theta = atan2(pos_err_I.y, pos_err_I.x);

    if (error_norm > 2) error_norm = 2;

    vel_msg.linear.x = KP_x * error_norm;
    vel_msg.angular.z = KP_z * error_theta;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtle_formation");
    ros::NodeHandle n;

    ros::Subscriber leader_sub = n.subscribe<turtlesim::Pose>("/turtlesim/leader/pose", 1, leader_cb);
    ros::Subscriber follower1_sub = n.subscribe<turtlesim::Pose>("/turtlesim/follower1/pose", 1, follower1_cb);
    ros::Subscriber follower2_sub = n.subscribe<turtlesim::Pose>("/turtlesim/follower2/pose", 1, follower2_cb);

    leader_pub = n.advertise<geometry_msgs::Twist>("/turtlesim/leader/cmd_vel", 1);
    follower1_pub = n.advertise<geometry_msgs::Twist>("/turtlesim/follower1/cmd_vel", 1);
    follower2_pub = n.advertise<geometry_msgs::Twist>("/turtlesim/follower2/cmd_vel", 1);

    while (true) {
        ROS_INFO("Input (x, y). x > 0, y > 0");
        cout << "goal x: ";
        cin >> leader_goal.x;
        cout << "goal y: ";
        cin >> leader_goal.y;

	// terminate on EOF
	if (!leader_goal.x || !leader_goal.y) return 0;

        if (leader_goal.x <= 0 || leader_goal.x >= 11) leader_goal.x = 1;
        if (leader_goal.y <= 0 || leader_goal.y >= 11) leader_goal.y = 1;

        ros::Rate loop_rate(10);

        do {
            ROS_INFO("goal x: %f\ty: %f", leader_goal.x, leader_goal.y);
            ROS_INFO("pose x: %f\ty: %f", leader.x, leader.y);
            ROS_INFO("pose theta: %f", leader.theta);

            follower1_goal.x = -1;
            follower1_goal.y = -1;

            follower2_goal.x = -1;
            follower2_goal.y = 1;

            leaderToWorld2D(follower1_goal, leader);
            leaderToWorld2D(follower2_goal, leader);

            positionControl(leader_goal, leader, leader_twist, "leader");
            positionControl(follower1_goal, follower1, follower1_twist, "1");
            positionControl(follower2_goal, follower2, follower2_twist, "2");

            leader_pub.publish(leader_twist);
            follower1_pub.publish(follower1_twist);
            follower2_pub.publish(follower2_twist);

            ros::spinOnce();
            loop_rate.sleep();
        } while (ros::ok() && (leader_twist.linear.x != 0 || leader_twist.angular.z != 0
                           || follower1_twist.linear.x != 0 || follower1_twist.angular.z != 0
                           || follower2_twist.linear.x != 0 || follower2_twist.angular.z != 0));
    }

    return 0;
}
