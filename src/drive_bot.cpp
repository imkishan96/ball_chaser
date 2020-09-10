#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include "ball_chaser/DriveToTarget.h"

ros::Publisher motor_command_publisher;
geometry_msgs::Twist motor_command;

bool set_vel(ball_chaser::DriveToTarget::Request &req, ball_chaser::DriveToTarget::Response &res)
{
  motor_command.linear.x = req.linear_x;
  motor_command.angular.z = req.angular_z;
  motor_command_publisher.publish(motor_command);
  res.msg_feedback =
      "linear_x set to: " + std::to_string(req.linear_x) + " & angular z set to: " + std::to_string(req.angular_z);
  // ROS_INFO_STREAM(res.msg_feedback);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "drive_bot");
  ros::NodeHandle n;
  motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  ros::ServiceServer _server = n.advertiseService("/ball_chaser/command_robot", set_vel);

  ros::spin();
  return 0;
}