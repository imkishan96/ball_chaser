#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "ball_chaser/DriveToTarget.h"

ros::ServiceClient client;

void move_robot(const std::string& loc)
{
  ball_chaser::DriveToTarget motor_command;
  if (loc.empty())
  {
    ROS_INFO("Ball not found: Stopping the Robot.");
    motor_command.request.linear_x = 0;
    motor_command.request.angular_z = 0;
  }
  else
  {
    if (loc == "left")
    {
      motor_command.request.linear_x = 0.1;
      motor_command.request.angular_z = 1.2;
      ROS_INFO("Ball found: turning left towards the Ball.");
    }
    else if (loc == "right")
    {
      motor_command.request.linear_x = 0.1;
      motor_command.request.angular_z = -1.2;
      ROS_INFO("Ball found: turning right towards the Ball.");
    }
    else
    {
      motor_command.request.linear_x = 0.5;
      motor_command.request.angular_z = 0.0;
      ROS_INFO("Ball found: moving straight towards the Ball.");
    }
  }
  if (!client.call(motor_command))
    ROS_ERROR("Failed to call service Command_robot");
}

void look_for_ball(const sensor_msgs::Image img)
{
  bool white_pixel = false;
  size_t img_wid = img.width;
  size_t img_step = img.step;
  size_t img_hig = img.height;
  int rgb = img_step / img_wid;
  std::string ball_loc;

  for (size_t i = 0; i < img_hig * img_step; i = i + rgb)
  {
    if (img.data[i] == 255)
    {
      if (img.data[i + 1] == 255 && img.data[i + 2] == 255)
      {
        white_pixel = true;
        ball_loc =
            ((i % img_step) < (img_step / 3)) ? "left" : ((i % img_step) > (img_step * 2 / 3)) ? "right" : "center";
        break;
      }
    }
  }
  move_robot(ball_loc);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "process_image");
  ros::NodeHandle n;
  ROS_INFO("**************Process_image Node started**********\n");
  client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");
  ros::Subscriber sub = n.subscribe("/camera/rgb/image_raw", 2, look_for_ball);
  ros::spin();
  return 0;
}