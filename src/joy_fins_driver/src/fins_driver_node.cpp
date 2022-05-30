#include "ros/ros.h"
#include "joy_fins_driver/joy_fins_driver.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "joy_fins_driver_node");

  ros::NodeHandle nh(""), nh_param("~");
  fins_driver_joy::FinsDriverJoy fins_driver(&nh, &nh_param);

  ros::spin();
}