#include <exocet_msgs/ExocetCMD.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
#include "joy_fins_driver/joy_fins_driver.h"

#include <string>
#include <map>
#include <string>
#include <algorithm>
#include <array>
#include <iterator>

namespace fins_driver_joy
{

/**
 * Internal members of class. This is the pimpl idiom, and allows more flexibility in adding
 * parameters later without breaking ABI compatibility, for robots which link FinsDriverJoy
 * directly into base nodes.
 */
struct FinsDriverJoy::Impl
{
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void sendCmdExocet(const sensor_msgs::Joy::ConstPtr& joy_msg, const std::string& which_map);

  ros::Subscriber joy_sub;
  ros::Publisher exocet_cmd_pub;
  
  std::array<ros::Publisher, 4> wings_fold_pub;
  std::array<ros::Publisher, 4> fins_fold_pub;
  std::array<ros::Publisher, 4> fins_rev_pub;

  // ros::wings_fold_publisher;

  int enable_button;
  int enable_turbo_button;

  std::map<std::string, int> axis_linear_map;
  std::map< std::string, std::map<std::string, double> > scale_linear_map;

  std::map<std::string, int> axis_angular_map;
  std::map< std::string, std::map<std::string, double> > scale_angular_map;

  bool sent_disable_msg;
};

/**
 * Constructs FinsDriverJoy.
 * \param nh NodeHandle to use for setting up the publisher and subscriber.
 * \param nh_param NodeHandle to use for searching for configuration parameters.
 */
FinsDriverJoy::FinsDriverJoy(ros::NodeHandle* nh, ros::NodeHandle* nh_param)
{
  pimpl_ = new Impl;

  pimpl_->exocet_cmd_pub = nh->advertise<exocet_msgs::ExocetCMD>("ExocetCMD", 1, true);
  pimpl_->joy_sub = nh->subscribe<sensor_msgs::Joy>("joy", 1, &FinsDriverJoy::Impl::joyCallback, pimpl_);

  for (size_t i = 0; i < 4; i++)
  {
    std::string wings_tpc = "/exocetMM40/wings_fold" + std::to_string(i+1) + "_position_controller/command";
    pimpl_->wings_fold_pub[i] = nh->advertise<std_msgs::Float64>(wings_tpc, 1, true);
    std::string fins_tpc = "/exocetMM40/fins_fold" + std::to_string(i+1) + "_position_controller/command";
    pimpl_->fins_fold_pub[i] = nh->advertise<std_msgs::Float64>(fins_tpc, 1, true);
    std::string rev_tpc = "/exocetMM40/fins_rev" + std::to_string(i+1) + "_position_controller/command";
    pimpl_->fins_rev_pub[i] = nh->advertise<std_msgs::Float64>(rev_tpc, 1, true);
  }

  nh_param->param<int>("enable_button", pimpl_->enable_button, 0);
  nh_param->param<int>("enable_turbo_button", pimpl_->enable_turbo_button, -1);

  if (nh_param->getParam("axis_linear", pimpl_->axis_linear_map))
  {
    nh_param->getParam("scale_linear", pimpl_->scale_linear_map["normal"]);
    nh_param->getParam("scale_linear_turbo", pimpl_->scale_linear_map["turbo"]);
  }
  else
  {
    nh_param->param<int>("axis_linear", pimpl_->axis_linear_map["x"], 1);
    nh_param->param<double>("scale_linear", pimpl_->scale_linear_map["normal"]["x"], 0.5);
    nh_param->param<double>("scale_linear_turbo", pimpl_->scale_linear_map["turbo"]["x"], 1.0);
  }

  if (nh_param->getParam("axis_angular", pimpl_->axis_angular_map))
  {
    nh_param->getParam("scale_angular", pimpl_->scale_angular_map["normal"]);
    nh_param->getParam("scale_angular_turbo", pimpl_->scale_angular_map["turbo"]);
  }
  else
  {
    nh_param->param<int>("axis_angular", pimpl_->axis_angular_map["yaw"], 0);
    nh_param->param<double>("scale_angular", pimpl_->scale_angular_map["normal"]["yaw"], 0.5);
    nh_param->param<double>("scale_angular_turbo",
        pimpl_->scale_angular_map["turbo"]["yaw"], pimpl_->scale_angular_map["normal"]["yaw"]);
  }

  ROS_INFO_NAMED("FinsDriverJoy", "Teleop enable button %i.", pimpl_->enable_button);
  ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0, "FinsDriverJoy",
      "Turbo on button %i.", pimpl_->enable_turbo_button);

  for (std::map<std::string, int>::iterator it = pimpl_->axis_linear_map.begin();
      it != pimpl_->axis_linear_map.end(); ++it)
  {
    ROS_INFO_NAMED("FinsDriverJoy", "Linear axis %s on %i at scale %f.",
    it->first.c_str(), it->second, pimpl_->scale_linear_map["normal"][it->first]);
    ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0, "FinsDriverJoy",
        "Turbo for linear axis %s is scale %f.", it->first.c_str(), pimpl_->scale_linear_map["turbo"][it->first]);
  }

  for (std::map<std::string, int>::iterator it = pimpl_->axis_angular_map.begin();
      it != pimpl_->axis_angular_map.end(); ++it)
  {
    ROS_INFO_NAMED("FinsDriverJoy", "Angular axis %s on %i at scale %f.",
    it->first.c_str(), it->second, pimpl_->scale_angular_map["normal"][it->first]);
    ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0, "FinsDriverJoy",
        "Turbo for angular axis %s is scale %f.", it->first.c_str(), pimpl_->scale_angular_map["turbo"][it->first]);
  }

  pimpl_->sent_disable_msg = false;
}

double getVal(const sensor_msgs::Joy::ConstPtr& joy_msg, const std::map<std::string, int>& axis_map,
              const std::map<std::string, double>& scale_map, const std::string& fieldname)
{
  if (axis_map.find(fieldname) == axis_map.end() ||
      scale_map.find(fieldname) == scale_map.end() ||
      joy_msg->axes.size() <= axis_map.at(fieldname))
  {
    return 0.0;
  }

  return joy_msg->axes[axis_map.at(fieldname)] * scale_map.at(fieldname);
}

void FinsDriverJoy::Impl::sendCmdExocet(const sensor_msgs::Joy::ConstPtr& joy_msg,
                                         const std::string& which_map)
{
  // Initializes with zeros by default.
  // geometry_msgs::Twist cmd_vel_msg;

  // cmd_vel_msg.linear.x = getVal(joy_msg, axis_linear_map, scale_linear_map[which_map], "x");
  // cmd_vel_msg.linear.y = getVal(joy_msg, axis_linear_map, scale_linear_map[which_map], "y");
  // cmd_vel_msg.linear.z = getVal(joy_msg, axis_linear_map, scale_linear_map[which_map], "z");
  // cmd_vel_msg.angular.z = getVal(joy_msg, axis_angular_map, scale_angular_map[which_map], "yaw");
  // cmd_vel_msg.angular.y = getVal(joy_msg, axis_angular_map, scale_angular_map[which_map], "pitch");
  // cmd_vel_msg.angular.x = getVal(joy_msg, axis_angular_map, scale_angular_map[which_map], "roll");

  // exocet_cmd_pub.publish(cmd_vel_msg);
  sent_disable_msg = false;
}

void FinsDriverJoy::Impl::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
  // ROS_INFO_STREAM("BUTTON 0 ->" << joy_msg->buttons[0]);
  // ROS_INFO_STREAM("BUTTON 1 ->" << joy_msg->buttons[1]);
  // ROS_INFO_STREAM("BUTTON 2 ->" << joy_msg->buttons[2]);
  // ROS_INFO_STREAM("BUTTON 3 ->" << joy_msg->buttons[3]);
  // ROS_INFO_STREAM("BUTTON 4 ->" << joy_msg->buttons[4]);
  // ROS_INFO_STREAM("BUTTON 5 ->" << joy_msg->buttons[5]);
  // ROS_INFO_STREAM("BUTTON 6 ->" << joy_msg->buttons[6]);
  // ROS_INFO_STREAM("BUTTON 7 ->" << joy_msg->buttons[7]);
  // ROS_INFO_STREAM("BUTTON 8 ->" << joy_msg->buttons[8]);
  // ROS_INFO_STREAM("BUTTON 9 ->" << joy_msg->buttons[9]);
  // ROS_INFO_STREAM("BUTTON 10 ->" << joy_msg->buttons[10]);
  // ROS_INFO_STREAM("BUTTON 11 ->" << joy_msg->buttons[11]);
  
  // ROS_INFO_STREAM("AXIS 0 " << joy_msg->axes[0]);
  // ROS_INFO_STREAM("AXIS 1 " << joy_msg->axes[1]);
  // ROS_INFO_STREAM("AXIS 2 " << joy_msg->axes[2]);
  // ROS_INFO_STREAM("AXIS 3 " << joy_msg->axes[3]);
  // ROS_INFO_STREAM("AXIS 4 " << joy_msg->axes[4]);
  // ROS_INFO_STREAM("AXIS 5 " << joy_msg->axes[5]);
  // ROS_INFO_STREAM("AXIS 6 " << joy_msg->axes[6]);

  exocet_msgs::ExocetCMD cmd;
 
  geometry_msgs::Vector3 force;
  force.x = 0.0;
  force.y = 0.0;
  force.z = 75000*joy_msg->axes[1];
  geometry_msgs::Vector3 torque;
  if (force.z < 0.0)
  {
    force.z = 0.0;
  }
  
  cmd.detach_booster = bool(joy_msg->buttons[5]);
  cmd.propulsion.force = force;
  
  exocet_cmd_pub.publish(cmd);

  std_msgs::Float64 rev[4];

  float dir = bool(joy_msg->buttons[4]) ? 1.0 : -1.0;
  // float joint_limit = 0.785398;
  float joint_limit = 0.349066;
  rev[0].data = joint_limit*joy_msg->axes[3];
  rev[2].data = dir*joint_limit*joy_msg->axes[3];
  rev[1].data = joint_limit*joy_msg->axes[4];
  rev[3].data = dir*joint_limit*joy_msg->axes[4];

  for (size_t i = 0; i < 4; i++)
  {
    std_msgs::Float64 value;
    value.data = 0.0;
    fins_rev_pub[i].publish(rev[i]);
  }

  if (bool(joy_msg->buttons[8]))
  {
    static bool prev = 0;
    prev = !prev;
    std_msgs::Float64 val;
    val.data = prev ? 2.0 : 0.0;
    for (size_t i = 0; i < 4; i++)
    {
      wings_fold_pub[i].publish(val);
      fins_fold_pub[i].publish(val);
    }
  }

  // if (enable_turbo_button >= 0 &&
  //     joy_msg->buttons.size() > enable_turbo_button &&
  //     joy_msg->buttons[enable_turbo_button])
  // {
  //   sendCmdExocet(joy_msg, "turbo");
  // }
  // else if (joy_msg->buttons.size() > enable_button &&
  //          joy_msg->buttons[enable_button])
  // {
  //   sendCmdExocet(joy_msg, "normal");
  // }
  // else
  // {
  //   // When enable button is released, immediately send a single no-motion command
  //   // in order to stop the robot.
  //   if (!sent_disable_msg)
  //   {
  //     // Initializes with zeros by default.
  //     geometry_msgs::Twist cmd_vel_msg;
  //     exocet_cmd_pub.publish(cmd_vel_msg);
  //     sent_disable_msg = true;
  //   }
  // }
}

}  // namespace joy_fins_driver