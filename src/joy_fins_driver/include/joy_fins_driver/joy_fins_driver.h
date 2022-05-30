#ifndef JOY_FINS_DRIVER_H
#define JOY_FINS_DRIVER_H

namespace ros { class NodeHandle; }

namespace fins_driver_joy
{

/**
 * Class implementing a basic Joy -> Twist translation.
 */
class FinsDriverJoy
{
public:
  FinsDriverJoy(ros::NodeHandle* nh, ros::NodeHandle* nh_param);

private:
  struct Impl;
  Impl* pimpl_;
};

}  // namespace fins_driver_joy

#endif  // JOY_FINS_DRIVER_H