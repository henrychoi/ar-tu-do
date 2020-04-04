// -*- mode:c++; fill-column: 100; -*-

#ifndef VESC_DRIVER_VESC_DRIVER_H_
#define VESC_DRIVER_VESC_DRIVER_H_

#include <string>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32MultiArray.h>
#include <boost/optional.hpp>

#include "vesc_driver/vesc_interface.h"
#include "vesc_driver/vesc_packet.h"

namespace vesc_driver
{

class VescDriver
{
public:

  VescDriver(ros::NodeHandle nh,
             ros::NodeHandle private_nh);

private:
  // interface to the VESC
  VescInterface vesc_;
  void vescPacketCallback(const boost::shared_ptr<VescPacket const>& packet);
  void vescErrorCallback(const std::string& error);

  // limits on VESC commands
  struct CommandLimit
  {
    CommandLimit(const ros::NodeHandle& nh, const std::string& str,
                 const boost::optional<double>& min_lower = boost::optional<double>(),
                 const boost::optional<double>& max_upper = boost::optional<double>());
    double clip(double value);
    std::string name;
    boost::optional<double> lower;
    boost::optional<double> upper;
  };
  CommandLimit duty_cycle_limit_;
  CommandLimit current_limit_;
  CommandLimit brake_limit_;
  CommandLimit speed_limit_;
  CommandLimit position_limit_;
  CommandLimit servo_limit_;

  // ROS services
  ros::Publisher state_pub_;
  ros::Subscriber duty_cycle_sub_;
  ros::Subscriber current_sub_;
  ros::Subscriber brake_sub_;
  ros::Subscriber speed_sub_;
  ros::Subscriber position_sub_;
  ros::Subscriber servo_sub_;
  ros::Timer timer_;

  // driver modes (possible states)
  typedef enum {
    MODE_INITIALIZING,
    MODE_OPERATING
  } driver_mode_t;

  // other variables
  driver_mode_t driver_mode_;           ///< driver state machine mode (state)
  int fw_version_major_;                ///< firmware major version reported by vesc
  int fw_version_minor_;                ///< firmware minor version reported by vesc

  float servo_duty_[2] = {0.5f, 0.5f};
  float speed_goal_ = 0 // [eRPM] target set from high level
    // (e.g. manual, navigation); may change in a step manner
    , speed_r_ = 0 // [eRPM] low level target, for short time horizon smooth speed
    , brake_ // [eRPM / timer_period]; > 0
    , accel_ // [eRPM / timer_period]; > 0
    , decel_ // [eRPM / timer_period]; > 0
    ;
  inline void adjust_speed_r(float adj) {
    if (fabs(speed_goal_ - speed_r_) < fabs(adj)) {
      speed_r_ = speed_goal_;
    } else {
      speed_r_ += adj;
    }
  }
  // ROS callbacks
  void timerCallback(const ros::TimerEvent& event);
  void dutyCycleCallback(const std_msgs::Float64::ConstPtr& duty_cycle);
  void currentCallback(const std_msgs::Float64::ConstPtr& current);
  void brakeCallback(const std_msgs::Float64::ConstPtr& brake);
  void speedCallback(const std_msgs::Float64::ConstPtr& speed);
  void positionCallback(const std_msgs::Float64::ConstPtr& position);
  void servoCallback(const std_msgs::Float32MultiArray::ConstPtr& servo);
};

} // namespace vesc_driver

#endif // VESC_DRIVER_VESC_DRIVER_H_
