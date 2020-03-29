#include "car_controller.h"
#include "car_config.h"
#include <std_msgs/Float32MultiArray.h>
#include <boost/algorithm/clamp.hpp>

template <typename T>
inline bool getRequiredParam(const ros::NodeHandle& nh, std::string name, T& value)
{
  if (nh.getParam(name, value))
    return true;

  ROS_FATAL("Parameter %s is required.", name.c_str());
  return false;
}

CarController::CarController(ros::NodeHandle nh, ros::NodeHandle private_nh)
{
    (void)private_nh;
    if (!getRequiredParam(nh, "speed_to_erpm_gain", speed_to_erpm_gain_))
      return;
    if (!getRequiredParam(nh, "steering_angle_to_servo0_gain", steering_to_servo_gain_[0]))
      return;
    if (!getRequiredParam(nh, "steering_angle_to_servo0_offset", steering_to_servo_offset_[0]))
      return;
    if (!getRequiredParam(nh, "steering_angle_to_servo1_gain", steering_to_servo_gain_[1]))
      return;
    if (!getRequiredParam(nh, "steering_angle_to_servo1_offset", steering_to_servo_offset_[1]))
      return;

    m_drive_parameters_subscriber =
      nh.subscribe<drive_msgs::drive_param>("/commands/drive_param", 1,
					    &CarController::driveParametersCallback, this);
    m_drive_mode_subscriber = nh.subscribe<std_msgs::Int32>("/commands/drive_mode", 1,
							    &CarController::driveModeCallback, this);
    m_emergency_stop_subscriber =
        nh.subscribe<std_msgs::Bool>("/commands/emergency_stop", 1,
				     &CarController::emergencyStopCallback, this);

    m_speed_publisher = nh.advertise<std_msgs::Float64>("/commands/motor/speed", 1);
    m_angle_publisher = nh.advertise<std_msgs::Float32MultiArray>("/commands/servo/position", 1);
    m_brake_publisher = nh.advertise<std_msgs::Float64>("commands/motor/brake", 1);
}

// convert drive_param to the speed, and front/back servo angles
void CarController::driveParametersCallback(const drive_msgs::drive_param::ConstPtr& parameters)
{
    if (m_drive_param_lock) {
	publishDriveParameters(0, 0, 0);
    } else {
        // TODO: solve the 4-wheel steering problem
        const float angle = parameters->angle;
        publishDriveParameters(m_emergency_stop_lock ? 0 : parameters->velocity,
			     angle, parameters->crab_steer ? angle : -angle);
    }
}

void CarController::publishDriveParameters(double relative_speed, float front, float back)
{
    double speed = relative_speed * speed_to_erpm_gain_;
    const float front_servo = steering_to_servo_gain_[0] * front + steering_to_servo_offset_[0];
    const float back_servo = steering_to_servo_gain_[1] * back + steering_to_servo_offset_[1];

    publishSpeed(speed);
    publishAngle(front_servo, back_servo);

    ROS_DEBUG("running: | speed: %.1f | front: %.1f | back: %.1f",
	     speed, front_servo, back_servo);
}

void CarController::publishSpeed(double speed)
{
    std_msgs::Float64 speed_message;
    speed_message.data = speed;
    m_speed_publisher.publish(speed_message);
}

void CarController::publishAngle(float front, float back)
{
    std_msgs::Float32MultiArray servos;
    servos.data.push_back(front);
    servos.data.push_back(back);
    m_angle_publisher.publish(servos);
}

void CarController::driveModeCallback(const std_msgs::Int32::ConstPtr& drive_mode_message)
{
    m_current_drive_mode = (DriveMode)drive_mode_message->data;
    m_drive_param_lock = m_current_drive_mode == DriveMode::LOCKED;
    if (m_drive_param_lock)
        stop();
}

void CarController::emergencyStopCallback(const std_msgs::Bool::ConstPtr& emergency_stop_message)
{
    bool enable_emergency_stop = emergency_stop_message->data && m_current_drive_mode != DriveMode::MANUAL;
    m_emergency_stop_lock = enable_emergency_stop;
    if (m_emergency_stop_lock)
        stop();
}

void CarController::stop()
{
    publishSpeed(0);

    std_msgs::Float64 brake_message;
    brake_message.data = 0;
    m_brake_publisher.publish(brake_message);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "car_controller");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    CarController carController(nh, private_nh);
    ros::spin();
    return EXIT_SUCCESS;
}
