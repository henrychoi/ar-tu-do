#pragma once

#include "drive_mode.h"
#include <ros/ros.h>

#include <algorithm>

#include <drive_msgs/drive_param.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>

class CarController
{
public:
    CarController(ros::NodeHandle nh, ros::NodeHandle private_nh);

private:
//ros::NodeHandle m_node_handle;

    ros::Subscriber m_drive_parameters_subscriber;
    ros::Subscriber m_drive_mode_subscriber;
    ros::Subscriber m_emergency_stop_subscriber;

    ros::Publisher m_speed_publisher;
    ros::Publisher m_angle_publisher;
    ros::Publisher m_brake_publisher;

//MARK: parameters read from YAML?
    float speed_to_erpm_gain_;
    float steering_to_servo_gain_[2], steering_to_servo_offset_[2];
    bool m_drive_param_lock = true;
    bool m_emergency_stop_lock = true;
    DriveMode m_current_drive_mode;

    /**
     * @brief deals with incomming drive param messages
     */
    void driveParametersCallback(const drive_msgs::drive_param::ConstPtr& parameters);

    /**
     * @brief sets the current drive mode
     */
    void driveModeCallback(const std_msgs::Int32::ConstPtr& drive_mode_message);

    /**
     * @brief callback for the topic that enables / disables the motor
     */
    void emergencyStopCallback(const std_msgs::Bool::ConstPtr& drive_mode_message);

    /**
     * @brief takes a speed and angle, converts and forwards them to gazebo/focbox
     */
    void publishDriveParameters(double raw_speed, float front, float back);

    /**
     * @brief takes speed and publishes it to gazebo/focbox
     */
    void publishSpeed(double speed);

    /**
     * @brief takes servo angles and publishes it to gazebo/focbox
     */
    void publishAngle(float front, float back);

    /**
     * @brief publishes a brake message that stops the car
     */
    void stop();
};
