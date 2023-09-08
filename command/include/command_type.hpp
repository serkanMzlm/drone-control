#ifndef __COMMAND_TYPE_HPP__
#define __COMMAND_TYPE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>

using namespace px4_msgs::msg;
using joyMsg = sensor_msgs::msg::Joy;
using sensorCombinedMsg = px4_msgs::msg::SensorCombined;
using localPosMsg = px4_msgs::msg::VehicleLocalPosition;
using odomMsg = px4_msgs::msg::VehicleOdometry;

typedef struct{
    rclcpp::Publisher<OffboardControlMode>::SharedPtr mode;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr setpoint;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command;
}Pub_t;

typedef struct{
    rclcpp::Subscription<joyMsg>::SharedPtr joy;
    rclcpp::Subscription<sensorCombinedMsg>::SharedPtr sensor_combine;
    rclcpp::Subscription<localPosMsg>::SharedPtr local_pos;
    rclcpp::Subscription<odomMsg>::SharedPtr odom;
}Sub_t;

typedef union{
    struct{
        float x;
        float y;
        float z;
        float yaw;
    };
    float state[4];
} State_t; 

typedef struct{
    bool is_arm;
    bool is_joy;
    bool is_press;
    bool is_takeoff;
    bool is_land;
} Flag_t;

typedef enum {STATE, ALL_DATA} Data_t;

#endif