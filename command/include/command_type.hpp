#ifndef __COMMAND_TYPE_HPP__
#define __COMMAND_TYPE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>

using namespace px4_msgs::msg;
using joyMsg = sensor_msgs::msg::Joy;

typedef struct{
    rclcpp::Publisher<OffboardControlMode>::SharedPtr mode;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr setpoint;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command;
}Pub_t;

typedef struct{
    rclcpp::Subscription<joyMsg>::SharedPtr joy;
}Sub_t;

typedef union{
    struct{
        float x;
        float y;
        float z;
        float yaw;
    };
    float state[4];
} Satate_t; 

typedef struct{
    bool is_arm;
    bool is_joy;
    bool is_press;
} Flag_t;

typedef enum {STATE, ALL_DATA} Data_t;

#endif