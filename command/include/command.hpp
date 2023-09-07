#ifndef __COMMAND_HPP__
#define __COMMAND_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include "command_type.hpp"

#include <chrono>
#include <cmath>

using namespace px4_msgs::msg;
using joyMsg = sensor_msgs::msg::Joy;

class Command : public rclcpp::Node{
private:
    rclcpp::Subscription<joyMsg>::SharedPtr joySub_;
    rclcpp::Publisher<OffboardControlMode>::SharedPtr px4_mode_pub_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr setpoint_pub_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_pub_;
private:
    Flag_t flags;
    float coef = 0.1f;
    int prev_button = 0;
    Satate_t set_point {0.0, 0.0, 0.0, 0.0};
public:
    Command();
    void joyCallback(joyMsg joy_msg);
    void resetData(Data_t select);
    void initTopic();
    void initParam();
public:
    void controlMode();
	void trajectorySetpoint(Satate_t new_data);
	void vehicleCommand(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    void armDisarm();
};

#endif