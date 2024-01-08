#ifndef __SENSOR_READER_HPP__ 
#define __SENSOR_READER_HPP__

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/sensor_combined.hpp"
#include "px4_msgs/msg/sensor_gps.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/vehicle_control_mode.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"

#include "sensor_reader_type.hpp"
#include "geometry_msgs/msg/twist.hpp"

using SensorCombinedMsg = px4_msgs::msg::SensorCombined;
using SensorGpsMsg = px4_msgs::msg::SensorGps;
using VehicleStatusMsg = px4_msgs::msg::VehicleStatus;
using localPosMsg = px4_msgs::msg::VehicleLocalPosition;
using twistMsg = geometry_msgs::msg::Twist;

typedef struct {
    rclcpp::Subscription<SensorCombinedMsg>::SharedPtr imu;
    rclcpp::Subscription<SensorGpsMsg>::SharedPtr gps;
    rclcpp::Subscription<VehicleStatusMsg>::SharedPtr vehcile_status;
    rclcpp::Subscription<localPosMsg>::SharedPtr vehicle_local_pos;
} Sub_t;

class SensorReader : public rclcpp::Node {
private:
    Sub_t sub;
    rclcpp::Publisher<twistMsg>::SharedPtr pub_;
    std::vector<bool> debug_data;
public:
    explicit SensorReader();
    void initParams();

    void readIMUCallback(const SensorCombinedMsg::UniquePtr msg);
    void readGPSCallback(const SensorGpsMsg::UniquePtr msg);
    void vehicleStatusCallback(const VehicleStatusMsg::UniquePtr msg);
    void vehicleLocalPosCallback(const localPosMsg::UniquePtr msg);
};

#endif