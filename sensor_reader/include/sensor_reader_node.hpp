#ifndef __SENSOR_READER_HPP__ 
#define __SENSOR_READER_HPP__

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/sensor_combined.hpp"
#include "px4_msgs/msg/sensor_gps.hpp"

#include "sensor_reader_type.hpp"

using SensorCombinedMsg = px4_msgs::msg::SensorCombined;
using SensorGpsdMsg = px4_msgs::msg::SensorGps;

typedef struct {
    rclcpp::Subscription<SensorCombinedMsg>::SharedPtr imu;
    rclcpp::Subscription<SensorGpsdMsg>::SharedPtr gps;
} Sub_t;

class SensorReader : public rclcpp::Node {
private:
    Sub_t sub;
    std::vector<bool> debug_data;
public:
    explicit SensorReader();
    void initParams();

    void readIMUCallback(const SensorCombinedMsg::UniquePtr msg);
    void readGPSCallback(const SensorGpsdMsg::UniquePtr msg);
};

#endif