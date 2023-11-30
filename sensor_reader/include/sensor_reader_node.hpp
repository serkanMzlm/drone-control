#ifndef __SENSOR_READER_HPP__ 
#define __SENSOR_READER_HPP__

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/sensor_combined.hpp"

using SensorCombinedMsg = px4_msgs::msg::SensorCombined;

typedef struct {
    rclcpp::Subscription<SensorCombinedMsg>::SharedPtr imu;
    // rclcpp::Subscription<SensorCombinedMsg>::SharedPtr gps;
} Sub_t;

class SensorReader : public rclcpp::Node {
private:
    Sub_t sub;
public:
    explicit SensorReader();
    ~SensorReader();
    void readIMUCallback(const SensorCombinedMsg::UniquePtr msg);
    // void readGPS(const);
};

#endif