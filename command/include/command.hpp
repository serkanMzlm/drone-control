#ifndef __COMMAND_HPP__
#define __COMMAND_HPP__

#include "command_type.hpp"

#include <chrono>
#include <cmath>

class Command : public rclcpp::Node{
private:
    Pub_t pub;
    Sub_t sub;
    Flag_t flags {false, false, false};
    float coef = 0.1f;
    int prev_button = 0;
    Satate_t set_point {0.0, 0.0, 0.0, 0.0};
public:
    explicit Command();

    void joyCallback(joyMsg joy_msg);
    void sensorListenerCallback(sensorCombinedMsg::UniquePtr sensor_msg);
    void localPosCallback(localPosMsg::UniquePtr pos_msg);
    void odomCallback(odomMsg::UniquePtr odom_msg);
       
    void resetData(Data_t select);
    void initTopic();
    void initParam();
    void setPosition(joyMsg new_data);
    void setTakeoff();

    void controlMode();
	void trajectorySetpoint(Satate_t new_data);
	void vehicleCommand(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    void armDisarm();
};

#endif