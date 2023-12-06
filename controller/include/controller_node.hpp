#ifndef __CONTROLLER_NODE_HPP__
#define __CONTROLLER_NODE_HPP__

#include "geometry_utils.hpp"
#include "command.hpp"

class Controller : public rclcpp::Node, public Command{
private:
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::TimerBase::SharedPtr fall_timer;
public:
    explicit Controller();
    void initTopic();

    void joyCallback(joyMsg msg);
    void localPosCallback(localPosMsg::UniquePtr msg);
    void vehicleStatusCallback(const VehicleStatusMsg::UniquePtr msg);
    void iniAirMode();
    void controllerCallback();
    void detectFallCallback();

    void controlMode(Mode_e mod);
	void trajectorySetpoint();
    void fallTrajectorySetpoint(float error);
	void vehicleCommand(uint16_t command, float param1 = 0.0, float param2 = 0.0);
};

#endif