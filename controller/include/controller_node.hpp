#ifndef __CONTROLLER_NODE_HPP__
#define __CONTROLLER_NODE_HPP__

#include "geometry_utils.hpp"
#include "command.hpp"

class Controller : public rclcpp::Node, public Command{
private:
    rclcpp::TimerBase::SharedPtr timer;
public:
    explicit Controller();
    void initTopic();

    void joyCallback(joyMsg msg);
    void localPosCallback(localPosMsg::UniquePtr msg);
    void iniAirMode();
    void controllerCallback();

    void controlMode(Mode_e mod);
	void trajectorySetpoint();
	void vehicleCommand(uint16_t command, float param1 = 0.0, float param2 = 0.0);
};

#endif