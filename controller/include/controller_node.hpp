#ifndef __CONTROLLER_NODE_HPP__
#define __CONTROLLER_NODE_HPP__

#include "geometry_utils.hpp"
#include "command.hpp"

class Controller : public rclcpp::Node, public Command{
public:
    explicit Controller();
    void initTopic();

    void joyCallback(joyMsg msg);
    void localPosCallback(localPosMsg::UniquePtr msg);
    void odomCallback(odomMsg::UniquePtr msg);

    void resetData(Data_t select);
    void setPosition(joyMsg new_data);
};

#endif