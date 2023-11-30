#ifndef __COMMAND_NODE_HPP__
#define __COMMAND_NODE_HPP__

#include <chrono>
#include "geometry_utils.hpp"

class Command{
public:
    Pub_t pub;
    Sub_t sub;

    Joy_t joy_data;
    State_t state;
    State_t setpoint;

public:
    void controlMode();
	void trajectorySetpoint(State_t new_data);
	void vehicleCommand(uint16_t command, float param1 = 0.0, float param2 = 0.0);
};

#endif