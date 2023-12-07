#ifndef __COMMAND_NODE_HPP__
#define __COMMAND_NODE_HPP__

#include <chrono>
#include "geometry_utils.hpp"

class Command{
private:
    int arming = DISARM; 
    int air_mode = UNDEFINED;

protected:
    Pub_t pub;
    Sub_t sub;

    Joy_t joy_data;
    State_t status;
    State_t setpoint;

    Flag_t flag;
    float start_point;
    int count = 0;

public:
    // void updateState();
    void setpointUpdate();
    void setDesiredX(float x_data, float state);
    void setDesiredY(float y_data, float state);
    void setDesiredZ(float z_data, float state);
    void setDesiredYaw(float yaw_data, float state);

    void initSetpoint(); 
    bool isArmChange();   
    int getArming() const;
    int getAirMode() const;
};

#endif