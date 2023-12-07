#include "command.hpp"

void Command::setpointUpdate(){
    float joy_body[2] = {0.0f, 0.0f};
    float joy_word[2] = {joy_data.axes[PITCH], joy_data.axes[ROLL]};
    world2Body(joy_word, joy_body, -status.att.yaw);
    setDesiredX(joy_body[C_X], status.pos.x);
    setDesiredY(joy_body[C_Y], status.pos.y);
    setDesiredZ(joy_data.axes[THR], -status.pos.z);
    setDesiredYaw(joy_data.axes[YAW], status.att.yaw);
}

void Command::setDesiredX(float x_data, float state){
    setpoint.pos.x = state + x_data * POS_COEF_X;
    setpoint.vel.x = x_data * VEL_COEF_X;
    setpoint.att.pitch = constrain(x_data, MIN_VALUE, MAX_VALUE);
}

void Command::setDesiredY(float y_data, float state){
    setpoint.pos.y = state + y_data * POS_COEF_Y;
    setpoint.vel.y = y_data * VEL_COEF_Y;
    setpoint.att.roll = constrain(y_data, MIN_VALUE, MAX_VALUE);
}

void Command::setDesiredZ(float z_data, float state){
    if(z_data > 0.1){
        flag.fall = true;        
    }
    setpoint.pos.z = state + z_data * POS_COEF_Z;
    setpoint.vel.z = z_data * POS_COEF_Z;
    setpoint.att.thrust = constrain(z_data, MIN_VALUE, MAX_VALUE);
}

void Command::setDesiredYaw(float yaw_data, float state){
    setpoint.att.yaw = state + (yaw_data * YAW_COEF);
}

void Command::initSetpoint(){
    setpoint.att.yaw   = status.att.yaw;
    setpoint.pos.x     = status.pos.x;
    setpoint.pos.y     = status.pos.y;
    setpoint.pos.z     = -status.pos.z;
}  

bool Command::isArmChange(){
    if(joy_data.button[DISARM]){
        arming = DISARM;
        flag.start_point = true;
        count = 0;
        return false;
    }else if(joy_data.button[ARM]){
        arming = ARM;
        return false;
    }
    return true;
}

int Command::getArming() const{
    return arming;
}

int Command::getAirMode() const{
    return air_mode;
}