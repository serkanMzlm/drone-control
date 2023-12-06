#include "command.hpp"

void Command::setpointUpdate(){
    float joy_body[2] = {0.0f, 0.0f};
    float joy_word[2] = {joy_data.axes[A_PITCH], joy_data.axes[A_ROLL]};
    world2Body(joy_word, joy_body, -drone_state.attitude.yaw);
    setDesiredX(joy_body[C_X], drone_state.position.x);
    setDesiredY(joy_body[C_Y], drone_state.position.y);
    setDesiredZ(joy_data.axes[A_THR], -drone_state.position.z);
    setDesiredYaw(joy_data.axes[A_YAW], drone_state.attitude.yaw);
}

void Command::setDesiredX(float x_data, float state){
    setpoint.position.x = state + x_data * POS_COEF_X;
    setpoint.velocity.x = x_data * VEL_COEF_X;
}

void Command::setDesiredY(float y_data, float state){
    setpoint.position.y = state + y_data * POS_COEF_Y;
    setpoint.velocity.y = y_data * VEL_COEF_Y;
}

void Command::setDesiredZ(float z_data, float state){
    if(z_data > 0.1){
        is_fall = true;        
    }
    setpoint.position.z = state + z_data * POS_COEF_Z;
    setpoint.velocity.z = z_data * POS_COEF_Z;
    // std::cout << "setpoint Z: " << setpoint.position.z << " state: " 
    //                                 << state << " joy: " << z_data << std::endl;
}

void Command::setDesiredYaw(float yaw_data, float state){
    setpoint.attitude.yaw = state + (yaw_data * YAW_COEFF);
}

void Command::initSetpoint(){
    setpoint.attitude.yaw   = drone_state.attitude.yaw;
    setpoint.position.x     = drone_state.position.x;
    setpoint.position.y     = drone_state.position.y;
    setpoint.position.z     = -drone_state.position.z;
}  

bool Command::isArmChange(){
    if(joy_data.button[B_DISARM]){
        arming = DISARM;
        flag_first_point = true;
        return false;
    }else if(joy_data.button[B_ARM]){
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