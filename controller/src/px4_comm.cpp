/*///////////////////////////////////////////// 
                PX4 Communication
*//////////////////////////////////////////////
#include "controller_node.hpp"

using namespace px4_msgs::msg;

// The data sent to PX4
void Controller::vehicleArming(int state){
    vehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6); 
	vehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, state);
}

void Controller::controlMode(Mode_e mod){
	offboardControlModeMsg msg{};
	msg.position     = mod == POSITION     ? true : false;
	msg.velocity     = mod == VELOCITY     ? true : false;
	msg.acceleration = mod == ACCELERATION ? true : false;
	msg.attitude     = mod == ATTITUDE     ? true : false;
	msg.body_rate    = mod == BODY_RATE    ? true : false;
	msg.actuator     = mod == ACTUATOR     ? true : false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	pub.mode->publish(msg);
}

void Controller::trajectorySetpoint(){
 	trajectorySetpointMsg msg{};
	msg.position = {setpoint.pos.x, setpoint.pos.y, -setpoint.pos.z};
	// msg.attitude = {setpoint.attitude.roll, setpoint.attitude.pitch, setpoint.attitude.thrust};
	// msg.velocity = {setpoint.velocity.x, setpoint.velocity.y, -setpoint.velocity.z};
	msg.yaw = setpoint.att.yaw;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	pub.trajectory_setpoint->publish(msg);
}

void Controller::fallTrajectorySetpoint(float error){
 	trajectorySetpointMsg msg{};
	msg.position = {setpoint.pos.x, setpoint.pos.y, error};
	msg.yaw = setpoint.att.yaw;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	pub.trajectory_setpoint->publish(msg);
}
void Controller::attitudeSetpoint(){
    vehicleAttitudeSetpointMsg msg;
    msg.roll_body = setpoint.att.roll;
    msg.pitch_body = setpoint.att.pitch;
    msg.yaw_body = setpoint.att.yaw;
    msg.thrust_body = {0.0, 0.0, -0.7};
    pub.attitude_setpoint->publish(msg);
}

void Controller::ratesSetpoint(){
    vehicleRatesSetpointMsg msg;
    msg.roll = setpoint.att.roll;
    msg.pitch = setpoint.att.pitch;
    msg.yaw = setpoint.att.yaw;
    msg.thrust_body = {0, 0, -setpoint.att.thrust};
    pub.rates_setpoint->publish(msg);
}

void Controller::vehicleCommand(uint16_t command, float param1, float param2){
  	vehicleCommandMsg msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	pub.vehicle_command->publish(msg);
}

// The data received from PX4
void Controller::localPosCallback(localPosMsg::UniquePtr msg){
	status.pos.x = msg->x;
	status.pos.y = msg->y;
	status.pos.z = msg->z;

    status.vel.x = msg->vx;
    status.vel.y = msg->vy;
    status.vel.z = msg->vz;

    status.acc.x = msg->ax;
    status.acc.y = msg->ay;
    status.acc.z = msg->az;

	status.att.yaw = msg->heading;
	if(flag.start_point){
		start_point = msg->z;
		flag.start_point = false;
	}
}

void Controller::vehicleStatusCallback(const VehicleStatusMsg::UniquePtr msg){
    status.arming = msg->arming_state == 2 ? ARM : DISARM;
}