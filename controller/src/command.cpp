#include "command.hpp"

void Command::controlMode(){
	offboardControlModeMsg msg{};
	msg.position     = true;
	msg.velocity     = false;
	msg.acceleration = false;
	msg.attitude     = false;
	msg.body_rate    = false;
	// msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	pub.mode->publish(msg);
}

void Command::trajectorySetpoint(State_t new_data){
 	trajectorySetpointMsg msg{};
	// msg.position = {new_data.x, new_data.y, -new_data.z};
	// msg.yaw = D2R(new_data.yaw);
	// msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	pub.setpoint->publish(msg);
}

void Command::vehicleCommand(uint16_t command, float param1, float param2){
  	vehicleCommandMsg msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	// msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	pub.vehicle_command->publish(msg);
}