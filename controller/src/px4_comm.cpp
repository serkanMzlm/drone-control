#include "controller_node.hpp"

using namespace px4_msgs::msg;

//////////////////////// PX4 Communication ////////////////////////
void Controller::controlMode(Mode_e mod){
	offboardControlModeMsg msg{};
	msg.position     = mod == M_POSITION ? true : false;
	msg.velocity     = mod == M_VELOCITY ? true : false;
	msg.acceleration = mod == M_ACCELERATION ? true : false;
	msg.attitude     = mod == M_ATTITUDE ? true : false;
	msg.body_rate    = mod == M_BODY_RATE ? true : false;
	msg.timestamp = rclcpp::Node::get_clock()->now().nanoseconds() / 1000;
	pub.mode->publish(msg);
}

void Controller::trajectorySetpoint(){
 	trajectorySetpointMsg msg{};
	msg.position = {setpoint.position.x, setpoint.position.y, -setpoint.position.z};
	// msg.attitude = {setpoint.attitude.roll, setpoint.attitude.pitch, setpoint.attitude.thrust};
	// msg.velocity = {setpoint.velocity.x, setpoint.velocity.y, -setpoint.velocity.z};
	msg.yaw = setpoint.attitude.yaw;
	msg.timestamp = rclcpp::Node::get_clock()->now().nanoseconds() / 1000;
	pub.setpoint->publish(msg);
}

void Controller::fallTrajectorySetpoint(float error){
 	trajectorySetpointMsg msg{};
	msg.position = {setpoint.position.x, setpoint.position.y, error};
	// msg.velocity = {setpoint.velocity.x, setpoint.velocity.y, -setpoint.velocity.z};
	msg.yaw = setpoint.attitude.yaw;
	msg.timestamp = rclcpp::Node::get_clock()->now().nanoseconds() / 1000;
	pub.setpoint->publish(msg);
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

void Controller::controllerCallback(){
	iniAirMode();
	if(!getArming()){ return; }
	setpointUpdate();
	controlMode(M_POSITION);
	vehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
	trajectorySetpoint();
}


void Controller::localPosCallback(localPosMsg::UniquePtr msg){
	RCLCPP_DEBUG(this->get_logger(), "state: x: %.2f | y: %.2f | z: %.2f | yaw: %.2f",
				msg->x, msg->y, msg->z, msg->heading);	
	drone_state.position.x = msg->x;
	drone_state.position.y = msg->y;
	drone_state.position.z = msg->z;
	drone_state.attitude.yaw = msg->heading;
    fall_vel = msg->vz;
	if(flag_first_point){
		start_point = msg->z;
		flag_first_point = false;
	}
}

void Controller::vehicleStatusCallback(const VehicleStatusMsg::UniquePtr msg){
        // RCLCPP_INFO(this->get_logger(), "Arm Status: %d", msg->arming_state);
		vehicle_arm_status = msg->arming_state;
}