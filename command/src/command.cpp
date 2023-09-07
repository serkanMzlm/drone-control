#include "command.hpp"
#include "numeric_transform.hpp"

using namespace std::placeholders;

Command::Command(): Node("command_node") {
	this->declare_parameter<bool>("is_joy", true);
	initParam();
	initTopic();
	RCLCPP_DEBUG(this->get_logger(), "Command Started.");
}

void Command::joyCallback(joyMsg joy_msg){
	flags.is_press = (joy_msg.buttons[0] != prev_button) && ((joy_msg.buttons[0] == 1) || !flags.is_joy);
	if(flags.is_press){
		this->vehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
		armDisarm();
	}	
	setPosition(joy_msg);
	controlMode();	
	trajectorySetpoint(set_point);	
	RCLCPP_DEBUG(this->get_logger(), "x: %.2f | y: %.2f | z: %.2f | yaw: %.2f",
				set_point.x, set_point.y, set_point.z, set_point.yaw);

	prev_button = joy_msg.buttons[0];
}

void Command::setPosition(joyMsg new_data){
	if(!flags.is_arm) { return; }
	set_point.x += new_data.axes[4] * coef;
	set_point.y += new_data.axes[3] * coef;
	set_point.z += new_data.axes[1] * coef;
	set_point.yaw -= new_data.axes[0] * coef;
}

void Command::setTakeoff(){

}

void Command::resetData(Data_t select){
	switch (select){
	case STATE:
		set_point.x  = 0;
		set_point.y  = 0;
		set_point.z  = 0;
		set_point.yaw = 0;
		break;
	case ALL_DATA:
		set_point.x  = 0;
		set_point.y  = 0;
		set_point.z  = 0;
		set_point.yaw = 0;
		break;
	default:
		break;
	}
}

void Command::initTopic(){
	pub.mode = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
	pub.setpoint = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
	pub.vehicle_command = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

	sub.joy = this->create_subscription<joyMsg>("/joy", 10, 
				std::bind(&Command::joyCallback, this, _1));
}

void Command::initParam(){
	flags.is_joy = this->get_parameter("is_joy").as_bool();
}

/////////////////////////////////////////
void Command::armDisarm(){
  flags.is_arm = !flags.is_arm;
  vehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, (flags.is_arm ? 1.0 : 0.0));
  RCLCPP_INFO(this->get_logger(), "%s command send", (flags.is_arm ? "Arm" : "Disarm"));
}

void Command::controlMode(){
	OffboardControlMode msg{};
	msg.position     = true;
	msg.velocity     = false;
	msg.acceleration = false;
	msg.attitude     = false;
	msg.body_rate    = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	pub.mode->publish(msg);
}

void Command::trajectorySetpoint(Satate_t new_data){
 	TrajectorySetpoint msg{};
	msg.position = {new_data.x, new_data.y, -new_data.z};
	msg.yaw = new_data.yaw;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	pub.setpoint->publish(msg);
}

void Command::vehicleCommand(uint16_t command, float param1, float param2){
  	VehicleCommand msg{};
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
    
int main(int argc, char ** args){
  rclcpp::init(argc, args);
  rclcpp::spin(std::make_shared<Command>());
  rclcpp::shutdown();
  return 0;
}