#include "command.hpp"
#include "numeric_transform.hpp"
// #define DEBUG_SENSOR
using namespace std::placeholders;

Command::Command(): Node("command_node") {
	this->declare_parameter<bool>("is_joy", true);

	rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
	auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
	sub.sensor_combine = this->create_subscription<sensorCombinedMsg>("/fmu/out/sensor_combined", qos,
							std::bind(&Command::sensorListenerCallback, this, _1));
	sub.local_pos = this->create_subscription<localPosMsg>("/fmu/out/vehicle_local_position", qos,
							std::bind(&Command::localPosCallback, this, _1));
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

void Command::sensorListenerCallback(sensorCombinedMsg::UniquePtr sensor_msg){
#ifdef DEBUG_SENSOR
	std::cout << "\n\n\n";
	std::cout << "RECEIVED SENSOR COMBINED DATA"   << std::endl;
	std::cout << "============================="   << std::endl;
	std::cout << "ts: "          << sensor_msg->timestamp    << std::endl;
	std::cout << "gyro_rad[0]: " << sensor_msg->gyro_rad[0]  << std::endl;
	std::cout << "gyro_rad[1]: " << sensor_msg->gyro_rad[1]  << std::endl;
	std::cout << "gyro_rad[2]: " << sensor_msg->gyro_rad[2]  << std::endl;
	std::cout << "gyro_integral_dt: " << sensor_msg->gyro_integral_dt << std::endl;
	std::cout << "accelerometer_timestamp_relative: " << sensor_msg->accelerometer_timestamp_relative << std::endl;
	std::cout << "accelerometer_m_s2[0]: " << sensor_msg->accelerometer_m_s2[0] << std::endl;
	std::cout << "accelerometer_m_s2[1]: " << sensor_msg->accelerometer_m_s2[1] << std::endl;
	std::cout << "accelerometer_m_s2[2]: " << sensor_msg->accelerometer_m_s2[2] << std::endl;
	std::cout << "accelerometer_integral_dt: " << sensor_msg->accelerometer_integral_dt << std::endl;
#endif
}

void Command::localPosCallback(localPosMsg::UniquePtr pos_msg){
	// RCLCPP_INFO(this->get_logger(), "x: %.2f | y: %.2f | z: %.2f | yaw: %.2f",
	// 			pos_msg->x, pos_msg->y, pos_msg->z, pos_msg->heading);	
				RCLCPP_INFO(this->get_logger(), "--------");	
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
	// sub.sensor_combine = this->create_subscription<sensorCombinedMsg>("/fmu/out/sensor_combined", 10,
	// 						std::bind(&Command::sensorListenerCallback, this, _1));
	// sub.local_pos = this->create_subscription<localPosMsg>("/fmu/out/vehicle_local_position", 10,
	// 						std::bind(&Command::localPosCallback, this, _1));
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