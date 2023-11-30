#include "controller_node.hpp"

using namespace std::placeholders;
// this->get_clock()->now().nanoseconds() 
Controller::Controller(): Node("controller_node"){
	initTopic();
}

void Controller::initTopic(){
	rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
	auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

	pub.mode = this->create_publisher<offboardControlModeMsg>("/fmu/in/offboard_control_mode", 10);
	pub.setpoint = this->create_publisher<trajectorySetpointMsg>("/fmu/in/trajectory_setpoint", 10);
	pub.vehicle_command = this->create_publisher<vehicleCommandMsg>("/fmu/in/vehicle_command", 10);

	sub.joy = this->create_subscription<joyMsg>("/joy", 10, 
				std::bind(&Controller::joyCallback, this, _1));		
	sub.local_pos = this->create_subscription<localPosMsg>("/fmu/out/vehicle_local_position", qos,
							std::bind(&Controller::localPosCallback, this, _1));
	sub.odom = this->create_subscription<odomMsg>("/fmu/out/vehicle_odometry", qos, 
							std::bind(&Controller::odomCallback, this, _1));
}

void Controller::joyCallback(joyMsg msg){
	joy_data.button[B_ARM] = msg.buttons[0];
	joy_data.button[B_DISARM] = msg.buttons[1];
	joy_data.button[B_TAKEOFF] = msg.buttons[2];
	joy_data.button[B_LAND] = msg.buttons[3];

	joy_data.axes[A_YAW]   = FILTER_OFFSET(msg.axes[0]);
	joy_data.axes[A_THR]   = FILTER_OFFSET(msg.axes[1]);
	joy_data.axes[A_ROLL]  = FILTER_OFFSET(msg.axes[3]);
	joy_data.axes[A_PITCH] = FILTER_OFFSET(msg.axes[4]);
}

void Controller::localPosCallback(localPosMsg::UniquePtr msg){
	RCLCPP_DEBUG(this->get_logger(), "state: x: %.2f | y: %.2f | z: %.2f | yaw: %.2f",
				msg->x, msg->y, msg->z, msg->heading);	
}

void Controller::odomCallback(odomMsg::UniquePtr msg){
	RCLCPP_DEBUG(this->get_logger(), "x: %.2f | y: %.2f | z: %.2f",
				msg->position[0], msg->position[1], msg->position[2]);	
}

void Controller::setPosition(joyMsg new_data){
	// if(!flags.is_arm) { return; }
	// set_point.x = vec_state.x + new_data.axes[4] * coef;
	// set_point.y = vec_state.y + new_data.axes[3] * coef;
	// set_point.z = -vec_state.z + new_data.axes[1] * coef;
	// set_point.yaw = vec_state.yaw + new_data.axes[0];
}

void Controller::resetData(Data_t select){
	// switch (select){
	// case STATE:
	// 	setpoint.x  = 0;
	// 	setpoint.y  = 0;
	// 	setpoint.z  = 0;
	// 	setpoint.yaw = 0;
	// 	break;
	// case ALL_DATA:
	// 	setpoint.x  = 0;
	// 	setpoint.y  = 0;
	// 	setpoint.z  = 0;
	// 	setpoint.yaw = 0;
	// 	break;
	// default:
	// 	break;
	// }
}

int main(int argc, char ** args){
  rclcpp::init(argc, args);
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();
  return 0;
}