#include "controller_node.hpp"

using namespace std::placeholders;
using namespace px4_msgs::msg;

Controller::Controller(): Node("controller_node"){
	initTopic();
}

void Controller::iniAirMode(){
	if(isArmChange()) { return; }
	if(getArming() == ARM){
		initSetpoint();
	}
	vehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6); 
	vehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, getArming());
}


void Controller::initTopic(){
	rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
	auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

	pub.mode = this->create_publisher<offboardControlModeMsg>("/fmu/in/offboard_control_mode", 10);
	pub.setpoint = this->create_publisher<trajectorySetpointMsg>("/fmu/in/trajectory_setpoint", 10);
	pub.vehicle_command = this->create_publisher<vehicleCommandMsg>("/fmu/in/vehicle_command", 10);

	sub.joy = this->create_subscription<joyMsg>("joy", 10, 
				std::bind(&Controller::joyCallback, this, _1));		
	sub.local_pos = this->create_subscription<localPosMsg>("/fmu/out/vehicle_local_position", qos,
							std::bind(&Controller::localPosCallback, this, _1));
	// sub.odom = this->create_subscription<odomMsg>("/fmu/out/vehicle_odometry", qos, 
	// 						std::bind(&Controller::odomCallback, this, _1));

	timer = this->create_wall_timer(std::chrono::milliseconds(F2P(50)), 
							std::bind(&Controller::controllerCallback, this));
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
	drone_state.position.x = msg->x;
	drone_state.position.y = msg->y;
	drone_state.position.z = msg->z;
	drone_state.attitude.yaw = msg->heading;
}


void Controller::controllerCallback(){
	iniAirMode();
	if(!getArming()){ return; }
	setpointUpdate();
	controlMode(M_POSITION);
	vehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
	trajectorySetpoint();
}

//////////////////////// PX4 Communication ////////////////////////
void Controller::controlMode(Mode_e mod){
	offboardControlModeMsg msg{};
	msg.position     = mod == M_POSITION ? true : false;
	msg.velocity     = mod == M_VELOCITY ? true : false;
	msg.acceleration = mod == M_ACCELERATION ? true : false;
	msg.attitude     = mod == M_ATTITUDE ? true : false;
	msg.body_rate    = mod == M_BODY_RATE ? true : false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	pub.mode->publish(msg);
}

void Controller::trajectorySetpoint(){
 	trajectorySetpointMsg msg{};
	msg.position = {setpoint.position.x, setpoint.position.y, -setpoint.position.z};
	// msg.velocity = {setpoint.velocity.x, setpoint.velocity.y, -setpoint.velocity.z};
	msg.yaw = setpoint.attitude.yaw;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
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

int main(int argc, char ** args){
  rclcpp::init(argc, args);
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();
  return 0;
}