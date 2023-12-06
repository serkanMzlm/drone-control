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

void Controller::detectFallCallback(){
	if(vehicle_arm_status != 1 && is_fall) { return; }
	int diff = start_point - drone_state.position.z;
	if(abs(diff) > 0.5){
		is_fall = false;
		std::cout << COLOR_RED   << "The drone is losing altitude..." << std::endl;
		vehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6); 
		vehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1);
		controlMode(M_POSITION);
		fallTrajectorySetpoint(start_point);
	}
	if(fall_vel < 0.0 ){
		start_point = drone_state.position.z;
		is_fall = true;
		std::cout << COLOR_GRN << "The vehicle's descent rate has been halted." << std::endl;

	}
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
	sub.vehcile_status = this->create_subscription<VehicleStatusMsg>("/fmu/out/vehicle_status", qos,
                            std::bind(&Controller::vehicleStatusCallback, this, _1));
	// sub.odom = this->create_subscription<odomMsg>("/fmu/out/vehicle_odometry", qos, 
	// 						std::bind(&Controller::odomCallback, this, _1));

	timer = this->create_wall_timer(std::chrono::milliseconds(F2P(50)), 
							std::bind(&Controller::controllerCallback, this));
	fall_timer = this->create_wall_timer(std::chrono::milliseconds(F2P(10)), 
							std::bind(&Controller::detectFallCallback, this));
}

int main(int argc, char ** args){
  rclcpp::init(argc, args);
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();
  return 0;
}