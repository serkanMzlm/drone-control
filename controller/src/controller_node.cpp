#include "controller_node.hpp"

using namespace std::placeholders;
using namespace px4_msgs::msg;


Controller::Controller(): Node("controller_node"){
	flag.fall = true;
	initTopic();
}

void Controller::iniAirMode(){
	if(isArmChange()) { return; }
	if(getArming() == ARM){
		initSetpoint();
	}
	vehicleArming(getArming());
}

void Controller::controllerCallback(){
	iniAirMode();
	if(!getArming()){ return; }
	setpointUpdate();
	controlMode(POSITION);
	vehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
	trajectorySetpoint();
	// attitudeSetpoint();
	// ratesSetpoint();
}

void Controller::detectFallCallback(){
	if(status.arming == ARM && flag.fall) { return; }
	int diff = start_point - status.pos.z;
	if(status.pos.z > -5.0f && count < 1 && diff < -FALL_OFFSET){
		flag.fall = false;
		std::cout << COLOR_RED   << "The drone is losing altitude..." << COLOR_RST << std::endl;
		vehicleArming(ARM);
		controlMode(ATTITUDE);
		attitudeSetpoint();
		count++;
		// fallTrajectorySetpoint(start_point);
	}
	if(status.vel.z < 0.1f){
		start_point = status.pos.z;
		flag.fall = true;
		// std::cout << COLOR_GRN << "The vehicle's descent rate has been halted." << COLOR_RST << std::endl;
	}
}

void Controller::joyCallback(joyMsg msg){
	joy_data.button[ARM] = msg.buttons[0];
	joy_data.button[DISARM] = msg.buttons[1];
	joy_data.button[TAKEOFF] = msg.buttons[2];
	joy_data.button[LAND] = msg.buttons[3];

	joy_data.axes[YAW]   = FILTER_OFFSET(msg.axes[0]);
	joy_data.axes[THR]   = FILTER_OFFSET(msg.axes[1]);
	joy_data.axes[ROLL]  = FILTER_OFFSET(msg.axes[3]);
	joy_data.axes[PITCH] = FILTER_OFFSET(msg.axes[4]);
}

void Controller::initTopic(){
	rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
	auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

	pub.mode = this->create_publisher<offboardControlModeMsg>("/fmu/in/offboard_control_mode", 10);
	pub.vehicle_command = this->create_publisher<vehicleCommandMsg>("/fmu/in/vehicle_command", 10);
	pub.trajectory_setpoint = this->create_publisher<trajectorySetpointMsg>("/fmu/in/trajectory_setpoint", 10);
	pub.attitude_setpoint = this->create_publisher<vehicleAttitudeSetpointMsg>("/fmu/in/vehicle_attitude_setpoint", 10);
	pub.rates_setpoint = this->create_publisher<vehicleRatesSetpointMsg>("/fmu/in/vehicle_rates_setpoint", 10);

	sub.joy = this->create_subscription<joyMsg>("joy", 10, 
				std::bind(&Controller::joyCallback, this, _1));		
	sub.local_pos = this->create_subscription<localPosMsg>("/fmu/out/vehicle_local_position", qos,
							std::bind(&Controller::localPosCallback, this, _1));
	sub.vehcile_status = this->create_subscription<VehicleStatusMsg>("/fmu/out/vehicle_status", qos,
                            std::bind(&Controller::vehicleStatusCallback, this, _1));

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