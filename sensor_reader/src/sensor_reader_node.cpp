#include "sensor_reader_node.hpp"

using namespace std::placeholders;

SensorReader::SensorReader(): Node("sensor_reader_node"){
    initParams();
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
    sub.imu = this->create_subscription<SensorCombinedMsg>("/fmu/out/sensor_combined", qos,
                            std::bind(&SensorReader::readIMUCallback, this, _1));
    sub.gps = this->create_subscription<SensorGpsMsg>("/fmu/out/vehicle_gps_position", qos,
                            std::bind(&SensorReader::readGPSCallback, this, _1));
    sub.vehcile_status = this->create_subscription<VehicleStatusMsg>("/fmu/out/vehicle_status", qos,
                            std::bind(&SensorReader::vehicleStatusCallback, this, _1));
}

void SensorReader::initParams(){
    this->declare_parameter("debug", std::vector<bool> (ALL_DEBUG, false));
    debug_data = this->get_parameter("debug").as_bool_array();
}

void SensorReader::readIMUCallback(const SensorCombinedMsg::UniquePtr msg){
    if(debug_data[IMU]){
        RCLCPP_INFO(this->get_logger(), "=============================");
        RCLCPP_INFO(this->get_logger(), "RECEIVED VEHICLE IMU DATA");
        RCLCPP_INFO(this->get_logger(), "=============================");
        RCLCPP_INFO(this->get_logger(), "Acc: %.2f, %.2f, %.2f ",msg->accelerometer_m_s2[0],
                                        msg->accelerometer_m_s2[1], msg->accelerometer_m_s2[2]);
        RCLCPP_INFO(this->get_logger(), "Gyro: %.2f, %.2f, %.2f ",msg->gyro_rad[0], msg->gyro_rad[1],
                                                                                    msg->gyro_rad[2]);
    }
}

void SensorReader::readGPSCallback(const SensorGpsMsg::UniquePtr msg){
    if(debug_data[GPS]){
        RCLCPP_INFO(this->get_logger(), "==================");
        RCLCPP_INFO(this->get_logger(), "RECEIVED GPS DATA");
        RCLCPP_INFO(this->get_logger(), "==================");
        RCLCPP_INFO(this->get_logger(), "Altitude: %.2lf", msg->alt);
        RCLCPP_INFO(this->get_logger(), "Altitude: %.2lf", msg->alt_ellipsoid);
        RCLCPP_INFO(this->get_logger(), "GPS Horizonal: %.2f", msg->eph);
        RCLCPP_INFO(this->get_logger(), "GPS Vertical: %.2f", msg->epv);
        RCLCPP_INFO(this->get_logger(), "GPS Ground Speed: %.2f", msg->vel_m_s);
        RCLCPP_INFO(this->get_logger(), "GPS North Velocity: %.2f", msg->vel_n_m_s);
        RCLCPP_INFO(this->get_logger(), "GPS East Velocity: %.2f", msg->vel_e_m_s);
        RCLCPP_INFO(this->get_logger(), "GPS Down Velocity: %.2f", msg->vel_d_m_s);
        RCLCPP_INFO(this->get_logger(), "Heading : %.2f", msg->heading);
        RCLCPP_INFO(this->get_logger(), "Heading Offset : %.2f", msg->heading_offset);
    }
}

void SensorReader::vehicleStatusCallback(const VehicleStatusMsg::UniquePtr msg){
    if(debug_data[VEH_STATUS]){
        RCLCPP_INFO(this->get_logger(), "============================");
        RCLCPP_INFO(this->get_logger(), "RECEIVED VEHICLE STATUS DATA");
        RCLCPP_INFO(this->get_logger(), "============================");
        RCLCPP_INFO(this->get_logger(), "The Time to Arm (microseconds): %ld", msg->armed_time);
        RCLCPP_INFO(this->get_logger(), "The Time to Takeoff (microseconds): %ld", msg->takeoff_time);
        RCLCPP_INFO(this->get_logger(), "Arm Status: %d", msg->arming_state);
        RCLCPP_INFO(this->get_logger(), "Selected Mode: %ld", msg->nav_state_timestamp);
        RCLCPP_INFO(this->get_logger(), "Vehicle Mode: %d", msg->nav_state);
        RCLCPP_INFO(this->get_logger(), "Number of Detected Faild: %d", msg->failure_detector_status);
        RCLCPP_INFO(this->get_logger(), "Vehicle Type: %d", msg->vehicle_type);
        RCLCPP_INFO(this->get_logger(), "Failsafe: %d", msg->failsafe);
        RCLCPP_INFO(this->get_logger(), "USB Connected: %s", msg->usb_connected ? "true" : "false");
    }
}

int main(int argc, char *argv[]){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SensorReader>());
	rclcpp::shutdown();
	return 0;
}