#include "sensor_reader_node.hpp"

using namespace std::placeholders;

SensorReader::SensorReader(): Node("sensor_reader_node"){
    std::cout << "Sensor Reader başlatıldı" << std::endl;
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
    sub.imu = this->create_subscription<SensorCombinedMsg>("/fmu/out/sensor_combined", qos,
                            std::bind(&SensorReader::readIMUCallback, this, _1));
}

SensorReader::~SensorReader(){

}

void SensorReader::readIMUCallback(const SensorCombinedMsg::UniquePtr msg){
    std::cout << "============================="   << std::endl;
    std::cout << "gyro: " << msg->gyro_rad[0] << ", " << msg->gyro_rad[1] 
                          << ", " << msg->gyro_rad[2] << std::endl;
    std::cout << "acc: " << msg->accelerometer_m_s2[0] << ", " <<  msg->accelerometer_m_s2[1]
                         << ", " << msg->accelerometer_m_s2[2] << std::endl;
}

int main(int argc, char *argv[]){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SensorReader>());
	rclcpp::shutdown();
	return 0;
}