#include "sensor_reader_node.hpp"

using namespace std::placeholders;

SensorReader::SensorReader(): Node("sensor_reader_node"){
    std::cout << "Sensor Reader başlatıldı" << std::endl;
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
    sub.imu = this->create_subscription<SensorCombinedMsg>("/fmu/out/sensor_combined", qos,
                            std::bind(&SensorReader::readIMUCallback, this, _1));
    sub.gps = this->create_subscription<SensorGpsdMsg>("/fmu/out/vehicle_gps_position", qos,
                            std::bind(&SensorReader::readGPSCallback, this, _1));
}

SensorReader::~SensorReader(){

}

void SensorReader::readIMUCallback(const SensorCombinedMsg::UniquePtr msg){
    std::cout << "RECEIVED VEHICLE IMU DATA"   << std::endl;
    std::cout << "============================="   << std::endl;
    std::cout << "gyro: " << msg->gyro_rad[0] << ", " << msg->gyro_rad[1] 
                          << ", " << msg->gyro_rad[2] << std::endl;
    std::cout << "acc: " << msg->accelerometer_m_s2[0] << ", " <<  msg->accelerometer_m_s2[1]
                         << ", " << msg->accelerometer_m_s2[2] << std::endl;
}

void SensorReader::readGPSCallback(const SensorGpsdMsg::UniquePtr msg){
    std::cout << "RECEIVED VEHICLE GPS POSITION DATA"   << std::endl;
    std::cout << "=================================="   << std::endl;
    std::cout << "alt_ellipsoid: " << msg->alt_ellipsoid << std::endl;
    std::cout << "s_variance_m_s: " << msg->s_variance_m_s << std::endl;
    std::cout << "c_variance_rad: " << msg->c_variance_rad << std::endl;
    std::cout << "fix_type: " << msg->fix_type << std::endl;
    std::cout << "eph: " << msg->eph << std::endl;
    std::cout << "epv: " << msg->epv << std::endl;
    std::cout << "hdop: " << msg->hdop << std::endl;
    std::cout << "vdop: " << msg->vdop << std::endl;
    std::cout << "noise_per_ms: " << msg->noise_per_ms << std::endl;
    std::cout << "vel_m_s: " << msg->vel_m_s << std::endl;
    std::cout << "vel_n_m_s: " << msg->vel_n_m_s << std::endl;
    std::cout << "vel_e_m_s: " << msg->vel_e_m_s << std::endl;
    std::cout << "vel_d_m_s: " << msg->vel_d_m_s << std::endl;
    std::cout << "cog_rad: " << msg->cog_rad << std::endl;
    std::cout << "vel_ned_valid: " << msg->vel_ned_valid << std::endl;
    std::cout << "timestamp_time_relative: " << msg->timestamp_time_relative << std::endl;
    std::cout << "time_utc_usec: " << msg->time_utc_usec << std::endl;
    std::cout << "satellites_used: " << msg->satellites_used << std::endl;
    std::cout << "heading: " << msg->heading << std::endl;
    std::cout << "heading_offset: " << msg->heading_offset << std::endl;
}

int main(int argc, char *argv[]){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SensorReader>());
	rclcpp::shutdown();
	return 0;
}