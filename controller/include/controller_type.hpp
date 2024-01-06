#ifndef __CONTROLLER_TYPE_HPP__
#define __CONTROLLER_TYPE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"

// Copter Msg
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_attitude_setpoint.hpp"
#include "px4_msgs/msg/vehicle_rates_setpoint.hpp"

// Generic Vehicle Msg
#include "px4_msgs/msg/vehicle_thrust_setpoint.hpp"
#include "px4_msgs/msg/vehicle_torque_setpoint.hpp"
#include "px4_msgs/msg/actuator_motors.hpp"
#include "px4_msgs/msg/actuator_servos.hpp"

#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/vehicle_control_mode.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"

#define COLOR_RED "\x1b[31m"
#define COLOR_YLW "\x1b[33m"
#define COLOR_GRN "\x1b[32m" 
#define COLOR_RST "\x1b[0m"

#define POS_COEF_X 0.5f
#define POS_COEF_Y 0.5f
#define POS_COEF_Z 1.0f

#define VEL_COEF_X 1.0f
#define VEL_COEF_Y 1.0f
#define VEL_COEF_Z 1.0f

#define YAW_COEF -0.5f

#define OFFSET 0.1f
#define FALL_OFFSET 0.05f

#define FAIL (0)
#define OK   (1)

#define MIN_VALUE -1.0F
#define MAX_VALUE 1.0F

using joyMsg = sensor_msgs::msg::Joy;
using odomMsg = px4_msgs::msg::VehicleOdometry;
using VehicleStatusMsg = px4_msgs::msg::VehicleStatus;
using localPosMsg = px4_msgs::msg::VehicleLocalPosition;

using vehicleCommandMsg = px4_msgs::msg::VehicleCommand;
using offboardControlModeMsg = px4_msgs::msg::OffboardControlMode;

using trajectorySetpointMsg = px4_msgs::msg::TrajectorySetpoint;
using vehicleAttitudeSetpointMsg = px4_msgs::msg::VehicleAttitudeSetpoint;
using vehicleRatesSetpointMsg = px4_msgs::msg::VehicleRatesSetpoint;

typedef struct{
    rclcpp::Publisher<offboardControlModeMsg>::SharedPtr mode;
    rclcpp::Publisher<vehicleCommandMsg>::SharedPtr vehicle_command;
    rclcpp::Publisher<trajectorySetpointMsg>::SharedPtr trajectory_setpoint;
    rclcpp::Publisher<vehicleAttitudeSetpointMsg>::SharedPtr attitude_setpoint;
    rclcpp::Publisher<vehicleRatesSetpointMsg>::SharedPtr rates_setpoint;
}Pub_t;

typedef struct{
    rclcpp::Subscription<joyMsg>::SharedPtr joy;
    rclcpp::Subscription<localPosMsg>::SharedPtr local_pos;
    rclcpp::Subscription<odomMsg>::SharedPtr odom;
    rclcpp::Subscription<VehicleStatusMsg>::SharedPtr vehcile_status;
}Sub_t;

typedef enum{
    THR, YAW, PITCH, ROLL, AXES_ALL
} Axes_e;

typedef enum{
    DISARM, ARM, TAKEOFF, LAND, UNDEFINED, V_ALL
} Vehicle_mod;

typedef enum{
    POSITION, VELOCITY, ACCELERATION, ATTITUDE, BODY_RATE, ACTUATOR, M_ALL
} Mode_e;

typedef enum { 
  C_X, C_Y, C_Z, ALL_CC 
} cc_mode_t;

typedef struct{
  union{
    struct{
      float roll;
      float pitch;
      float thrust;
      float yaw;
    };
    float pos[4];
  };
}Attitude_t;

typedef struct  {
  union{
    struct{
      float x;
      float y;
      float z;
    };
    float pos[3];
  };
}Cartesian_s ;

typedef struct{
    Cartesian_s  pos;
    Cartesian_s  vel;
    Cartesian_s  acc;
    Attitude_t att;
    int arming; // arm disarm
}State_t; 

typedef struct{
    bool arm; 
    bool joy;
    bool press;
    bool takeoff;
    bool land;
    bool start_point;
    bool fall;
}Flag_t;

typedef struct{
   float axes[AXES_ALL];
   int button[V_ALL];
}Joy_t;

typedef enum {STATE, ALL_DATA} Data_t;

#endif