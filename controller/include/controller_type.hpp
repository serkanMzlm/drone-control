#ifndef __CONTROLLER_TYPE_HPP__
#define __CONTROLLER_TYPE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_control_mode.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"

#define POS_COEF_X 1.0f
#define POS_COEF_Y 1.0f
#define POS_COEF_Z 1.0f

#define VEL_COEF_X 1.0f
#define VEL_COEF_Y 1.0f
#define VEL_COEF_Z 1.0f

#define YAW_COEFF -0.5f

#define OFFSET 0.1f

#define ARM     1
#define DISARM  0

#define FAIL (0)
#define OK   (1)

using joyMsg = sensor_msgs::msg::Joy;
using odomMsg = px4_msgs::msg::VehicleOdometry;
using VehicleStatusMsg = px4_msgs::msg::VehicleStatus;
using vehicleCommandMsg = px4_msgs::msg::VehicleCommand;
using localPosMsg = px4_msgs::msg::VehicleLocalPosition;
using trajectorySetpointMsg = px4_msgs::msg::TrajectorySetpoint;
using offboardControlModeMsg = px4_msgs::msg::OffboardControlMode;

typedef struct{
    rclcpp::Publisher<offboardControlModeMsg>::SharedPtr mode;
    rclcpp::Publisher<trajectorySetpointMsg>::SharedPtr setpoint;
    rclcpp::Publisher<vehicleCommandMsg>::SharedPtr vehicle_command;
}Pub_t;

typedef struct{
    rclcpp::Subscription<joyMsg>::SharedPtr joy;
    rclcpp::Subscription<localPosMsg>::SharedPtr local_pos;
    rclcpp::Subscription<odomMsg>::SharedPtr odom;
    rclcpp::Subscription<VehicleStatusMsg>::SharedPtr vehcile_status;
}Sub_t;

typedef enum{
    A_THR, A_YAW, A_PITCH, A_ROLL, AXES_ALL
} Axes_e;

typedef enum{
    B_ARM, B_DISARM, B_TAKEOFF, B_LAND, B_ALL
} Button_e;

typedef enum{
    M_POSITION, M_VELOCITY, M_ACCELERATION, M_ATTITUDE, M_BODY_RATE
} Mode_e;

typedef enum{
    A_LAND, A_TAKEOFF, A_UNDEFINED
} Air_mode_e;

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
    Cartesian_s  position;
    Cartesian_s  velocity;
    Attitude_t attitude;
} State_t; 

typedef struct{
    bool is_arm;
    bool is_joy;
    bool is_press;
    bool is_takeoff;
    bool is_land;
} Flag_t;

typedef struct{
   float axes[AXES_ALL];
   int button[B_ALL];
} Joy_t;

typedef enum {STATE, ALL_DATA} Data_t;

#endif