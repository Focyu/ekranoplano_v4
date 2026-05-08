// Copyright 2022-2024 The MathWorks, Inc.
// Generated 07-May-2026 21:51:11
#include "slros2_initialize.h"
const std::string SLROSNodeName("pid_control_V4");
// pid_control_V4/Publish
SimulinkPublisher<sensor_msgs::msg::Imu,SL_Bus_sensor_msgs_Imu> Pub_pid_control_V4_548;
// pid_control_V4/Publish1
SimulinkPublisher<nav_msgs::msg::Odometry,SL_Bus_nav_msgs_Odometry> Pub_pid_control_V4_697;
// pid_control_V4/SUBSYSTEM_MODEL/Subscribe1_TURBULENCIA1
SimulinkSubscriber<std_msgs::msg::Bool,SL_Bus_std_msgs_Bool> Sub_pid_control_V4_423;
// pid_control_V4/SUBSYSTEM_MODEL/Subscribe_HEAVE
SimulinkSubscriber<std_msgs::msg::Float64,SL_Bus_std_msgs_Float64> Sub_pid_control_V4_443;
// pid_control_V4/SUBSYSTEM_MODEL/Subscribe_RATE
SimulinkSubscriber<std_msgs::msg::Float64,SL_Bus_std_msgs_Float64> Sub_pid_control_V4_445;
// pid_control_V4/SUBSYSTEM_MODEL/Subscribe_TURBULENCIA
SimulinkSubscriber<std_msgs::msg::Bool,SL_Bus_std_msgs_Bool> Sub_pid_control_V4_417;
// pid_control_V4/Subscribe-ALTURA
SimulinkSubscriber<std_msgs::msg::Float64,SL_Bus_std_msgs_Float64> Sub_pid_control_V4_435;
// pid_control_V4/Subscribe-YAW
SimulinkSubscriber<std_msgs::msg::Float64,SL_Bus_std_msgs_Float64> Sub_pid_control_V4_377;
// pid_control_V4/Call Service
SimulinkServiceCaller<gazebo_msgs::srv::SetEntityState,SL_Bus_gazebo_msgs_SetEntityStateRequest,SL_Bus_gazebo_msgs_SetEntityStateResponse> ServCall_pid_control_V4_326;
