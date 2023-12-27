/*
 * mainpp.h
 *
 *  Created on: 2018/01/17
 *      Author: yoneken
 */

#ifndef MAINPP_H_
#define MAINPP_H_

#include <std_msgs/msg/bool.h>
#include <geometry_msgs/msg/twist.h>
#include <krabi_msgs/msg/motors.h>
#include <krabi_msgs/msg/odom_light.h>
#include <krabi_msgs/msg/odom_lighter.h>
#include <krabi_msgs/msg/encoders.h>
#include <krabi_msgs/msg/motors_parameters.h>
//#include <ros.h>
#include <std_msgs/msg/string.h>
#include <nav_msgs/msg/odometry.h>
#include <krabi_msgs/msg/motors_cmd.h>
#include <krabi_msgs/srv/set_odom.h>
#include <MCP3002.h>
#include "stm32f3xx_hal.h"
#include "DCMotor.h"


#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#define UPDATE_FREQ 10
#define MS_BETWEEN_UPDATES 1000/UPDATE_FREQ


//std_msgs::String str_msg;
//ros::Publisher chatter("chatter", &str_msg);
//nav_msgs::Odometry odom_msg;
//ros::Publisher odom_pub("odom", &odom_msg);
//ros::Publisher asserv_pub("asserv", &asserv_msg);

float get_orientation_float(int32_t encoder1, int32_t encoder2);
int fixOverflow(long after, long before);
int32_t fixOverflowAngular(int16_t after, int32_t before);
constexpr float ticksToMillimeters(int32_t ticks);
constexpr int32_t millimetersToTicks(float millimeters);

/*void motors_cmd_cb(const krabi_msgs::motors_cmd &motors_cmd_msg);
void cmd_vel_cb(const geometry_msgs::Twist& twist);
void parameters_cb(const krabi_msgs::motors_parameters& parameters);
void enable_motor_cb(const std_msgs::Bool& enable);*/
void motors_cmd_cb(const krabi_msgs__msg__MotorsCmd& motors_cmd_msg);
void cmd_vel_cb(const geometry_msgs__msg__Twist& twist);
void parameters_cb(const krabi_msgs__msg__MotorsParameters& parameters);
void enable_motor_cb(const std_msgs__msg__Bool& enable);

class MotorBoard //: public rclcpp::Node
{
public:
	MotorBoard(TIM_HandleTypeDef* motorTimHandler);
	MotorBoard();
	~MotorBoard();

	static rcl_node_t& getNodeHandle(void);
	static DCMotor& getDCMotor(void);
	static void set_odom(float a_x, float a_y, float a_theta);
	void set_odom_cb(const krabi_msgs__srv__SetOdom_Request& req, krabi_msgs__srv__SetOdom_Response &res);

	void update();
	void update_inputs();
private:
	//static ros::NodeHandle nh;
	static DCMotorHardware motorsHardware;
	static DCMotor motors;
	static MCP3002 currentReader;
	volatile long last_encoder_left = 0;
	volatile long last_encoder_right = 0;
	volatile int32_t last_encoder_left_angular = 0;
	volatile int32_t last_encoder_right_angular = 0;
	float compute_linear_dist(const long encoder_left, const long encoder_right);
	static float X;
	static float Y;
	static float theta_offset;
	volatile static long long message_counter;

	rcl_publisher_t encoders_pub;
	rcl_publisher_t motors_pub;
	rcl_publisher_t odom_light_pub;
	rcl_publisher_t odom_lighter_pub;
	rcl_publisher_t asserv_pub;

	krabi_msgs__msg__Encoders encoders_msg;
	krabi_msgs__msg__Motors motors_msg;
	krabi_msgs__msg__MotorsParameters asserv_msg;
	krabi_msgs__msg__OdomLight odom_light_msg;
	krabi_msgs__msg__OdomLighter odom_lighter_msg;

	rcl_publisher_t publisher;
	rclc_support_t support;
	rcl_allocator_t allocator;
	rcl_node_t node;
	rclc_executor_t executor;


	//TODO comprendre pourquoi ça ne marche pas
#ifdef __cplusplus
 extern "C" {
#endif


void setup();
void loop(TIM_HandleTypeDef* motorTimHandler, TIM_HandleTypeDef* loopTimHandler);

#ifdef __cplusplus
}
#endif
}

#endif /* MAINPP_H_ */
