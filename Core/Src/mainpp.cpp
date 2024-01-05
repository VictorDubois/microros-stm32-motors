/*
 * main.cpp
 *
 *  Created on: 2018/01/17
 *      Author: yoneken
 */
#include <mainpp.h>
#include <constants.h>
#include <math.h>

extern "C" {
	#include "main.h"
}
//#include <tf/tf.h>

/*ros::Subscriber<geometry_msgs__msg__Twist> twist_sub("cmd_vel", cmd_vel_cb);
ros::Subscriber<krabi_msgs__msg__MotorsParameters> parameters_sub("motors_parameters", parameters_cb);
ros::Subscriber<std_msgs__msg__Bool> enable_sub("enable_motor", enable_motor_cb);
ros::Subscriber<krabi_msgs__msg__MotorsCmd> motors_cmd_sub("motors_cmd", motors_cmd_cb);*/

void motors_cmd_cb(const void* motors_cmd_msg_void)
{
	// Cast received message to used type
	const krabi_msgs__msg__MotorsCmd * motors_cmd_msg = (const krabi_msgs__msg__MotorsCmd *)motors_cmd_msg_void;
	if (motors_cmd_msg->reset_encoders)
	{
		MotorBoard::set_odom(0, 0, 0);
	}

	MotorBoard::getDCMotor().set_enable_motors(motors_cmd_msg->enable_motors);

	if (!motors_cmd_msg->enable_motors) {
		MotorBoard::getDCMotor().resetMotors();
		return;
	}

	if (motors_cmd_msg->override_pwm)
	{
		MotorBoard::getDCMotor().override_PWM(motors_cmd_msg->pwm_override_left, motors_cmd_msg->pwm_override_right);
	}
	else
	{
		MotorBoard::getDCMotor().stop_pwm_override();
	}
}

/*void set_odom_alone_cb(const krabi_msgs__srv__SetOdom_Request &req, krabi_msgs__srv__SetOdom_Response &res)
{
	MotorBoard::set_odom(req.x, req.y, req.theta);
	res.success = true;
}*/

//ros::ServiceServer<krabi_msgs::SetOdomRequest, krabi_msgs::SetOdomResponse> set_odom_srv("set_odom", set_odom_alone_cb);

void parameters_cb(const void* a_parameters_void)
{
	krabi_msgs__msg__MotorsParameters * a_parameters = (krabi_msgs__msg__MotorsParameters*) a_parameters_void;
	MotorBoard::getDCMotor().set_max_current(a_parameters->max_current);
	MotorBoard::getDCMotor().set_max_current(a_parameters->max_current_left, a_parameters->max_current_right);
}

void cmd_vel_cb(const void* twist_void)
{
	geometry_msgs__msg__Twist* twist = (geometry_msgs__msg__Twist*) twist_void;
	MotorBoard::getDCMotor().set_speed_order(twist->linear.x, -twist->angular.z);
	//asserv_msg.max_current_left = twist.linear.x;

}

void enable_motor_cb(const void* enable_void)
{
	std_msgs__msg__Bool * enable = (std_msgs__msg__Bool *)enable_void;
	if(!enable->data) {
		MotorBoard::getDCMotor().resetMotors();
	}
}
/*
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	MotorBoard::getNodeHandle().getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	MotorBoard::getNodeHandle().getHardware()->reset_rbuf();
}*/
/*
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
	if (htim->Instance == TIM1) {
	    HAL_IncTick();
	  }
	if (htim->Instance == TIM15) {
		MotorBoard::getDCMotor().update();

	}
	if (htim->Instance == TIM7) {
	}
}*/

//ros::NodeHandle MotorBoard::nh;
// @todo comprendre pourquoi c'est un conflit?
DCMotorHardware MotorBoard::motorsHardware;
DCMotor MotorBoard::motors;
MCP3002 MotorBoard::currentReader;
volatile long long MotorBoard::message_counter = 0;

float MotorBoard::X = 0;
float MotorBoard::Y = 0;
float MotorBoard::theta_offset = 0;

void MotorBoard::set_odom(float a_x, float a_y, float a_theta)
{
	X = a_x;
	Y = a_y;
	int16_t encoder_left = motors.get_encoder_ticks(M_L);
	int16_t encoder_right = motors.get_encoder_ticks(M_R);

	float current_theta = get_orientation_float(encoder_left, encoder_right);
	theta_offset = a_theta - current_theta;
}

MotorBoard::MotorBoard(TIM_HandleTypeDef* a_motorTimHandler)//:Node("STM32")
{

	while(false)
	{
		HAL_GPIO_WritePin(DIR_B_GPIO_Port, DIR_B_Pin, static_cast<GPIO_PinState>(bool(int(HAL_GetTick()/1000)%2))); // Turn On/OFF LED
		HAL_Delay(100);
	}

	HAL_Delay(1);//3000, 4000, 5000 works
	//1, 500, 2000 does not work

	motorsHardware = DCMotorHardware(TIM1, TIM2, a_motorTimHandler, TIM_CHANNEL_4, a_motorTimHandler, TIM_CHANNEL_1);
	currentReader = MCP3002();
	motors = DCMotor(&motorsHardware, &currentReader);

	motors.set_max_acceleration(millimetersToTicks(9810));//mm/s/s
	motors.set_max_speed(millimetersToTicks(2000));//mm/s (=1.9rad/s)

	set_odom(0, 0, 0);




    allocator = rcl_get_default_allocator();

    //create init_options
    rclc_support_init(&support, 0, NULL, &allocator);

    // create node
    rclc_node_init_default(&node, "cubemx_node", "", &support);


	rclc_publisher_init_default(
		&encoders_pub,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(krabi_msgs, msg, Encoders),
		"encoders");
	rclc_publisher_init_default(
			&motors_pub,
			&node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(krabi_msgs, msg, Motors),
			"motors");
	/*rclc_publisher_init_default(
			&odom_light_pub,
			&node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(krabi_msgs, msg, OdomLight),
			"odom_light");*/
	rclc_publisher_init_default(
			&odom_lighter_pub,
			&node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(krabi_msgs, msg, OdomLighter),
			"odom_lighter");
	/*rclc_publisher_init_default(
				&asserv_pub,
				&node,
				ROSIDL_GET_MSG_TYPE_SUPPORT(krabi_msgs, msg, MotorsParameters),
				"asserv");*/

	rcl_ret_t rc;
	rcl_subscription_t twist_sub;
	rcl_subscription_t parameters_sub;
	rcl_subscription_t enable_sub;
	rcl_subscription_t motors_cmd_sub;

	rc = rclc_subscription_init_default(
	  &twist_sub, &node,
	  ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel");
	rc = rclc_subscription_init_default(
		  &parameters_sub, &node,
		  ROSIDL_GET_MSG_TYPE_SUPPORT(krabi_msgs, msg, MotorsParameters), "motors_parameters");
	rc = rclc_subscription_init_default(
		  &enable_sub, &node,
		  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "enable_motor");
	rc = rclc_subscription_init_default(
		  &motors_cmd_sub, &node,
		  ROSIDL_GET_MSG_TYPE_SUPPORT(krabi_msgs, msg, MotorsCmd), "motors_cmd");


	// Callbacks
	geometry_msgs__msg__Twist twist_msg;
	krabi_msgs__msg__MotorsParameters motors_parameters_msg;
	std_msgs__msg__Bool enable_msg;
	krabi_msgs__msg__MotorsCmd motors_cmd_msg;

	rc = rclc_executor_add_subscription(
	  &executor, &twist_sub, &twist_msg,
	  &cmd_vel_cb, ON_NEW_DATA);
	rc = rclc_executor_add_subscription(
		  &executor, &parameters_sub, &motors_parameters_msg,
		  &parameters_cb, ON_NEW_DATA);
	rc = rclc_executor_add_subscription(
		  &executor, &enable_sub, &enable_msg,
		  &enable_motor_cb, ON_NEW_DATA);
	rc = rclc_executor_add_subscription(
		  &executor, &motors_cmd_sub, &motors_cmd_msg,
		  &motors_cmd_cb, ON_NEW_DATA);


	// Services
	/*rcl_service_t set_odom_srv;
	rc = rclc_service_init_default(
	  &set_odom_srv, &node,
	  ROSIDL_GET_SRV_TYPE_SUPPORT(krabi_msgs, srv, SetOdom), "set_odom");*/

	/*ros::Subscriber<geometry_msgs__msg__Twist> twist_sub("cmd_vel", cmd_vel_cb);
	ros::Subscriber<krabi_msgs__msg__MotorsParameters> parameters_sub("motors_parameters", parameters_cb);
	ros::Subscriber<std_msgs__msg__Bool> enable_sub("enable_motor", enable_motor_cb);
	ros::Subscriber<krabi_msgs__msg__MotorsCmd> motors_cmd_sub("motors_cmd", motors_cmd_cb);*/

	/*nh.initNode();
	HAL_Delay(100);

	//nh.advertise(odom_pub);
	nh.advertise(asserv_pub);
	nh.advertise(odom_light_pub);
	nh.advertise(odom_lighter_pub);
	nh.advertise(encoders_pub);
	nh.advertise(motors_pub);
	nh.subscribe(twist_sub);
	nh.subscribe(parameters_sub);
	nh.subscribe(enable_sub);
	nh.subscribe(motors_cmd_sub);

	nh.advertiseService(set_odom_srv);*/

	HAL_Delay(100);


	// Check connection before continuing
	/*
	uint8_t reinit = 1;
	while (!nh.connected())
	{
	    nh.spinOnce();
		if (HAL_GetTick()/1000 > reinit){
			MotorBoard::getNodeHandle().getHardware()->reset_rbuf();
			reinit++;
		}
	    HAL_Delay(MS_BETWEEN_UPDATES);
	}*/


}
MotorBoard::MotorBoard()// : Node("toto")
{}
MotorBoard::~MotorBoard() {}

/*rcl_node_t& MotorBoard::getNodeHandle(void) {
	return node;
}*/

DCMotor& MotorBoard::getDCMotor(void) {
	return motors;
}

/*
    Return the Robot's orientation, in degrees, with respect to the last encoder reset.
*/
float get_orientation_float(int32_t encoder1, int32_t encoder2)
{


    float absolute_orientation = fmod((encoder2 - encoder1) / TICKS_PER_DEG, 360);

    if (absolute_orientation >= 0)
        return (absolute_orientation);
    else
        return (360.f + absolute_orientation); // reminder: abs_ori is < 0 here
}

int32_t fixOverflowAngular(int16_t a_after, int32_t before)
{
	int32_t after = a_after;
    while (after - before > TICKS_half_OVERFLOW)
    {
        // printf("before (%ld) - after (%ld) > TICKS_half_OVERFLOW (%d). Returning %ld\n\n\n",
        // before, after, TICKS_half_OVERFLOW, after - before - TICKS_OVERFLOW);
        after -= TICKS_OVERFLOW;
    }
    while (after - before < -TICKS_half_OVERFLOW)
    {
        // printf("after (%ld) - before (%ld) < -TICKS_half_OVERFLOW (%d). Returning %ld\n\n\n",
        // after, before, -TICKS_half_OVERFLOW, after - before + TICKS_OVERFLOW);
        after += TICKS_OVERFLOW;
    }
    return after;
}

int fixOverflow(long after, long before)
{
    if (after - before > TICKS_half_OVERFLOW)
    {
        // printf("before (%ld) - after (%ld) > TICKS_half_OVERFLOW (%d). Returning %ld\n\n\n",
        // before, after, TICKS_half_OVERFLOW, after - before - TICKS_OVERFLOW);
        return after - before - TICKS_OVERFLOW;
    }
    if (after - before < -TICKS_half_OVERFLOW)
    {
        // printf("after (%ld) - before (%ld) < -TICKS_half_OVERFLOW (%d). Returning %ld\n\n\n",
        // after, before, -TICKS_half_OVERFLOW, after - before + TICKS_OVERFLOW);
        return after - before + TICKS_OVERFLOW;
    }
    return after - before;
}

constexpr float ticksToMillimeters(int32_t ticks)
{
	return (DIST_PER_REVOLUTION * (float)ticks / TICKS_PER_REVOLUTION);
}


constexpr int32_t millimetersToTicks(float millimeters)
{
	return static_cast<int32_t>(millimeters * TICKS_PER_REVOLUTION/DIST_PER_REVOLUTION);
}

/*
        Given current value of both encoders
        return the linear dist by approximating it as the average of both wheels' linear distances.
        Static variables are used to keep last value of encoders.
*/
float MotorBoard::compute_linear_dist(const long encoder1, const long encoder2)
{
    float dist1, dist2, dist;
    int diff_encoder1, diff_encoder2;

    // Compute difference in nb of ticks between last measurements and now
    diff_encoder1 = fixOverflow(encoder1, last_encoder_left);
    diff_encoder2 = fixOverflow(encoder2, last_encoder_right);

    // Compute each wheel's dist and approximate linear dist as their average
    dist1 = ticksToMillimeters(diff_encoder1);
    dist2 = ticksToMillimeters(diff_encoder2);
    dist = (dist1 + dist2) / 2.0f;

    if (fabsf(dist) > 500.)
    {
        //printf("\n/!\\ HIGH SPEED DETECTED: %.2f /!\\\n\n", dist);
        // exit(4);
    }

    // Update static variables' values (current encoder values become old ones)
    last_encoder_left = encoder1;
    last_encoder_right = encoder2;

    // Return the computed linear dist
    return dist / 1000.f; // convert to meters
}

void MotorBoard::update_inputs() {
	//nh.spinOnce();
	rclc_executor_spin(&executor);
}

geometry_msgs__msg__Quaternion createQuaternionMsgFromYaw(double yaw)
{
  geometry_msgs__msg__Quaternion q;
  q.x = 0;
  q.y = 0;
  q.z = sin(0.5 * yaw);
  q.w = cos(0.5 * yaw);
  return q;
}

void MotorBoard::update() {
	// Todo check if connected
	/*if (!node.connected()){
		return;
	}*/

	rcl_ret_t ret;


	int16_t encoder_left = motors.get_encoder_ticks(M_L);
	int16_t encoder_right = motors.get_encoder_ticks(M_R);

	encoders_msg.encoder_left = encoder_left;
	encoders_msg.encoder_right = encoder_right;
	//encoders_msg.header.stamp = nh.now();

	ret = rcl_publish(&encoders_pub, &encoders_msg, NULL);

	int32_t right_speed = motors.get_speed(M_R);
	int32_t left_speed = motors.get_speed(M_L);

	float linear_dist = compute_linear_dist(encoder_left, encoder_right);

    // Compute difference in nb of ticks between last measurements and now
    int32_t fixed_angular_encoder_left = fixOverflowAngular(encoder_left, last_encoder_left_angular);
    int32_t fixed_angular_encoder_right = fixOverflowAngular(encoder_right, last_encoder_right_angular);
	last_encoder_left_angular = fixed_angular_encoder_left;
	float current_theta = get_orientation_float(fixed_angular_encoder_left, fixed_angular_encoder_right);
	last_encoder_right_angular = fixed_angular_encoder_right;
	current_theta += theta_offset;

	float current_theta_rad = current_theta * M_PI / 180.f;

	X += linear_dist * cos(current_theta_rad);
	Y += linear_dist * sin(current_theta_rad);


	//int32_t right_speed = motors.get_speed(M_R);
	//int32_t left_speed = motors.get_speed(M_L);

	/*odom_msg.header.frame_id = "odom";
	odom_msg.header.stamp = nh.now();
	odom_msg.header.seq = message_counter++;
	odom_msg.child_frame_id = "base_link";

	for (unsigned int i = 0; i < (sizeof(odom_msg.pose.covariance)/sizeof(*(odom_msg.pose.covariance))); i++){
		odom_msg.pose.covariance[i] = 0;
	}

	odom_msg.pose.pose.position.x = X;
	odom_msg.pose.pose.position.y = Y;
	odom_msg.pose.pose.position.z = 0;

	odom_msg.pose.pose.orientation = odom_quat;

	//odom_msg.pose.pose.orientation.x = 0;
	//odom_msg.pose.pose.orientation.y = 0;
	//odom_msg.pose.pose.orientation.z = current_theta_rad;

	for (unsigned int i = 0; i < (sizeof(odom_msg.twist.covariance)/sizeof(*(odom_msg.twist.covariance))); i++){
		odom_msg.twist.covariance[i] = 0;
	}

	odom_msg.twist.twist.linear.x = ticksToMillimeters((left_speed+right_speed)/2)/1000.f;
	odom_msg.twist.twist.linear.y = 0;
	odom_msg.twist.twist.linear.z = 0;

	odom_msg.twist.twist.angular.x = 0;
	odom_msg.twist.twist.angular.y = 0;


	odom_msg.twist.twist.angular.z = odometry->getAngularSpeed();
	odom_pub.publish(&odom_msg);*/

	//odom_lighter_msg.header.stamp = nh.now();
	//odom_lighter_msg.header.seq = message_counter++;
	//odom_lighter_msg.header.frame_id = "";//Todo how to publish this?
	odom_lighter_msg.pose_x = X;
	odom_lighter_msg.pose_y = Y;
	odom_lighter_msg.angle_rz = current_theta_rad;
	odom_lighter_msg.speed_vx = ticksToMillimeters((left_speed+right_speed)/2)/1000.f;
	odom_lighter_msg.speed_wz = ((right_speed - left_speed)/TICKS_PER_DEG)*M_PI/180; // rad/s
	ret = rcl_publish(&odom_lighter_pub, &odom_lighter_msg, NULL);

	/*if (false && message_counter%100 == 0)
	{
		//odom_light_msg.header.stamp = nh.now();
		//odom_light_msg.header.seq = message_counter++;
		//odom_light_msg.header.frame_id = "odom_light_newer";//Todo how to publish this?
		odom_light_msg.pose.position.x = X;
		odom_light_msg.pose.position.y = Y;
		odom_light_msg.pose.position.z = MotorBoard::getDCMotor().get_dt();

		odom_light_msg.pose.orientation = createQuaternionMsgFromYaw(current_theta_rad);


		odom_light_msg.speed.linear.x = ticksToMillimeters((left_speed+right_speed)/2)/1000.f;
		odom_light_msg.speed.linear.y = ticksToMillimeters(MotorBoard::getDCMotor().get_linear_speed_order())/1000.f;
		odom_light_msg.speed.linear.z = MotorBoard::getDCMotor().get_linear_error_integ()/1000.f;

		odom_light_msg.speed.angular.x = ticksToMillimeters(MotorBoard::getDCMotor().get_voltage(M_L));
		odom_light_msg.speed.angular.y = ticksToMillimeters(MotorBoard::getDCMotor().get_voltage(M_R));
		odom_light_msg.speed.angular.z = MotorBoard::getDCMotor().get_linear_error()/1000.f;//ticksToMillimeters((left_speed-right_speed)/2)/1000.f; // C'est complètement faux, non ?
		odom_light_msg.current_motor_left = motors.get_accumulated_current(M_L);
		odom_light_msg.current_motor_right = motors.get_accumulated_current(M_R);




		ret = rcl_publish(&odom_light_pub, &odom_light_msg, NULL);


		motors_msg.current_left = motors.get_current(M_L);
		motors_msg.current_right = motors.get_current(M_R);

		motors_msg.current_left_accumulated = motors.get_accumulated_current(M_L);
		motors_msg.current_right_accumulated = motors.get_accumulated_current(M_R);

		ret = rcl_publish(&motors_pub, &motors_msg, NULL);


	}*/

	  rclc_executor_spin(&executor);
}

void setup()
{
  //pinMode(BRAKE, OUTPUT);
  //digitalWrite(BRAKE, LOW);

  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);//LED
  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);//BRAKE
  HAL_GPIO_WritePin(DIR_A_GPIO_Port, DIR_A_Pin, GPIO_PIN_RESET);//DIR_A
  HAL_GPIO_WritePin(DIR_B_GPIO_Port, DIR_B_Pin, GPIO_PIN_RESET);//DIR_B

	/*HAL_GPIO_WritePin(DIR_B_GPIO_Port, DIR_B_Pin, GPIO_PIN_SET); // Turn On LED
	HAL_Delay(1000);
	HAL_GPIO_WritePin(DIR_B_GPIO_Port, DIR_B_Pin, GPIO_PIN_RESET); // Turn Off LED
	HAL_Delay(1000);*/


}


void loop(TIM_HandleTypeDef* a_motorTimHandler, TIM_HandleTypeDef* a_loopTimHandler)
{
	HAL_GPIO_WritePin(DIR_B_GPIO_Port, DIR_B_Pin, GPIO_PIN_SET); // Turn On LED
	MotorBoard myboard = MotorBoard(a_motorTimHandler);
	HAL_TIM_Base_Start_IT(a_loopTimHandler);
	int32_t waiting_time = 0;
	while(true) {
		myboard.update();

		for(int ii = 0; ii< 10 ; ii++){
			myboard.update_inputs();


			// Debug messages

			/*int32_t right_speed = MotorBoard::getDCMotor().get_speed(M_R);
			int32_t left_speed = MotorBoard::getDCMotor().get_speed(M_L);
			float current_speed = ticksToMillimeters((left_speed+right_speed)/2)/1000.f;
			asserv_msg.max_current = current_speed;
			asserv_msg.max_current_right = static_cast<float>(MotorBoard::getDCMotor().get_voltage(M_R))/500;
			asserv_msg.max_current_left = static_cast<float>(MotorBoard::getDCMotor().get_linear_speed_order())/500;


			asserv_pub.publish(&asserv_msg);*/



			HAL_Delay(waiting_time);
		}
	}
}
