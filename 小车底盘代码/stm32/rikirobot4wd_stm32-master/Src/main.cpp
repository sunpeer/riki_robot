/*
 * A skeleton main.c
 * Add your own code!
 */

#include <stdio.h>
#include "hardwareserial.h"
#include "gy85.h"
#include "motor.h"
#include "encoder.h"
#include "led.h"
#include "PID.h"
#include "Kinematics.h"
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <riki_msgs/PID.h>
#include <riki_msgs/Velocities.h>
#include <ros_arduino_msgs/RawImu.h>
#include <geometry_msgs/Vector3.h>
#include <ros/time.h>

int Motor::counts_per_rev_ = COUNTS_PER_REV;

double required_angular_vel = 0;
double required_linear_vel = 0;
uint32_t previous_command_time = 0;

bool is_first = true;
bool en_call = true;

extern "C"{ void * __dso_handle = 0 ;}

PID motor1_pid(-255, 255, K_P, K_I, K_D);
PID motor2_pid(-255, 255, K_P, K_I, K_D);
PID motor3_pid(-255, 255, K_P, K_I, K_D);
PID motor4_pid(-255, 255, K_P, K_I, K_D);

static void callConstructors()
{
	//Start and end points of the constructor list,
	// defined by the linker script.
	extern void (*__init_array_start)();
	extern void (*__init_array_end)();

	// Call each function in the list.
	// We have to take the address of the symbols, as __init_array_start *is*
	// the first function pointer, not the address of it.
	for (void (**p)() = &__init_array_start; p < &__init_array_end; ++p) {
		(*p)();
	}
}

void pid_callback( const riki_msgs::PID& pid)
{
	motor1_pid.updateConstants(pid.p, pid.i, pid.d);
	motor2_pid.updateConstants(pid.p, pid.i, pid.d);
	motor3_pid.updateConstants(pid.p, pid.i, pid.d);
	motor4_pid.updateConstants(pid.p, pid.i, pid.d);

}

void command_callback( const geometry_msgs::Twist& cmd_msg)
{
	required_linear_vel = cmd_msg.linear.x;
	required_angular_vel = cmd_msg.angular.z;

	previous_command_time = millis();
}

void move_base(Motor *motor1, Motor *motor2, Motor *motor3, Motor *motor4, Kinematics *kinematics)
{
  Kinematics::output req_rpm;
  //get the required rpm for each motor based on required velocities
  req_rpm = kinematics->getRPM(required_linear_vel, 0.0, required_angular_vel);

  //the required rpm is capped at -/+ MAX_RPM to prevent the PID from having too much error
  //the PWM value sent to the motor driver is the calculated PID based on required RPM vs measured RPM
  motor1->spin(motor1_pid.compute(constrain(req_rpm.motor3, -MAX_RPM, MAX_RPM), motor1->rpm));
  motor3->spin(motor3_pid.compute(constrain(req_rpm.motor2, -MAX_RPM, MAX_RPM), motor3->rpm));
  motor2->spin(motor2_pid.compute(constrain(req_rpm.motor4, -MAX_RPM, MAX_RPM), motor2->rpm));
  motor4->spin(motor4_pid.compute(constrain(req_rpm.motor1, -MAX_RPM, MAX_RPM), motor4->rpm));
}

void stop_base()
{
	required_linear_vel = 0;
	required_angular_vel = 0;
}

void publish_linear_velocity(Motor *motor1, Motor *motor2,  Motor *motor3, 
							Motor *motor4, Kinematics *kinematics, 
							riki_msgs::Velocities *raw_vel_msg, 
							ros::Publisher *raw_vel_pub)
{
  //update the current speed of each motor based on encoder's count
  motor1->updateSpeed(en_pos1);
  motor2->updateSpeed(en_pos2);
  motor3->updateSpeed(en_pos3);
  motor4->updateSpeed(en_pos4);

  Kinematics::velocities vel;
  //vel = kinematics->getVelocities(motor1->rpm, motor2->rpm, motor3->rpm, motor4->rpm);
  vel = kinematics->getVelocities(motor3->rpm, motor4->rpm, motor2->rpm, motor1->rpm);

  //fill in the object
  raw_vel_msg->linear_x = vel.linear_x;
  raw_vel_msg->linear_y = 0.0;
  raw_vel_msg->angular_z = vel.angular_z;

  //publish raw_vel_msg object to ROS
  raw_vel_pub->publish(raw_vel_msg);
}

void check_imu(Gy85 *imu, ros_arduino_msgs::RawImu *raw_imu_msg, ros::NodeHandle *nh)
{
	raw_imu_msg->gyroscope = imu->check_gyroscope();
	raw_imu_msg->accelerometer = imu->check_accelerometer();
	raw_imu_msg->magnetometer = imu->check_magnetometer();

	if (!raw_imu_msg->accelerometer){
		nh->logerror("Accelerometer NOT FOUND!");
	}

	if (!raw_imu_msg->gyroscope){
		nh->logerror("Gyroscope NOT FOUND!");
	}

	if (!raw_imu_msg->magnetometer){
		nh->logerror("Magnetometer NOT FOUND!");
	}

	is_first = false;
}

void publish_imu(Gy85 *imu, ros_arduino_msgs::RawImu *raw_imu_msg, ros::NodeHandle *nh, ros::Publisher *raw_imu_pub)
{
	//geometry_msgs::Vector3 acceler, gyro, mag;
	//this function publishes raw IMU reading
	raw_imu_msg->header.stamp = nh->now();
	raw_imu_msg->header.frame_id = "imu_link";
	//measure accelerometer
	if (raw_imu_msg->accelerometer){
		imu->measure_acceleration();
		raw_imu_msg->raw_linear_acceleration = imu->raw_acceleration;
	}

	//measure gyroscope
	if (raw_imu_msg->gyroscope){
		imu->measure_gyroscope();
		raw_imu_msg->raw_angular_velocity = imu->raw_rotation;
	}

	//measure magnetometer
	if (raw_imu_msg->magnetometer){
		imu->measure_magnetometer();
		raw_imu_msg->raw_magnetic_field = imu->raw_magnetic_field;
	}

	//publish raw_imu_msg object to ROS
	raw_imu_pub->publish(raw_imu_msg);

}

void print_debug(ros::NodeHandle *nh)
{
	char buffer[50];
	//sprintf (buffer, "Encoder Left Rear: %d , req_rpm: %d , pwm: %d, rpm: %d", en_pos1, m4req_rpm, m1pwm, m1rpm);
	//nh->loginfo(buffer);
	sprintf (buffer, "Encoder Left Rear: %d, Right Rear: %d", en_pos1, en_pos2);
	nh->loginfo(buffer);
	sprintf (buffer, "Encoder Right Front: %d, Left Front: %d", en_pos3, en_pos4);
	nh->loginfo(buffer);
}


int main(void) 
{
	uint32_t previous_debug_time = 0;
	uint32_t previous_imu_time = 0;
	uint32_t previous_control_time = 0;
	uint32_t publish_vel_time = 0;
	uint8_t call_cnt = 0;


	SystemInit();
	initialise();

	// motor init 
	// left side motor encoder
	//right side motor encoder
	Motor motor1(MOTOR1, 254, 575);
	Motor motor2(MOTOR2, 254, 575);
	Motor motor3(MOTOR3, 254, 575);
	Motor motor4(MOTOR4, 254, 575);
	motor1.init();
	motor2.init();	
	motor3.init();
	motor4.init();	

	encoder_init(ENCODER1);
	encoder_init(ENCODER2);
	encoder_init(ENCODER3);
	encoder_init(ENCODER4);

	Kinematics kinematics(MAX_RPM, WHEEL_DIAMETER, BASE_WIDTH, PWM_BITS);

	// imu init 
	Gy85  imu;
	imu.init();

	Led led;
	led.init();

	/** init ros **/ 
	ros::NodeHandle nh;
	nh.initNode();

	ros_arduino_msgs::RawImu raw_imu_msg;
	riki_msgs::Velocities raw_vel_msg;

	ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", command_callback);
	ros::Subscriber<riki_msgs::PID> pid_sub("pid", pid_callback);
	ros::Publisher raw_vel_pub("raw_vel", &raw_vel_msg);
	ros::Publisher raw_imu_pub("raw_imu", &raw_imu_msg);
	nh.subscribe(pid_sub);
	nh.subscribe(cmd_sub);
	nh.advertise(raw_vel_pub);
	nh.advertise(raw_imu_pub);
	while (!nh.connected()){
		nh.spinOnce();
	}
	nh.loginfo("Rikibase Connected!");

	led.on_off(ON);

	//delay(1000);

	while(1){
		if ((millis() - previous_control_time) >= (1000 / COMMAND_RATE)){
			if(en_call){
				if(call_cnt > 10){
					en_call = false;
					en_pos1 = 0;
					en_pos2 = 0;
					en_pos3 = 0;
					en_pos4 = 0;
					callConstructors();
				}
				call_cnt++;
			}

			move_base(&motor1, &motor2, &motor3, &motor4, &kinematics);
			previous_control_time = millis();
		}

		if ((millis() - previous_command_time) >= 400){
			stop_base();
		}

		if ((millis() - publish_vel_time) >= (1000 / VEL_PUBLISH_RATE)){
			publish_linear_velocity(&motor1, &motor2, &motor3, &motor4, &kinematics, &raw_vel_msg, &raw_vel_pub);
			publish_vel_time = millis();
		}

		if ((millis() - previous_imu_time) >= (1000 / IMU_PUBLISH_RATE)){
			//sanity check if the IMU exits
			if (is_first){
				check_imu(&imu, &raw_imu_msg, &nh);
			} else{
				//publish the IMU data
				publish_imu(&imu, &raw_imu_msg, &nh, &raw_imu_pub);
			}
			previous_imu_time = millis();
		}

		if(DEBUG){
			if ((millis() - previous_debug_time) >= (1000 / DEBUG_RATE)) {
				print_debug(&nh);
				previous_debug_time = millis();
			}
		}
		nh.spinOnce();
	}
}


