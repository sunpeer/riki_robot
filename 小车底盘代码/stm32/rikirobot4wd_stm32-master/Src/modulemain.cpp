
#include <stdio.h>
#include "hardwareserial.h"
#include "gy85.h"
#include "motor.h"
#include "encoder.h"
#include "led.h"
#include "battery.h"
#include "servo.h"

// Test Driver Module , Please ROS set 0 
#define SERIAL 		1
#define IMU 		0
#define LED 		1
#define BATTERY		0
#define SERVO		0
#define MOTOR		1

// Test ROS, Please Driver Module set 0
#define ROS			0


int main(void) 
{
	SystemInit();
	initialise();

#if SERIAL // test driver module ,please set 1
	HardwareSerial Serial(SERIAL1);
	Serial.begin(9600);
#endif

#if IMU
	Vector gyros, acc, mag; 
	Gy85 imu;
	imu.init();
#endif

#if LED
	bool sled = true;
	Led led;
	led.init();
#endif

#if BATTERY	
	float bat;
	Battery battery(25, 10.6, 12.6);
	battery.init();
#endif

#if SERVO
	Servo servo(SERVO1);
	servo.init();
#endif

#if MOTOR
	//Motor motor1(MOTOR1, 254, 575); //left back
	//Motor motor2(MOTOR2, 254, 575);
	//Motor motor3(MOTOR3, 254, 575);
	Motor motor4(MOTOR4, 254, 575);
	//motor1.init();
	//motor2.init();
	//motor3.init();
	motor4.init();
	//encoder_init(ENCODER1);
	//encoder_init(ENCODER2);
	//encoder_init(ENCODER3);
	encoder_init(ENCODER4);
	//motor1.spin(100);
	//motor2.spin(100);
	//motor3.spin(100);
	motor4.spin(100);

#endif

	while(1){
	    #if IMU
			if(imu.check_gyroscope()){
				gyros = imu.measure_gyroscope();
				Serial.print("gyros x: %f, y : %f, z: %f \r\n", gyros.x, gyros.y, gyros.z);
			}
			if(imu.check_accelerometer()){
				acc = imu.measure_acceleration();
				Serial.print("acceleration x: %f, y : %f, z: %f \r\n", acc.x, acc.y, acc.z);
			}
			if(imu.check_magnetometer()){
				mag = imu.measure_magnetometer();
				Serial.print("magnetometer x: %f, y : %f, z: %f \r\n", mag.x, mag.y, mag.z);
			}
		#endif
		#if BATTERY
			bat = battery.get_volt();
			Serial.print("current volt is : %f\r\n", bat);
		#endif
		#if LED
			if(sled){
				led.on_off(ON);
				sled = false;
			}else {
				led.on_off(OFF);
				sled = true;
			}
		#endif
		#if SERVO
			servo.pos(90);
			delay(1000);
			servo.pos(180);
		#endif

		#if MOTOR
			//Serial.print("encoder1 cnt: %ld\r\n", en_pos1);
			//Serial.print("encoder2 cnt: %ld\r\n", en_pos2);
			//Serial.print("encoder3 cnt: %ld\r\n", en_pos3);
			Serial.print("encoder4 cnt: %ld\r\n", en_pos4);
		#endif

		delay(1000);
	}

}
