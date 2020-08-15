/**
 * This sample program balances a two-wheeled Segway type robot such as Gyroboy in EV3 core set.
 *
 * References:
 * http://www.hitechnic.com/blog/gyro-sensor/htway/
 * http://www.cs.bgu.ac.il/~ami/teaching/Lejos-2013/classes/src/lejos/robotics/navigation/Segoway.java
 */

#include "ev3api.h"
#include "app.h"

#include "ros.h"
#include "std_msgs/String.h"

#include <stdio.h>
#include <string>
#include <iostream>
#include <sstream>
#include <string.h>
#include <stdlib.h>

using namespace std;


#ifdef __cplusplus
extern "C" {
#endif
//dark
//#define white 78
//#define black 20
//bright
#define white 80
#define black 20

/**
 * Define the connection ports of the gyro sensor and motors.
 * By default, the Gyro Boy robot uses the following ports:
 * Gyro sensor: Port 2
 * Left motor:  Port A
 * Right motor: Port D
 */
const static int gyro_sensor = EV3_PORT_2;
const static motor_port_t left_motor = EV3_PORT_A;
const static motor_port_t right_motor = EV3_PORT_B;
const static motor_port_t arm_motor = EV3_PORT_C;

typedef enum {
	CLOSSING_UP = 0,
	CLOSSING_DOWN,
} ClossingControlType;

static ClossingControlType clossing_control;

void usr_task1(intptr_t unused)
{
	int argc = 0;
	char *argv = NULL;
	int counter = 0;
	ros::init(argc,argv,"clossing_controller");
	ros::NodeHandle n;

	syslog(LOG_NOTICE,"========Activate user task1========");

    // Configure motors
    ev3_motor_config(arm_motor, LARGE_MOTOR);

    syslog(LOG_NOTICE, "#### Crossing Motor start");
    while(1) {
        tslp_tsk(100); /* 100msec */
        if (clossing_control == CLOSSING_UP) {
            ev3_motor_set_power(arm_motor, -10);
            if (counter > 25) {
            	counter = 0;
            	clossing_control = CLOSSING_DOWN;
            }
        }
        else {
            ev3_motor_set_power(arm_motor, 10);
            if (counter > 25) {
            	counter = 0;
            	clossing_control = CLOSSING_UP;
            }
        }
        counter++;
    }
    return;
}
static char str_buf[1024];

static void topic_publish(ros::Publisher &pub, int value)
{
	std_msgs::String str;
	sprintf(str_buf, "v:%d", value);
	str.data = string(str_buf);
	pub.publish(str);
	return;
}

void usr_task2(intptr_t unused)
{
	syslog(LOG_NOTICE,"========Activate user task2========");
	int argc = 0;
	char *argv = NULL;

	ros::init(argc,argv,"crossing_sensor");
	ros::NodeHandle n;
	ros::Rate loop_rate(10); //100msec

	ros::Publisher pub = n.advertise<std_msgs::String>("crossing_alarm", 1);

	while (1) {
		topic_publish(pub, clossing_control);

		loop_rate.sleep();
	}
	return;
}

#ifdef __cplusplus
}
#endif
