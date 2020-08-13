/**
 * This sample program balances a two-wheeled Segway type robot such as Gyroboy in EV3 core set.
 *
 * References:
 * http://www.hitechnic.com/blog/gyro-sensor/htway/
 * http://www.cs.bgu.ac.il/~ami/teaching/Lejos-2013/classes/src/lejos/robotics/navigation/Segoway.java
 */

#include "ev3api.h"
#include "app.h"


#ifdef __cplusplus
extern "C" {
#endif

/**
 * Define the connection ports of the gyro sensor and motors.
 * By default, the Gyro Boy robot uses the following ports:
 * Gyro sensor: Port 2
 * Left motor:  Port A
 * Right motor: Port D
 */
const static int gyro_sensor = EV3_PORT_2;
const static int left_motor = EV3_PORT_A;
const static int right_motor = EV3_PORT_B;


void usr_task1(intptr_t unused)
{
    ev3_sensor_config(EV3_PORT_1, COLOR_SENSOR);

    // Configure motors
    ev3_motor_config(left_motor, LARGE_MOTOR);
    ev3_motor_config(right_motor, LARGE_MOTOR);

#if 0
    LogDataType log_data;
    int i = 0;
#endif
    syslog(LOG_NOTICE, "#### motor control start");
    while(1) {

    /**
     * PID controller
     */

//dark
//#define white 78
//#define black 20
//bright
#define white 100
#define black 50
        static float lasterror = 0, integral = 0;
        static float midpoint = (white - black) / 2 + black;
        {
            float error = midpoint - ev3_color_sensor_get_reflect(EV3_PORT_1);
            //integral = error + integral * 0.5;
            //float steer = 0.07 * error + 0.3 * integral + 1 * (error - lasterror);
            integral = error + integral * 0.3;
            float steer = 0.6 * error + 0.3 * integral + 1 * (error - lasterror);
            ev3_motor_steer(left_motor, right_motor, 10, steer);
            lasterror = error;
        }
        tslp_tsk(20); /* 100msec */

    }
    return;
}

void usr_task2(intptr_t unused)
{
	return;
}

#ifdef __cplusplus
}
#endif
