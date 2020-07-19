/**
 * This sample program balances a two-wheeled Segway type robot such as Gyroboy in EV3 core set.
 *
 * References:
 * http://www.hitechnic.com/blog/gyro-sensor/htway/
 * http://www.cs.bgu.ac.il/~ami/teaching/Lejos-2013/classes/src/lejos/robotics/navigation/Segoway.java
 */

#include "ev3api.h"
#include "app.h"

#define DEBUG

#ifdef DEBUG
#define _debug(x) (x)
#else
#define _debug(x)
#endif

/**
 * Define the connection ports of the gyro sensor and motors.
 * By default, the Gyro Boy robot uses the following ports:
 * Gyro sensor: Port 2
 * Left motor:  Port A
 * Right motor: Port D
 */
const int gyro_sensor = EV3_PORT_2, left_motor = EV3_PORT_A, right_motor = EV3_PORT_B, arm_motor = EV3_PORT_C;

/**
 * Constants for the self-balance control algorithm.
 */
const float KSTEER=-0.25;
const float EMAOFFSET = 0.0005f, KGYROANGLE = 7.5f, KGYROSPEED = 1.15f, KPOS = 0.07f, KSPEED = 0.1f, KDRIVE = -0.02f;
const float WHEEL_DIAMETER = 5.6;
const uint32_t WAIT_TIME_MS = 5;
const uint32_t FALL_TIME_MS = 1000;
const float INIT_GYROANGLE = -0.25;
const float INIT_INTERVAL_TIME = 0.014;

/**
 * Constants for the self-balance control algorithm. (Gyroboy Version)
 */
//const float EMAOFFSET = 0.0005f, KGYROANGLE = 15.0f, KGYROSPEED = 0.8f, KPOS = 0.12f, KSPEED = 0.08f, KDRIVE = -0.01f;
//const float WHEEL_DIAMETER = 5.6;
//const uint32_t WAIT_TIME_MS = 1;
//const uint32_t FALL_TIME_MS = 1000;
//const float INIT_GYROANGLE = -0.25;
//const float INIT_INTERVAL_TIME = 0.014;

/**
 * Global variables used by the self-balance control algorithm.
 */
static int motor_diff, motor_diff_target;
static int loop_count, motor_control_drive, motor_control_steer;
static float gyro_offset, gyro_speed, gyro_angle, interval_time;
static float motor_pos, motor_speed;

/**
 * Calculate the initial gyro offset for calibration.
 */
static ER calibrate_gyro_sensor() {
	int i;
    int gMn = 1000, gMx = -100, gSum = 0;
    for (i = 0; i < 200; ++i) {
        int gyro = ev3_gyro_sensor_get_rate(gyro_sensor);
        gSum += gyro;
        if (gyro > gMx)
            gMx = gyro;
        if (gyro < gMn)
            gMn = gyro;
        tslp_tsk(4);
    }
    if(!(gMx - gMn < 2)) { // TODO: recheck the condition, '!(gMx - gMn < 2)' or '(gMx - gMn < 2)'
        gyro_offset = gSum / 200.0f;
        return E_OK;
    } else {
        return E_OBJ;
    }
}

/**
 * Calculate the average interval time of the main loop for the self-balance control algorithm.
 * Units: seconds
 */
static void update_interval_time() {
    static SYSTIM start_time;

    if(loop_count++ == 0) { // Interval time for the first time (use 6ms as a magic number)
        //interval_time = 0.006;
        interval_time = INIT_INTERVAL_TIME;
        ER ercd = get_tim(&start_time);
        assert(ercd == E_OK);
    } else {
        SYSTIM now;
        ER ercd = get_tim(&now);
        assert(ercd == E_OK);
        interval_time = ((float)(now - start_time)) / loop_count / 1000;
    }
}

/**
 * Update data of the gyro sensor.
 * gyro_offset: the offset for calibration.
 * gyro_speed: the speed of the gyro sensor after calibration.
 * gyro_angle: the angle of the robot.
 */
static void update_gyro_data() {
    int gyro = ev3_gyro_sensor_get_rate(gyro_sensor);
    gyro_offset = EMAOFFSET * gyro + (1 - EMAOFFSET) * gyro_offset;
    gyro_speed = gyro - gyro_offset;
    gyro_angle += gyro_speed * interval_time;
}

/**
 * Update data of the motors
 */
static void update_motor_data() {
    static int32_t prev_motor_cnt_sum, motor_cnt_deltas[4];

    if(loop_count == 1) { // Reset
        motor_pos = 0;
        prev_motor_cnt_sum = 0;
        motor_cnt_deltas[0] = motor_cnt_deltas[1] = motor_cnt_deltas[2] = motor_cnt_deltas[3] = 0;
    }

    int32_t left_cnt = ev3_motor_get_counts(left_motor);
    int32_t right_cnt = ev3_motor_get_counts(right_motor);
    int32_t motor_cnt_sum = left_cnt + right_cnt;
    motor_diff = right_cnt - left_cnt; // TODO: with diff
    int32_t motor_cnt_delta = motor_cnt_sum - prev_motor_cnt_sum;

    prev_motor_cnt_sum = motor_cnt_sum;
    motor_pos += motor_cnt_delta;
    motor_cnt_deltas[loop_count % 4] = motor_cnt_delta;
    motor_speed = (motor_cnt_deltas[0] + motor_cnt_deltas[1] + motor_cnt_deltas[2] + motor_cnt_deltas[3]) / 4.0f / interval_time;
}

/**
 * Control the power to keep balance.
 * Return false when the robot has fallen.
 */
static bool_t keep_balance() {
    static SYSTIM ok_time;

    if(loop_count == 1) // Reset ok_time
        get_tim(&ok_time);

    float ratio_wheel = WHEEL_DIAMETER / 5.6;

    // Apply the drive control value to the motor position to get robot to move.
    motor_pos -= motor_control_drive * interval_time;

    // This is the main balancing equation
    int power = (int)((KGYROSPEED * gyro_speed +               // Deg/Sec from Gyro sensor
                       KGYROANGLE * gyro_angle) / ratio_wheel + // Deg from integral of gyro
                       KPOS       * motor_pos +                // From MotorRotaionCount of both motors
                       KDRIVE     * motor_control_drive +       // To improve start/stop performance
                       KSPEED     * motor_speed);              // Motor speed in Deg/Sec

    // Check fallen
    SYSTIM time;
    get_tim(&time);
    if(power > -100 && power < 100)
        ok_time = time;
    else if(time - ok_time >= FALL_TIME_MS)
        return false;

    // Steering control
    motor_diff_target += motor_control_steer * interval_time;

    int left_power, right_power;

    // TODO: support steering and motor_control_drive
    int power_steer = (int)(KSTEER * (motor_diff_target - motor_diff));
    left_power = power + power_steer;
    right_power = power - power_steer;
    if(left_power > 100)
        left_power = 100;
    if(left_power < -100)
        left_power = -100;
    if(right_power > 100)
        right_power = 100;
    if(right_power < -100)
        right_power = -100;

    ev3_motor_set_power(left_motor, (int)left_power);
    ev3_motor_set_power(right_motor, (int)right_power);

    return true;
}

void balance_task(intptr_t unused) {
    ER ercd;
    int i;

    /**
     * Reset
     */
    loop_count = 0;
    motor_control_drive = 0;
    ev3_motor_reset_counts(left_motor);
    ev3_motor_reset_counts(right_motor);
    //TODO: reset the gyro sensor
    ev3_gyro_sensor_reset(gyro_sensor);

    /**
     * Calibrate the gyro sensor and set the led to green if succeeded.
     */
    _debug(syslog(LOG_NOTICE, "Start calibration of the gyro sensor."));
    for(i = 10; i > 0; --i) { // Max retries: 10 times.
        ercd = calibrate_gyro_sensor();
        if(ercd == E_OK) break;
        if(i != 1)
            syslog(LOG_ERROR, "Calibration failed, retry.");
        else {
            syslog(LOG_ERROR, "Max retries for calibration exceeded, exit.");
            return;
        }
    }
    _debug(syslog(LOG_INFO, "Calibration succeed, offset is %de-3.", (int)(gyro_offset * 1000)));
    gyro_angle = INIT_GYROANGLE;
    ev3_led_set_color(LED_GREEN);

    /**
     * Main loop for the self-balance control algorithm
     */
    while(1) {
        // Update the interval time
        update_interval_time();

        // Update data of the gyro sensor
        update_gyro_data();

        // Update data of the motors
        update_motor_data();

        // Keep balance
        if(!keep_balance()) {
            ev3_motor_stop(left_motor, false);
            ev3_motor_stop(right_motor, false);
            ev3_led_set_color(LED_RED); // TODO: knock out
            syslog(LOG_NOTICE, "Knock out!");
            return;
        }

        tslp_tsk(WAIT_TIME_MS);
    }
}

static int power = 0;
static bool_t brake = 0;
static void button_clicked_handler(intptr_t button) {
    switch(button) {
    case ENTER_BUTTON:
        syslog(LOG_NOTICE, "Enter button clicked.");
        ev3_led_set_color(LED_OFF);
        if (brake) {
            ev3_motor_stop(EV3_PORT_A, 1);
            brake = 0;
        }
        else {
            ev3_motor_stop(EV3_PORT_A, 0);
            brake = 1;
        }
        break;
    case BACK_BUTTON:
        syslog(LOG_NOTICE, "Back button clicked.");
        break;
    case LEFT_BUTTON:
    	syslog(LOG_NOTICE, "Left button clicked.");
        ev3_led_set_color(LED_RED);
    	break;
    case RIGHT_BUTTON:
    	syslog(LOG_NOTICE, "Right button clicked.");
        ev3_led_set_color(LED_GREEN);
    	break;
    case UP_BUTTON:
    	syslog(LOG_NOTICE, "Up button clicked.");
        ev3_led_set_color(LED_ORANGE);
        power += 10;
        ev3_motor_set_power(EV3_PORT_A, power);
    	break;
    case DOWN_BUTTON:
    	syslog(LOG_NOTICE, "Down button clicked.");
        ev3_led_set_color(LED_OFF);
        power -= 10;
        ev3_motor_set_power(EV3_PORT_A, power);
    	break;
    }
}

static FILE *bt = NULL;

void idle_task(intptr_t unused) {
    while(1) {
    	fprintf(bt, "Press 'h' for usage instructions.\n");
    	tslp_tsk(1000);
    }
}

#if 0
#define UDnTX_BASE				UINT_C(0xFFFFFA07)
#define UDnTX(CH)				(UDnTX_BASE + ((CH) * 16U))
typedef struct {
	int step;
	float data1;
	float data2;
	float data3;
} LogDataType;
static void put_log(LogDataType *data)
{
	int i = 0;
	volatile char *addr = (char*)UDnTX(1);
	char *p = (char*)data;
	for (i = 0; i < sizeof(LogDataType); i++) {
		*addr = p[i];
	}
	return;
}
#endif
#include "syssvc/serial.h"
#define	TASK_PORT_ID		3
#define	SVC_PERROR(expr)	svc_perror(__FILE__, __LINE__, #expr, (expr))
    
void main_task(intptr_t unused) {

	ER ercd = serial_opn_por(TASK_PORT_ID);
	if (ercd < 0 && MERCD(ercd) != E_OBJ) {
		syslog(LOG_ERROR, "%s (%d) reported by `serial_opn_por'.",
									itron_strerror(ercd), SERCD(ercd));
	}
	SVC_PERROR(serial_ctl_por(TASK_PORT_ID,
							(IOCTL_CRLF | IOCTL_FCSND | IOCTL_FCRCV)));
    ev3_led_set_color(LED_GREEN);
    // Register button handlers
    ev3_button_set_on_clicked(BACK_BUTTON, button_clicked_handler, BACK_BUTTON);
    ev3_button_set_on_clicked(ENTER_BUTTON, button_clicked_handler, ENTER_BUTTON);
    ev3_button_set_on_clicked(LEFT_BUTTON, button_clicked_handler, LEFT_BUTTON);
    ev3_button_set_on_clicked(RIGHT_BUTTON, button_clicked_handler, RIGHT_BUTTON);
    ev3_button_set_on_clicked(UP_BUTTON, button_clicked_handler, UP_BUTTON);
    ev3_button_set_on_clicked(DOWN_BUTTON, button_clicked_handler, DOWN_BUTTON);

    ev3_sensor_config(EV3_PORT_1, COLOR_SENSOR);

    // Configure motors
    ev3_motor_config(left_motor, LARGE_MOTOR);
    ev3_motor_config(right_motor, LARGE_MOTOR);
    ev3_motor_config(arm_motor, LARGE_MOTOR);
  
#if 0 
    LogDataType log_data;
    int i = 0;
#endif
    int steer = 0;
    const int steer_delta = 10;
    int power = 0;
    int power_delta = 1;
    int arm_power = 0;
    syslog(LOG_NOTICE, "#### motor control start");
    while(1) {
#if 1
    	char c;
    	ER_UINT	err = serial_rea_dat(TASK_PORTID, &c, 1);
    	if (err <= 0) {
    		if (steer > 0) {
    			steer -= 1;
    		}
    		else if (steer < 0) {
    			steer += 1;
    		}
    		tslp_tsk(20);
    		continue;
    	}
    	switch (c) {
    	case 'u':
    		power += power_delta;
    		if (power > 10) {
    			power = 10;
    		}
    		break;
    	case 'n':
    		power -= power_delta;
    		if (power < 0) {
        		power = 0;
    		}
    		break;
    	case 'k':
    		arm_power = -1;
    		break;
    	case 'i':
    		arm_power = 0;
    		break;
    	case 'j':
    		arm_power = 1;
    		break;
    	case 'l':
    		steer += steer_delta;
    		if (steer > 100) {
    			steer = 100;
    		}
    		break;
    	case 'h':
    		steer -= steer_delta;
    		if (steer < -100) {
    			steer = -100;
    		}
    		break;
    	default:
    		break;
    	}
    	syslog(LOG_NOTICE, "arm=%d power=%d steer=%d", arm_power, power, steer);
        ev3_motor_steer(left_motor, right_motor, power, steer);
        ev3_motor_set_power(arm_motor, arm_power);
#else
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
#endif
        tslp_tsk(20); /* 100msec */

    }
}
