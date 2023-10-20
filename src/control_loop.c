#include "dummy_task.h"

#include <stdint.h>

#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "uart.h"
#include "pendulum.h"
#include "motor.h"
#include "IMU.h"
#include "kalman.h"
#include "controller.h"
#include "dummy_task.h"


#define FREQ 100      //period of control loop in seconds

float angle;
float velocity;
float angvel;
float u = 0;
float ang_momentum;
float ref = 0;
float speed = 0;

static void control_loop_update(void *arg);
static osTimerId_t _control_timer_id;
static osTimerAttr_t _control_timer_attr =
{
    .name = "controlTimer"
};
static void kalman_loop_update(void *arg);
static osTimerId_t _kalman_timer_id;
static osTimerAttr_t _kalman_timer_attr =
{
    .name = "kalmanTimer"
};

static uint8_t _is_running = 0;
static uint8_t _is_init = 0;

void control_loop_init(void)  
 { 
     /* TODO: Initialise timer for use with pendulum data logging */
     _control_timer_id = osTimerNew(control_loop_update, osTimerPeriodic, NULL, &_control_timer_attr);
     _kalman_timer_id = osTimerNew(kalman_loop_update, osTimerPeriodic, NULL, &_kalman_timer_attr);
 }
void kalman_timer_start(void)
{
    int delay = 1000/100;
    osTimerStart(_kalman_timer_id,delay);
}
void kalman_timer_stop(void)
{
    osTimerStop(_kalman_timer_id);
}

void control_timer_start(void)
{
    int delay = 1000/FREQ;
    osTimerStart(_control_timer_id,delay);
}

void control_timer_stop(void)
{
    osTimerStop(_control_timer_id);
}

void control_set_speed(float spd)
{
    speed = spd;            //save speed from dummy_task
}


void kalman_loop_update(void *arg)  //kalman updates for calibration
{       UNUSED(arg);
        IMU_read();
        angle = get_acc_angle();
        angvel = get_gyroY();     //want angle of rotation about Y axis
        kalman_set_angle(angle);  
        kalman_set_velocity(angvel);
        kalman_update();          //update kalman

}

void control_loop_update(void *arg)
{       UNUSED(arg);

    ref = getReference();               //read set reference velocity
    if ((speed-ref) > (0.3)){           //compare to speed from dummy_task
        ctrl_set_yref(ref + 0.3);       //increment reference
        }
    else{
        if ((ref-speed) > (0.3))        //negative case
        {
            ctrl_set_yref(ref - 0.3);
        }
        else
        {
            ctrl_set_yref(speed);       //pass speed directly to reference
        }     
    }

    IMU_read();  /* Read IMU */      //now handled in kalman timer
    angle = get_acc_angle();
    angvel = get_gyroY();     //want angle of rotation about Y axis
    kalman_set_angle(angle);
    kalman_set_velocity(angvel);

    // update kalman
    kalman_update();
    angle = getKalmanAngle() + 0.07;     //correction term to prevent walking, estimated from matlab script
    velocity = getKalmanVelocity();      // return kalman estimated measurements
    ang_momentum = 0.0035689629*u + 0.017739975*velocity;     // calculate angular momentum
    ctrl_set_x1(angle);
    ctrl_set_x2(ang_momentum);
    ctrl_update();      //set states and update controller
    u = getControl();
    velocity_adjust(u);
}

