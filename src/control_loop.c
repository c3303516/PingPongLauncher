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


#define FREQ 10      //period of control loop in seconds
#define INPUTMAX 100000

float angle;
float velocity;
float angvel;
float u = 0;
float ang_momentum;
float refvel = 0;
float speed = 0;
float thrustpercent;

int32_t enc1_t=0;
int32_t enc2_t= 0;
int32_t enc3_t= 0;
int32_t enc1_t1 = 0;
int32_t enc2_t1= 0;
int32_t enc3_t1= 0;
int32_t enc1_diff;
static float errorold = 0;
static float errornew = 0;
static float erri = 0;
static float derr = 0;
static float input = 0;
static float ubar = 0;
int i = 0;

static void control_loop_update(void *arg);
static osTimerId_t _control_timer_id;
static osTimerAttr_t _control_timer_attr =
{
    .name = "controlTimer"
};
// static void kalman_loop_update(void *arg);
// static osTimerId_t _kalman_timer_id;
// static osTimerAttr_t _kalman_timer_attr =
// {
//     .name = "kalmanTimer"
// };

static uint8_t _is_running = 0;
static uint8_t _is_init = 0;

void control_loop_init(void)  
 { 
     /* TODO: Initialise timer for use with pendulum data logging */
     _control_timer_id = osTimerNew(control_loop_update, osTimerPeriodic, NULL, &_control_timer_attr);
    //  _kalman_timer_id = osTimerNew(kalman_loop_update, osTimerPeriodic, NULL, &_kalman_timer_attr);


 }
// void kalman_timer_start(void)
// {
//     int delay = 1000/100;
//     osTimerStart(_kalman_timer_id,delay);
// }
// void kalman_timer_stop(void)
// {
//     osTimerStop(_kalman_timer_id);
// }

void control_timer_start(void)
{
    int delay = 1000/FREQ;
    // int delay = 5000;
    printf("started\n");
    osTimerStart(_control_timer_id,delay);
}

void control_timer_stop(void)
{
    osTimerStop(_control_timer_id);
}

void control_set_speed(float spd)
{
    speed = spd;            //save speed from dummy_task
    ctrl_set_yref(speed);
}


void kalman_loop_update(void *arg)  //kalman updates for calibration
{       UNUSED(arg);
        IMU_read();
        angle = get_acc_angle();
        angvel = get_gyroY();     //want angle of rotation about Y axis
        // kalman_set_angle(angle);  
        // kalman_set_velocity(angvel);
        // kalman_update();          //update kalman

}

void control_loop_update(void *arg)
{       UNUSED(arg);

    // find error
    enc1_t = motor_encoder_getValue();
    enc1_diff = enc1_t - enc1_t1;
    if (enc1_diff < 0){     //wrap around consideration for encodr count
        enc1_diff = enc1_diff + 2147483647;
    }
    angvel = motor_encoder_getRev(enc1_diff)/(1./FREQ);    //find ang velocity, in RPS


    refvel = getReference();

    //35 is 20 000 input, for the 5V
    ubar = 20000*(refvel/35);       // approximate linearly from ref


    printf("referencevalue %0.1f\n", refvel);
    errornew = refvel - angvel;

    if ((angvel == 0 )&&(refvel == 0 )){
        errornew = 0;
        erri = 0;       //clear all control
        derr = 0;
    }


    derr = (errornew - errorold)/(1./FREQ);      //find derr/dt
    erri = erri + 0.5*(errornew + errorold)*(1./FREQ);      //find approx integral. maybe adjust this later

    //try simpsons rule, assuming linear over timestep

    // erri = erri + (1./FREQ)*(1/3)*(errorold + 2*(errornew+errorold) + errornew);


    ctrl_update(errornew, erri, derr);      //update control

    input = getControl();       //max value will be 100 0000. Input will always be positive now. Controller will
                                        //adjust it
    


 
 
 
    thrustpercent = ubar + input;           // this will allow negative inptus

    printf("controlinput %0.5f\n",thrustpercent);

    //saturation tests
    if (thrustpercent< 0 ){
        thrustpercent = 0;
    } else if (thrustpercent > INPUTMAX){
        thrustpercent = INPUTMAX;        //saturate the percentage
        erri = 0;       //cleear
    }

    // printf("thrust %0.5f\n", thrustpercent);
    velocity_adjust(thrustpercent);     //apply new velocity

    printf("motor vel %0.1f\n", angvel);

    enc1_t1 = enc1_t;       //update encoder count
    errorold = errornew;
}

