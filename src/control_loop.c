#include "dummy_task.h"

#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "uart.h"
#include "pendulum.h"
#include "motor.h"
#include "IMU.h"
#include "kalman.h"
#include "controller.h"
#include "dummy_task.h"

#define FREQL 5       //launher
#define FREQ 10      //period of control loop in seconds aim loop
#define INPUTMAX 100000/2       //cant remember why the 2 was there. Fixed pwm earlier tho?
#define PERIOD 1./FREQ
#define PERIODL 1./FREQL

float angle;
float u = 0;
float ang_momentum;
float refvel = 0;
float speed = 0;



int32_t enc1_t=0;
int32_t enc2_t= 0;
int32_t enc3_t= 0;
int32_t enc1_t1 = 0;
int32_t enc2_t1= 0;
int32_t enc3_t1= 0;
int32_t enc1_diff;
int32_t enc2_diff;
int32_t enc3_diff;

static float ubar = 0;          //verify this later

static float error1old = 0;
static float error1new = 0;
static float erri1 = 0;
static float derr1 = 0;
static float input1 = 0;
float angvel1;
float thrustpercent1;

static float error2old = 0;
static float error2new = 0;
static float erri2 = 0;
static float derr2 = 0;
static float input2 = 0;
float angvel2;
float thrustpercent2;

static float error3old = 0;
static float error3new = 0;
static float erri3 = 0;
static float derr3 = 0;
static float input3 = 0;
float angvel3;
float thrustpercent3;

int i = 0;

float Ele_old = 0;
float Ele_new = 0;
static float Ele_errorold = 0;
static float Ele_errornew = 0;
static float Ele_erri = 0;
static float Ele_derr = 0;
static float ele_input = 0;
static float ele_angvel = 0;
float ele_mag;
float refEle = 0;
float newrefEle = 0;
float elevation = 0;

float azi_old = 0;
float azi_new = 0;
static float azi_errorold = 0;
static float azi_errornew = 0;
static float azi_erri = 0;
static float azi_derr = 0;
static float azi_input = 0;
static float azi_angvel = 0;
float azi_mag;
float refazi = 0;
float newrefazi = 0;
float azimuth = 0;

static uint8_t _is_running = 0;
static uint8_t _is_init = 0;

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

static void aim_loop_update(void *arg);
static osTimerId_t _aim_timer_id;
static osTimerAttr_t _aim_timer_attr =
{
    .name = "aimTimer"
};

static void servo_loop_update(void *arg);
static osTimerId_t _servo_timer_id;
static osTimerAttr_t _servo_timer_attr =
{
    .name = "servoTimer"
};

void control_loop_init(void)  
 { 
     /* TODO: Initialise timer for use with pendulum data logging */
     _control_timer_id = osTimerNew(control_loop_update, osTimerPeriodic, NULL, &_control_timer_attr);
    //  _kalman_timer_id = osTimerNew(kalman_loop_update, osTimerPeriodic, NULL, &_kalman_timer_attr);
 }


 void aim_loop_init(void)  
 { 
     /* TODO: Initialise timer for use with pendulum data logging */
     _aim_timer_id = osTimerNew(aim_loop_update, osTimerPeriodic, NULL, &_aim_timer_attr);
    //  _kalman_timer_id = osTimerNew(kalman_loop_update, osTimerPeriodic, NULL, &_kalman_timer_attr);
 }

  void servo_loop_init(void)  
 { 
     /* TODO: Initialise timer for use with pendulum data logging */
     _servo_timer_id = osTimerNew(servo_loop_update, osTimerPeriodic, NULL, &_servo_timer_attr);
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
    int delay = 1000/FREQL;
    // int delay = 5000;
    printf("Control timer started\n");
    osTimerStart(_control_timer_id,delay);
}

void control_timer_stop(void)
{
    osTimerStop(_control_timer_id);
}

void aim_timer_start(void)
{
    int delay = 1000/FREQ;
    // int delay = 5000;
    printf("Aim timer started\n");
    osTimerStart(_aim_timer_id,delay);
}

void aim_timer_stop(void)
{
    osTimerStop(_aim_timer_id);
}

void servo_timer_start(void)
{
    int delay = 1000/FREQ;
    // int delay = 5000;
    printf("Servo timer started\n");
    osTimerStart(_servo_timer_id,delay);
}

void servo_timer_stop(void)
{
    osTimerStop(_servo_timer_id);
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
        // angvel = get_gyroY();     //want angle of rotation about Y axis
        // kalman_set_angle(angle);  
        // kalman_set_velocity(angvel);
        // kalman_update();          //update kalman

}

void control_loop_update(void *arg)
{       UNUSED(arg);

    // printf("%f\n", PERIOD);

    // find error
    enc1_t = motor_encoder1_getValue();
    enc2_t = motor_encoder2_getValue();
    enc3_t = motor_encoder3_getValue();
    // printf("Encoders %i, %i, %i\n",enc1_t,enc2_t,enc3_t);
    enc1_diff = enc1_t - enc1_t1;
    if (enc1_diff < 0){     //wrap around consideration for encodr count
        enc1_diff = enc1_diff + 2147483647;
    }
    angvel1 = 60*(enc1_diff/(12*4.995))/(PERIODL);    //find ang velocity, in RPM

    enc2_diff = enc2_t - enc2_t1;
    if (enc2_diff < 0){     //wrap around consideration for encodr count
        enc2_diff = enc2_diff + 2147483647;
    }
    angvel2 = 60*(enc2_diff/(12*4.995))/(PERIODL);    //find ang velocity, in RPS

    enc3_diff = enc3_t - enc3_t1;
    if (enc3_diff < 0){     //wrap around consideration for encodr count
        enc3_diff = enc3_diff + 2147483647;
    }
    angvel3 = 60*(enc3_diff/(12*4.995))/(PERIODL);    //find ang velocity, in RPS

    refvel = getReference();

    //10000 is 900rpm at 6 volts
    ubar = 10000*(refvel/900);       // approximate linearly from ref

    // printf("referencevalue %0.1f\n", refvel);
    //try simpsons rule, assuming linear over timestep
    // erri = erri + (PERIOD)*(1/3)*(errorold + 2*(errornew+errorold) + errornew);

    error1new = refvel - angvel1;
    derr1 = (error1new - error1old)/(PERIODL);      //find derr/dt might wrong here?
    erri1 = erri1 + 0.5*(error1new + error1old)*(PERIODL);      //find approx integral. maybe adjust this later

    if ((angvel1 == 0 )||(refvel == 0 )){
            error1new = 0;
            erri1 = 0;       //clear all control
            derr1 = 0;
        }
    input1 = ctrl_update(error1new, erri1, derr1);      //update control
    thrustpercent1 = ubar + input1;           // this will allow negative inptus

    error2new = refvel - angvel2;
    derr2 = (error2new - error2old)/(PERIODL);      //find derr/dt might wrong here?
    erri2 = erri2 + 0.5*(error2new + error2old)*(PERIODL);      //find approx integral. maybe adjust this later

    if ((angvel2 == 0 )||(refvel == 0 )){
            error2new = 0;
            erri2 = 0;       //clear all control
            derr2 = 0;
        }
    input2 = ctrl_update(error2new, erri2, derr2);      //update control
    thrustpercent2 = ubar + input2;           // this will allow negative inptus

    error3new = refvel - angvel3;
    derr3 = (error3new - error3old)/(PERIOD);      //find derr/dt might wrong here?
    erri3 = erri3 + 0.5*(error3new + error3old)*(PERIODL);      //find approx integral. maybe adjust this later

    if ((angvel3 == 0 )||(refvel == 0 )){
            error3new = 0;
            erri3 = 0;       //clear all control
            derr3 = 0;
        }
    input3 = ctrl_update(error3new, erri3, derr3);      //update control
    thrustpercent3 = ubar + input3;           // this will allow negative inptus

    // printf("controlinput %0.5f\n",thrustpercent);

    //saturation tests
    if (thrustpercent1 > INPUTMAX){
        thrustpercent1 = INPUTMAX;        //saturate the percentage
        // erri1 = 0;       //cleear
    }
    if (thrustpercent2 > INPUTMAX){
        thrustpercent2 = INPUTMAX;        //saturate the percentage
        // erri2 = 0;       //cleear
    }
    if (thrustpercent3 > INPUTMAX){
        thrustpercent3 = INPUTMAX;        //saturate the percentage
        // erri3 = 0;       //cleear
    }

    // printf("thrust %0.5f\n", thrustpercent1);
    velocity_adjust(thrustpercent1,thrustpercent2,thrustpercent3);     //apply new velocity

    printf("motor vel %0.5f\n", angvel1);

    enc1_t1 = enc1_t;       //update encoder count
    error1old = error1new;
    enc2_t1 = enc2_t;       //update encoder count
    error2old = error2new;
    enc3_t1 = enc3_t;       //update encoder count
    error3old = error3new;



}

void aim_loop_update(void *arg)
{   UNUSED(arg);
    // Ele_new = elevation_encoder_getAngle(ele_encoder_getValue());
    Ele_new = ele_encoder_getValue();
    ele_angvel = (Ele_new - Ele_old)/(PERIOD);
    // printf("Elevation %0.1f\n",getElevation());
    newrefEle = 298*14*2*(50/31.25)*(getElevation()/360);        //ref angle to ref encoder count
                        //gear ratio is approximated from creo model.
    if (newrefEle != refEle){
        Ele_erri = 0;       //clear integrator 
        refEle = newrefEle;
    }

    Ele_errornew = refEle - Ele_new;        //find error
    Ele_derr = (Ele_errornew - Ele_errorold)/(PERIOD);      //find derr/dt
    Ele_erri = Ele_erri + 0.5*(Ele_errornew + Ele_errorold)*(PERIOD);      //find approx integral. maybe adjust this later

    ele_ctrl_update(Ele_errornew, Ele_erri, Ele_derr);      //update control
    ele_input = getEleControl(); 

    ele_mag = fabs(Ele_errornew);
    // if ((ele_angvel = 0)&&(ele_mag<10)){  //will stop moving if within this?
    if ((ele_mag<10)){  //will stop moving if within this range.

        // Ele_errornew = 0;    //no longer resetting integrator
        // Ele_erri = 0;
        ele_input = 0;      //testing this
    }

    if (ele_input < (-1*INPUTMAX) ){
        ele_input = -INPUTMAX;
        Ele_erri = 0;
    } else if (ele_input > INPUTMAX){
        ele_input = INPUTMAX;        //saturate the percentage
        Ele_erri = 0;      //cleear
    }
    
    //AZIMUTH///////////
    azi_new = azi_encoder_getValue();
    azi_angvel = (azi_new - azi_old)/(PERIOD);
    // printf("Azi %0.1f\n", getAzimuth());
    newrefazi = 298*14*2*(getAzimuth()/360);        //work on encoder count now?

    if (newrefazi != refazi){
        azi_erri = 0;       //clear integrator 
        refazi = newrefazi;
    }

    azi_errornew = refazi - azi_new;        //find error
    azi_derr = (azi_errornew - azi_errorold)/(PERIOD);      //find derr/dt
    azi_erri = azi_erri + 0.5*(azi_errornew + azi_errorold)*(PERIOD);      //find approx integral. maybe adjust this later

    azi_ctrl_update(azi_errornew, azi_erri, azi_derr);      //update control
    azi_input = getAziControl(); 

    azi_mag = fabs(azi_errornew);
    if ((azi_angvel = 0)&&(azi_mag<10)){  //will stop moving if within this

        azi_errornew = 0;
        azi_erri = 0;
    }

    if (azi_input < (-1*INPUTMAX) ){
        azi_input = -INPUTMAX;
        azi_erri = 0;
    } else if (azi_input > INPUTMAX){
        azi_input = INPUTMAX;        //saturate the percentage
        azi_erri = 0;      //cleear
    }
    // ele_input = 0;

    elevation_adjust(ele_input);
    azimuth_adjust(azi_input);



    // printf("ElevationRef %0.2f\n", refEle);
    // printf("ControlINput %0.2f\n", ele_input);
    // printf("Elevation %0.2f\n", Ele_new);
    Ele_old = Ele_new;
    Ele_errorold = Ele_errornew;
    azi_old = azi_new;
    azi_errorold = azi_errornew;


    elevation = (360*(Ele_new/(298*14*2)))*(31.25/50);
    azimuth = 360*(azi_new/(298*14*2));
    // printf("Set Ele Azi %0.1f, %0.1f\n",newrefEle,newrefazi);
    // printf("Current Ele Azi %0.3f, %0.3f\n", elevation, azimuth);

    
}



void servo_loop_update(void *arg){

    // printf("Servo\n");
    float degrees = getServo(); // hijack elevation rn

    servo_adjust(degrees);

}