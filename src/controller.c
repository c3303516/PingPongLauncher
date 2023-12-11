#include <stddef.h>
#include "stm32f4xx_hal.h"
#include "arm_math.h" /* Include STM32 DSP matrix libraries */
#include "qpas_sub_noblas.h"
#include "controller.h"
 // Variables for QP solver
int numits,numadd,numdrop;


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
 // Define actuator limits
#define u_min -15*M_PI
#define u_max 15*M_PI
#define delta_u_min -0.5*M_PI
#define delta_u_max 0.5*M_PI
#define T 0.1

/* Define control arrays */
//Feedforward arrays
int i;

//his constants
// static float Kp = 200;    //launcher motor constants. might be for 150Hz measuring ubt hey
// static float Ki = 500;
// static float Kd = 0;

static float Kp = 6;    
static float Ki = 10;
static float Kd = 2;

static float ele_Kp = 60;        //these params are without the 13000 offset it elevation adjust
static float ele_Ki = 40;
static float ele_Kd = 20;
// static float ele_Kp = 10;        //these params are without the 13000 offset it elevation adjust
// static float ele_Ki = 5;
// static float ele_Kd = 10;

static float azi_Kp = 50;        //500    
static float azi_Ki = 100;
static float azi_Kd = 20;


static float elevation = 0;
static float azimuth = 0;
static float yref = 0;
static float servo = 0;

static float u;
static float u_ele;
static float u_azi;

 void ctrl_init(void)
 {
  // Set reference to 0 initially
  ctrl_set_yref(0);
 
 }


float getControl(void)
{
    return u;
}

float getEleControl(void)
{
    return u_ele;
}
float getAziControl(void)
{
    return u_azi;
}

void ctrl_set_yref(float y)
{
    // Updatereference velocity. THis might be unecessary but it works so.
    yref = y;
}

float getReference(void)
{
    return yref;
}

float getElevation(void)
{
    return elevation;
}

void control_set_elevation(float ele)
{
    elevation = ele;
}


float getAzimuth(void)
{
    return azimuth;
}

void control_set_azimuth(float azi)
{
    azimuth = azi;
}

float getServo(void)
{
    return servo;
}

void control_Servo(float ser)
{
    servo = ser;
}

/* Update control output */
float ctrl_update(float err, float err_i, float err_d)
{
    u = Kp*err +Ki*err_i + Kd*err_d;
    return u;
}

void ele_ctrl_update(float err, float err_i, float err_d)
{
u_ele = ele_Kp*err + ele_Ki*err_i + ele_Kd*err_d;
// return u;
}

void azi_ctrl_update(float err, float err_i, float err_d)
{
u_azi = azi_Kp*err + azi_Ki*err_i + azi_Kd*err_d;
// return u;
}