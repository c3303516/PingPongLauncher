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

static float Kp = 30;    
static float Ki = 30;
static float Kd = 0;



static float yref = 0;


static float u;


 void ctrl_init(void)
 {
  // Set reference to 0 initially
  ctrl_set_yref(0);
 
 }


float getControl(void)
{
    return u;
}


void ctrl_set_yref(float y)
{
// Updatereference velocity
yref = y;
}
float getReference(void)
{
    return yref;
}

/* Update control output */
 void ctrl_update(float err, float err_i, float err_d)
 {
    u = Kp*err +Ki*err_i + Kd*err_d;
 }