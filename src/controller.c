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
#define T 0.01

/* Define control arrays */
//Feedforward arrays
int i;

static float32_t ctrl_Nx_f32[CTRL_N_STATE-1] =
{
         0,
    0.0035689629,
};
static float32_t ctrl_Nu_f32[1] =
{
1,
};

static float32_t ctrl_Az_f32[1*CTRL_N_STATE] =
{
0.0,0.0,1.0,
};
 // Hessian
 static float32_t ctrl_H_f32[CTRL_N_HORIZON*CTRL_N_HORIZON] =
 {
0.29590802,0.089757607,0.08397461,0.078539956,0.073432591,0.06863273,0.064121788,0.0598823,0.055897858,0.052153046,
0.089757607,0.28402196,0.078619462,0.07353918,0.068764766,0.064277731,0.060060706,0.056097369,0.052372387,0.048871351,
0.08397461,0.078619462,0.27358467,0.068840675,0.064379086,0.060185969,0.056245095,0.052541218,0.049060012,0.045788018,
0.078539956,0.07353918,0.068840675,0.26442284,0.060258499,0.056341533,0.052660119,0.049200018,0.045947854,0.042891057,
0.073432591,0.068764766,0.064379086,0.060258499,0.25638365,0.052729474,0.049291836,0.046060777,0.043023809,0.040169202,
0.06863273,0.064277731,0.060185969,0.056341533,0.052729474,0.24933241,0.046127148,0.043111286,0.040276508,0.037611869,
0.064121788,0.060060706,0.056245095,0.052660119,0.049291836,0.046127148,0.24315042,0.040340077,0.037695267,0.035209114,
0.0598823,0.056097369,0.052541218,0.049200018,0.046060777,0.043111286,0.040340077,0.23773304,0.035270049,0.032951593,
0.055897858,0.052372387,0.049060012,0.045947854,0.043023809,0.040276508,0.037695267,0.035270049,0.23298809,0.030830528,
0.052153046,0.048871351,0.045788018,0.042891057,0.040169202,0.037611869,0.035209114,0.032951593,0.030830528,0.22883434,
};
 // f bar
 static float32_t ctrl_fBar_f32[CTRL_N_HORIZON*CTRL_N_STATE] =
 {
-52.02908,-470.39775,-0.55654164,
-48.62395,-439.61178,-0.50778497,
-45.424642,-410.68666,-0.46209627,
-42.418716,-383.50992,-0.41929011,
-39.594483,-357.97588,-0.37919226,
-36.94096,-333.98523,-0.341639,
-34.447829,-311.4447,-0.30647654,
-32.105395,-290.26662,-0.27356035,
-29.904549,-270.36865,-0.24275467,
-27.836732,-251.67341,-0.21393192,
 };
 // f
 static float32_t ctrl_f_f32[CTRL_N_HORIZON] =
 {
 0.0,
 0.0,
 0.0,
 0.0,
 0.0,
 0.0,
 0.0,
 0.0,
 0.0,
 0.0,
 };
 // State vector
   static float32_t ctrl_x_f32[CTRL_N_STATE] =
 {
 0.0,
 0.0,
 0.0
 };
  static float32_t ctrl_xHat_f32[CTRL_N_STATE] =
 {
 0.0,
 0.0,
 0.0
 };
   static float32_t ctrl_xStar_f32[CTRL_N_STATE] =
 {
 0.0,
 0.0,
 0.0
 };
 // Control
 static float32_t ctrl_u_f32[CTRL_N_INPUT] =
 {
 0.0,
 };

static float32_t ctrl_yref_f32[CTRL_N_INPUT] =
 {
 0.0,
 };
 // U star
 static float32_t ctrl_Ustar_f32[CTRL_N_HORIZON*CTRL_N_INPUT] =
 {
 0.0,
 0.0,
 0.0,
 0.0,
 0.0,
 0.0,
 0.0,
 0.0,
 0.0,
 0.0,
 };
 static float32_t ctrl_U_f32[CTRL_N_HORIZON*CTRL_N_INPUT] =
 {
 0.0,
 0.0,
 0.0,
 0.0,
 0.0,
 0.0,
 0.0,
 0.0,
 0.0,
 0.0,
 };
  static float32_t ctrl_result_f32[CTRL_N_HORIZON*CTRL_N_INPUT] =
 {
 0.0,
 0.0,
 0.0,
 0.0,
 0.0,
 0.0,
 0.0,
 0.0,
 0.0,
 0.0,
 };
   static float32_t ctrl_result2_f32[CTRL_N_HORIZON*CTRL_N_INPUT] =
 {
 0.0,
 0.0,
 0.0,
 0.0,
 0.0,
 0.0,
 0.0,
 0.0,
 0.0,
 0.0,
 };

 // Constraints
 static float ctrl_A_f32[CTRL_N_INEQ_CONST*CTRL_N_HORIZON] =
 {
1,-1,0,0,0,0,0,0,0,0,-1,1,0,0,0,0,0,0,0,0,
0,1,-1,0,0,0,0,0,0,0,0,-1,1,0,0,0,0,0,0,0,
0,0,1,-1,0,0,0,0,0,0,0,0,-1,1,0,0,0,0,0,0,
0,0,0,1,-1,0,0,0,0,0,0,0,0,-1,1,0,0,0,0,0,
0,0,0,0,1,-1,0,0,0,0,0,0,0,0,-1,1,0,0,0,0,
0,0,0,0,0,1,-1,0,0,0,0,0,0,0,0,-1,1,0,0,0,
0,0,0,0,0,0,1,-1,0,0,0,0,0,0,0,0,-1,1,0,0,
0,0,0,0,0,0,0,1,-1,0,0,0,0,0,0,0,0,-1,1,0,
0,0,0,0,0,0,0,0,1,-1,0,0,0,0,0,0,0,0,-1,1,
0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,-1,
 };
 static float ctrl_b_f32[CTRL_N_INEQ_CONST] =
 {
delta_u_max,
delta_u_max,
delta_u_max,
delta_u_max,
delta_u_max,
-delta_u_min,
-delta_u_min,
-delta_u_min,
-delta_u_min,
-delta_u_min,
 };
 static float ctrl_xl_f32[CTRL_N_LB_CONST] =
 {
u_min,
u_min,
u_min,
u_min,
u_min,
u_min,
u_min,
u_min,
u_min,
u_min,
 };
 static float ctrl_xu_f32[CTRL_N_UB_CONST] =
 {
u_max,
u_max,
u_max,
u_max,
u_max,
u_max,
u_max,
u_max,
u_max,
u_max,
 };
 static float ctrl_lm_f32[CTRL_N_EQ_CONST+CTRL_N_INEQ_CONST+CTRL_N_LB_CONST+CTRL_N_UB_CONST] =
 {
 };


/* Define control matrix variables */
 // rows, columns, data array
    arm_matrix_instance_f32 ctrl_H = {CTRL_N_HORIZON, CTRL_N_HORIZON, (float32_t *)ctrl_H_f32};
    arm_matrix_instance_f32 ctrl_fBar = {CTRL_N_HORIZON, CTRL_N_STATE, (float32_t *)ctrl_fBar_f32};
    arm_matrix_instance_f32 ctrl_f = {CTRL_N_HORIZON, 1, (float32_t *)ctrl_f_f32};
    arm_matrix_instance_f32 ctrl_x = {CTRL_N_STATE, 1, (float32_t *)ctrl_x_f32};
    arm_matrix_instance_f32 ctrl_xHat = {CTRL_N_STATE, 1, (float32_t *)ctrl_xHat_f32};
    arm_matrix_instance_f32 ctrl_xStar = {CTRL_N_STATE, 1, (float32_t *)ctrl_xStar_f32};
    arm_matrix_instance_f32 ctrl_u = {CTRL_N_INPUT, 1, (float32_t *)ctrl_u_f32};
    arm_matrix_instance_f32 ctrl_yref = {CTRL_N_INPUT, 1, (float32_t *)ctrl_yref_f32};
    arm_matrix_instance_f32 ctrl_Ustar = {CTRL_N_HORIZON, CTRL_N_INPUT, (float32_t *)ctrl_Ustar_f32};
    arm_matrix_instance_f32 ctrl_result = {CTRL_N_HORIZON, CTRL_N_INPUT,  (float32_t *)ctrl_result_f32};
    arm_matrix_instance_f32 ctrl_result2 = {CTRL_N_HORIZON, CTRL_N_INPUT,  (float32_t *)ctrl_result2_f32};
    arm_matrix_instance_f32 ctrl_U = {CTRL_N_HORIZON, CTRL_N_INPUT,  (float32_t *)ctrl_U_f32};
    arm_matrix_instance_f32 ctrl_Nx = {CTRL_N_STATE-1, 1, (float32_t *)ctrl_Nx_f32};
    arm_matrix_instance_f32 ctrl_Nu = {CTRL_N_INPUT, 1, (float32_t *)ctrl_Nu_f32};
    arm_matrix_instance_f32 ctrl_Az = {1, CTRL_N_STATE, (float32_t *)ctrl_Az_f32};
 /* Control functions */
 void ctrl_init(void)
 {
    arm_mat_init_f32(&ctrl_H, CTRL_N_HORIZON, CTRL_N_HORIZON, (float32_t *)ctrl_H_f32);
    arm_mat_init_f32(&ctrl_fBar, CTRL_N_HORIZON, CTRL_N_STATE, (float32_t *)ctrl_fBar_f32);
    arm_mat_init_f32(&ctrl_f, CTRL_N_HORIZON, 1, (float32_t *)ctrl_f_f32);
    arm_mat_init_f32(&ctrl_x, CTRL_N_STATE, 1, (float32_t *)ctrl_x_f32);
    arm_mat_init_f32(&ctrl_xHat, CTRL_N_STATE, 1, (float32_t *)ctrl_xHat_f32);
    arm_mat_init_f32(&ctrl_xStar, CTRL_N_STATE, 1, (float32_t *)ctrl_xStar_f32);
    arm_mat_init_f32(&ctrl_u, CTRL_N_INPUT, 1, (float32_t *)ctrl_u_f32);
    arm_mat_init_f32(&ctrl_yref, CTRL_N_INPUT, 1, (float32_t *)ctrl_yref_f32);
    arm_mat_init_f32(&ctrl_Ustar, CTRL_N_HORIZON, CTRL_N_INPUT,  (float32_t *)ctrl_Ustar_f32);
    arm_mat_init_f32(&ctrl_result, CTRL_N_HORIZON, CTRL_N_INPUT,  (float32_t *)ctrl_result_f32);
    arm_mat_init_f32(&ctrl_result2, CTRL_N_HORIZON, CTRL_N_INPUT,  (float32_t *)ctrl_result2_f32);
    arm_mat_init_f32(&ctrl_U, CTRL_N_HORIZON, CTRL_N_INPUT, (float32_t *)ctrl_U_f32);
    arm_mat_init_f32(&ctrl_Nx, CTRL_N_INPUT-1, 1, (float32_t *)ctrl_Nx_f32);
    arm_mat_init_f32(&ctrl_Nu, CTRL_N_INPUT, 1, (float32_t *)ctrl_Nu_f32);
    arm_mat_init_f32(&ctrl_Az, 1, CTRL_N_INPUT, (float32_t *)ctrl_Az_f32);
 }


float getControl(void)
{
    return ctrl_u_f32[0];
}
 void ctrl_set_x1(float x1)
{
// Update state x1
ctrl_x_f32[0] = x1;
}

void ctrl_set_x2(float x2)
{
// Update state x2
ctrl_x_f32[1] = x2;
}
void ctrl_set_yref(float y)
{
// Updatereference velocity
ctrl_yref_f32[0] = y;
}
float getReference(void)
{
    return ctrl_yref_f32[0];
}

/* Update control output */
 void ctrl_update(void)
 {



 }