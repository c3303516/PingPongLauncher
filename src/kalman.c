#include <stddef.h>
#include "stm32f4xx_hal.h"
#include "arm_math.h" /* Include STM32 DSP matrix libraries */
#include "qpas_sub_noblas.h"
#include "kalman.h"
#include "IMU.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define T 0.01 //discrete time segment - eventually sync this up with control


 // f
 static float32_t kal_R_f32[KAL_N_INPUT*KAL_N_INPUT] =
 {
0.0645,0,
0,0.00011568,
 };
 // State vector
 static float32_t kal_Pm_f32[KAL_N_STATE*KAL_N_STATE] =
 {
0.1,0,0,
0,0.1,0,
0,0,0.1,
 };
 // Control
 static float32_t kal_Q_f32[KAL_N_STATE*KAL_N_STATE] =
 {
0.045701074,0.0021237534,0.006740862,
0.0021237534,0.00010390508,0.00031494774,
0.006740862,0.00031494774,0.00099790184,
 };
static float32_t kal_L_f32[KAL_N_STATE*KAL_N_STATE] =
 {
0.26637682,0,0,
0.016305657,0.00017962317,0,
0.016553555,0.0015135437,0.0023942329,
 };
 static float32_t kal_xm_f32[KAL_N_STATE] =
 {
0.0,
0.0,
0.0,
 };
  static float32_t kal_xp_f32[KAL_N_STATE] =
 {
0.0,
0.0,
0.0,
 };
 static float32_t kal_Pp_f32[KAL_N_STATE*KAL_N_STATE] =
 {
0,0,0,
0,0,0,
0,0,0,
 };
  static float32_t kal_Ad_f32[KAL_N_STATE*KAL_N_STATE] =
 {
1,0,0,
T,1,0,
0,0,1,
 };
   static float32_t kal_Bd_f32[KAL_N_STATE] =
 {
0.0,
0.0,
0.0,
 };
   static float32_t kal_C_f32[KAL_N_INPUT*KAL_N_STATE] =
 {
0,1,0,
1,0,1,
 };
static float32_t inv_C_f32[KAL_N_STATE*KAL_N_INPUT] =
 {
0,1,
1,0,
0,1,
 };
    static float32_t kal_Kk_f32[KAL_N_STATE*KAL_N_INPUT] =
 {
0.0, 0.0,
0.0, 0.0,
0.0, 0.0,
 };
     static float32_t inv_Kk_f32[KAL_N_INPUT*KAL_N_STATE] =
 {
0.0, 0.0, 0.0,
0.0, 0.0, 0.0,
 };
     static float32_t kal_y_f32[KAL_N_INPUT] =
 {
0.0,
0.0,
 };

static float32_t result32_f32[3*2] =
 {
0.0, 0.0,
0.0, 0.0,
0.0, 0.0,
 };
static float32_t result33_f32[3*3] =
 {
0.0, 0.0, 0.0,
0.0, 0.0, 0.0,
0.0, 0.0, 0.0,
 };
 static float32_t result33_2_f32[3*3] =
 {
0.0, 0.0, 0.0,
0.0, 0.0, 0.0,
0.0, 0.0, 0.0,
 };
 static float32_t result23_f32[2*3] =
 {
0.0, 0.0, 0.0,
0.0, 0.0, 0.0,
 };
static float32_t result22_f32[2*2] =
 {
0.0, 0.0,
0.0, 0.0,
 };
 static float32_t result22_2_f32[2*2] =
 {
0.0, 0.0,
0.0, 0.0,
 };
 static float32_t result21_f32[2] =
 {
0.0,
0.0,
 };
static float32_t result21_2_f32[2] =
 {
0.0,
0.0,
 };
  static float32_t result31_f32[3*1] =
 {
0.0,
0.0,
0.0,
 };
static float32_t eye3_f32[3*3] =
 {
1.0, 0.0, 0.0,
0.0, 1.0, 0.0,
0.0, 0.0, 1.0,
 };
/* Define control matrix variables */
 // rows, columns, data array
arm_matrix_instance_f32 kal_R = {KAL_N_INPUT, KAL_N_INPUT, (float32_t *)kal_R_f32};
arm_matrix_instance_f32 kal_Pm = {KAL_N_STATE, KAL_N_STATE, (float32_t *)kal_Pm_f32};
arm_matrix_instance_f32 kal_Q = {KAL_N_STATE, KAL_N_STATE, (float32_t *)kal_Q_f32};
arm_matrix_instance_f32 kal_L = {KAL_N_STATE, KAL_N_STATE, (float32_t *)kal_L_f32};
arm_matrix_instance_f32 kal_xm = {KAL_N_STATE, 1, (float32_t *)kal_xm_f32};
arm_matrix_instance_f32 kal_xp = {KAL_N_STATE, 1, (float32_t *)kal_xp_f32};
arm_matrix_instance_f32 kal_Pp = {KAL_N_STATE, KAL_N_STATE, (float32_t *)kal_Pp_f32};
//arm_matrix_instance_f32 kal_xp = {KAL_N_STATE, 1, (float32_t *)kal_xp_f32};
arm_matrix_instance_f32 kal_Ad = {KAL_N_STATE, KAL_N_STATE, (float32_t *)kal_Ad_f32};
arm_matrix_instance_f32 kal_Bd = {KAL_N_STATE, 1, (float32_t *)kal_Bd_f32};
arm_matrix_instance_f32 kal_C = {KAL_N_INPUT, KAL_N_STATE, (float32_t *)kal_C_f32};
arm_matrix_instance_f32 kal_Kk = {KAL_N_STATE, KAL_N_INPUT, (float32_t *)kal_Kk_f32};
arm_matrix_instance_f32 inv_Kk = {KAL_N_INPUT, KAL_N_STATE,(float32_t *)inv_Kk_f32};
arm_matrix_instance_f32 kal_y = {KAL_N_INPUT, 1, (float32_t *)kal_y_f32};
arm_matrix_instance_f32 result32 = {3, 2, (float32_t *)result32_f32};
arm_matrix_instance_f32 result23 = {2, 3, (float32_t *)result23_f32};
arm_matrix_instance_f32 result22 = {2, 2, (float32_t *)result22_f32};
arm_matrix_instance_f32 result22_2 = {2, 2, (float32_t *)result22_2_f32};
arm_matrix_instance_f32 result33 = {3, 3, (float32_t *)result33_f32};
arm_matrix_instance_f32 result33_2 = {3, 3, (float32_t *)result33_2_f32};
arm_matrix_instance_f32 result21 = {2, 1, (float32_t *)result21_f32};
arm_matrix_instance_f32 result21_2 = {2, 1, (float32_t *)result21_2_f32};
arm_matrix_instance_f32 result31 = {3, 1, (float32_t *)result31_f32};
arm_matrix_instance_f32 inv_C = {KAL_N_STATE, KAL_N_INPUT, (float32_t *)inv_C_f32};
arm_matrix_instance_f32 eye3 = {3, 3, (float32_t *)eye3_f32};
 /* Control functions */
 void kalman_init(void)
 {
arm_mat_init_f32(&kal_R, KAL_N_INPUT, KAL_N_INPUT, (float32_t *)kal_R_f32);
arm_mat_init_f32(&kal_Pm, KAL_N_STATE, KAL_N_STATE, (float32_t *)kal_Pm_f32);
arm_mat_init_f32(&kal_Q, KAL_N_STATE, KAL_N_STATE, (float32_t *)kal_Q_f32);
arm_mat_init_f32(&kal_L, KAL_N_STATE, KAL_N_STATE, (float32_t *)kal_L_f32);
arm_mat_init_f32(&kal_xm, KAL_N_STATE, 1, (float32_t *)kal_xm_f32);
arm_mat_init_f32(&kal_xp, KAL_N_STATE, 1, (float32_t *)kal_xp_f32);
arm_mat_init_f32(&kal_Pp, KAL_N_STATE, KAL_N_STATE, (float32_t *)kal_Pp_f32);
//arm_mat_init_f32(&kal_xp, KAL_N_STATE, 1, (float32_t *)kal_xp_f32);
arm_mat_init_f32(&kal_Ad, KAL_N_STATE, KAL_N_STATE, (float32_t *)kal_Ad_f32);
arm_mat_init_f32(&kal_Bd, KAL_N_STATE, 1, (float32_t *)kal_Bd_f32);
arm_mat_init_f32(&kal_C, KAL_N_INPUT, KAL_N_STATE, (float32_t *)kal_C_f32);
arm_mat_init_f32(&kal_Kk, KAL_N_STATE, KAL_N_INPUT, (float32_t *)kal_Kk_f32);
arm_mat_init_f32(&inv_Kk, KAL_N_INPUT, KAL_N_STATE, (float32_t *)inv_Kk_f32);
arm_mat_init_f32(&kal_y, KAL_N_INPUT, 1, (float32_t *)kal_y_f32);
arm_mat_init_f32(&result32, 3, 2, (float32_t *)result32_f32);
arm_mat_init_f32(&result23, 2, 3, (float32_t *)result23_f32);
arm_mat_init_f32(&result22, 2, 2, (float32_t *)result22_f32);
arm_mat_init_f32(&result22_2, 2, 2, (float32_t *)result22_2_f32);
arm_mat_init_f32(&result33, 3, 3, (float32_t *)result33_f32);
arm_mat_init_f32(&result33_2, 3, 3, (float32_t *)result33_2_f32);
arm_mat_init_f32(&result21, 2, 1, (float32_t *)result21_f32);
arm_mat_init_f32(&result21_2, 2, 1, (float32_t *)result21_2_f32);
arm_mat_init_f32(&result31, 3, 1, (float32_t *)result31_f32);
arm_mat_init_f32(&inv_C, KAL_N_STATE, KAL_N_INPUT, (float32_t *)inv_C_f32);
arm_mat_init_f32(&eye3, 3, 3, (float32_t *)eye3_f32);
 }


 void kalman_update(){

    //Kk = Pm*C.'/(C*Pm*C.' + R);
    arm_mat_mult_f32(&kal_Pm, &inv_C, &result32);   //Pm*C.'
    arm_mat_mult_f32(&kal_C,&result32,&result22);   //C*Pm*C.'
    arm_mat_add_f32(&result22,&kal_R,&result22_2);
    arm_mat_inverse_f32(&result22_2,&result22);
    arm_mat_mult_f32(&result32,&result22,&kal_Kk);
    //xp = xm + Kk*(yi- C*xm);
    arm_mat_mult_f32(&kal_C,&kal_xm,&result21);
    // printf("1, %d\n", err);
    arm_mat_sub_f32(&kal_y,&result21,&result21_2);
    // printf("2, %d\n", err);
    arm_mat_mult_f32(&kal_Kk,&result21_2,&result31);
    // printf("3, %d\n", err);
    arm_mat_add_f32(&kal_xm,&result31,&kal_xp);
          //  printf("4, %d\n", err);
    // printf("%f\n", kal_xp_f32[0]);
    // printf("%f\n", kal_xp_f32[1]);
    // printf("%f\n", kal_xp_f32[2]);

    //Pp = (eye(3) - Kk*C)*Pm*(eye(3) - Kk*C).' + Kk*R*Kk.';
    arm_mat_mult_f32(&kal_Kk, &kal_C, &result33);
    arm_mat_sub_f32(&eye3, &result33, &result33_2);         //(eye(3) - Kk*C)
    arm_mat_trans_f32(&result33_2,&kal_Pp);
    arm_mat_mult_f32(&kal_Pm, &kal_Pp, &result33);
    arm_mat_mult_f32(&result33_2, &result33, &kal_Pp);      //(eye(3) - Kk*C)*Pm*(eye(3) - Kk*C).'
    arm_mat_trans_f32(&kal_Kk,&inv_Kk);                     // Kk.'
    arm_mat_mult_f32(&kal_R, &inv_Kk, &result23);           //R*Kk.'
    arm_mat_mult_f32(&kal_Kk, &result23, &result33);
    arm_mat_add_f32(&kal_Pp,&result33,&kal_Pp);
    
    //predict next state  xm = Ad*xp;
    arm_mat_mult_f32(&kal_Ad,&kal_xp,&kal_xm);

    //compute prediction error Pm = Ad*Pp*Ad.' + Q;
    arm_mat_trans_f32(&kal_Ad,&result33);
    arm_mat_mult_f32(&kal_Pp,&result33,&kal_Pm);
    arm_mat_mult_f32(&kal_Ad,&kal_Pm,&result33);
    arm_mat_add_f32(&result33,&kal_Q,&kal_Pm);
 }

 float getKalmanAngle(void)
{
   return kal_xp_f32[1];

}
 float getKalmanVelocity(void)
{       
    return kal_xp_f32[0];
}
 float getKalmanBias(void)
{       
    return kal_xp_f32[2];
     //   return kal_y_f32[1];
}
 void kalman_set_angle(float angle)
{
// Update state x1
    kal_y_f32[0] = angle;
}

void kalman_set_velocity(float angvel)
{
// Update state x2
    kal_y_f32[1] = angvel;
}


