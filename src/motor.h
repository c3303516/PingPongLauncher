#ifndef MOTOR_H
#define MOTOR_H

/* Add function prototypes here */

 void motor_PWM_init(void);
 void motor_encoder_init(void);
 void EXTI3_IRQHandler(void);
//  void EXTI1_IRQHandler(void);
 int32_t motor_encoder_getValue(void);
 float motor_encoder_getAngle(int32_t);
 float motor_encoder_getRev(int32_t);
//  void direction_adjust(float);
//  int microstep(float v);
// void comms_loop_init(void)
void velocity_adjust(float);
void elevation_adjust(float);

#endif