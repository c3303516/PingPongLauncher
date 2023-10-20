#ifndef KALMAN_H
#define KALMAN_H

/* Add function prototypes here */
void kalman_init(void);
void kalman_update(void);
float getKalmanAngle(void);
float getKalmanVelocity(void);
float getKalmanBias(void);
void kalman_set_angle(float);
void kalman_set_velocity(float);
enum {
 KAL_N_INPUT = 2, // number of kalman inputs (measurement signals)
 KAL_N_STATE = 3, // number of kalman states (states)
 };

#endif