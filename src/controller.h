#ifndef CONTROLLER_H
#define CONTROLLER_H

/* Add function prototypes here */
void ctrl_init(void);
float getControl(void);
void ctrl_update(void);
void ctrl_set_x1(float);
void ctrl_set_x2(float);
void ctrl_set_yref(float);
float getReference(void);

enum {
 CTRL_N_INPUT = 1, // number of controller inputs (reference signals)
 CTRL_N_STATE = 3, // number of controller states (states) + integrator
 CTRL_N_OUTPUT = 1, // number of controller outputs / plant inputs
 CTRL_N_HORIZON = 10, // control horizon length
 CTRL_N_EQ_CONST = 0, // number of equality constraints
 CTRL_N_INEQ_CONST = 20, // number of inequality constraints
 CTRL_N_LB_CONST = 10, // number of lower bound constraints
 CTRL_N_UB_CONST = 10, // number of upper bound constraints
 };

#endif