/*****************************************************************************
 * pid.c - Proportional-Integral-Derivative (PID) controller
 * Initial Author: Harleen Sandhu / Mehul Williams
 * Additional Author: Shaun Gilmore
 ******************************************************************************
 * General purpose PID controller, initially designed for Torque Vectoring.
 * 
 * The PID controller works by using three terms to calculate an output value that is used to control a system. The three terms are:
 * Proportional: This term is proportional to the error between the target and current values. It is multiplied by a constant gain value (Kp) that determines how much the controller responds to changes in the error.
 * Integral: This term is proportional to the running sum of errors over time. It is multiplied by a constant gain value (Ki) that determines how much the controller responds to steady-state errors.
 * Derivative: This term is proportional to the rate of change of the error. It is multiplied by a constant gain value (Kd) that determines how much the controller responds to changes in the rate of change of the error.
 * By adjusting the values of the three gain constants (Kp, Ki, and Kd), the controller can be tuned to respond differently to changes in the error, steady-state errors, and changes in the rate of change of the error.
 * Generally, higher values of Kp will lead to faster response to changes in the error, while higher values of Ki will lead to faster response to steady-state errors, and higher values of Kd will lead to faster response to changes in the rate of change of the error.
 * Conversion between SlipR and Torque -> Kp
 * Proportional test first with other output 0, get midway with target and then tune other items. There are many factors of noise.
 * Kp will give you the difference between 0.1 current vs 0.2 target -> if you want to apply 50nm if your error is 0.1 then you need 500 for Kp to get target
 ****************************************************************************/
#include <stdlib.h>
#include "PID.h"

PID* PID_new(int Kp, int Ki, int Kd, int setpoint) {
    // for some reason the kp ki kd values are not updated correctly so we reinit them 
    // Kp Ki & Kd are in deci units, aka a Kp of 12 is actually 1.2.
    PID* pid = (PID*)malloc(sizeof(PID));
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->setpoint      = setpoint; 
    pid->previousError = 0;
    pid->totalError    = 0;
    pid->dH            = 100; // 100 Hz aka 10 ms cycle time
    pid->output        = 0;
    return pid;
}
void PID_updateGainValues(PID* pid, int Kp, int Ki, int Kd){
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
}

void PID_setTotalError(PID* pid, int error){
    pid->totalError = error;
}

void PID_updateSetpoint(PID *pid, int setpoint) {
    pid->setpoint = setpoint; 
}

int PID_computeOutput(PID *pid, int sensorValue) {
    int currentError =  pid->setpoint - sensorValue;
    int proportional =  (pid->Kp * currentError) / 10; //proportional
    int integral     =  (pid->Ki * (pid->totalError + currentError) / pid->dH) / 10; //integral
    int derivative   =  (pid->Kd * (currentError - pid->previousError) * pid->dH) / 10; //derivative
    pid->previousError  = currentError;
    pid->totalError    += currentError;
    pid->output = (int)(proportional + integral + derivative); //divide by 10 because Kp & Ki & Kd are in deci- units to preserve complete accuracy for the tenth's place
    return pid->output;
}

/** GETTER FUNCTIONS **/

int PID_getKp(PID *pid){
    return pid->Kp;
}

int PID_getKi(PID *pid){
    return pid->Ki;
}

int PID_getKd(PID *pid){
    return pid->Kd;
}

int PID_getSetpoint(PID *pid){
    return pid->setpoint;
}

int PID_getTotalError(PID* pid){
    return pid->totalError;
}

int PID_getOutput(PID* pid){
    return pid->output;
}