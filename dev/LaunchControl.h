#ifndef _LAUNCHCONTROL_H
#define _LAUNCHCONTROL_H

#include "IO_Driver.h"
#include "wheelSpeeds.h"
#include "mathFunctions.h"
#include "initializations.h"
#include "sensors.h"
#include "torqueEncoder.h"
#include "brakePressureSensor.h"
#include "motorController.h"

typedef struct _PIDController {
    float kp;         // Proportional gain
    float ki;         // Integral gain
    float kd;         // Derivative gain
    float errorSum;   // Running sum of errors for the integral term
    float lastError;  // Previous error for the derivative term
} PIDController;

typedef struct _LaunchControl {
    float slipRatio;
    ubyte2 lcTorque;
    bool LCReady;
    bool LCStatus; // Just for CAN to showcase when enabled
    PIDController *pidController;
    ubyte1 potLC;

    ubyte1 buttonDebug;
} LaunchControl;

LaunchControl *LaunchControl_new();
void slipRatioCalculation(WheelSpeeds *wss, LaunchControl *lc);
void launchControlTorqueCalculation(LaunchControl *lc, TorqueEncoder *tps, BrakePressureSensor *bps, MotorController *mcm);
bool getLaunchControlStatus(LaunchControl *lc);
sbyte2 getCalculatedTorque();
ubyte1 getButtonDebug(LaunchControl *lc);

#endif