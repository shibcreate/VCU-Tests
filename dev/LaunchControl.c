#include <stdlib.h>
#include <math.h>
#include "IO_RTC.h"
#include "IO_DIO.h"
#include "launchControl.h"
#include "wheelSpeeds.h"
#include "mathFunctions.h"
#include "initializations.h"
#include "sensors.h"
#include "torqueEncoder.h"
#include "brakePressureSensor.h"
#include "motorController.h"
#include "sensorCalculations.h"
#include "PID.h"
#include "IO_Driver.h" //Includes datatypes, constants, etc - should be included in every c file

extern Sensor Sensor_LCButton;
extern Sensor Sensor_DRSKnob;

LaunchControl *LaunchControl_new(){
    LaunchControl* me = (LaunchControl*)malloc(sizeof(struct _LaunchControl));
    // malloc returns NULL if it fails to allocate memory
    if (me == NULL)
        return NULL;
    me->pid = PID_new(200, 0, 0, 0); //No saturation point to see what the behavior of the PID is, will need a saturation value somewhere to prevent wind-up of the pid in the future
    PID_updateSetpoint(me->pid, 20); // Having a statically coded slip ratio may not be the best. this requires knowing that this is both a) the best slip ratio for the track, and b) that our fronts are not in any way slipping / entirely truthful regarding the groundspeed of the car. Using accel as a target is perhaps better, but needs to be better understood.
    me->slipRatio = 0;
    me->lcTorqueCommand = NULL;
    me->lcReady = FALSE;
    me->lcActive = FALSE;
    me->buttonDebug = 0;
    return me;
}


void LaunchControl_calculateSlipRatio(LaunchControl *me, WheelSpeeds *wss){
    me->slipRatio = (WheelSpeeds_getSlowestFront(wss) / (WheelSpeeds_getFastestRear(wss))) - 1;
    if (me->slipRatio > 1.0) {
        me->slipRatio = 1.0;
    }
    if (me->slipRatio < -1.0) {
        me->slipRatio = -1.0;
    }
}

void LaunchControl_calculateTorqueCommand(LaunchControl *me, TorqueEncoder *tps, BrakePressureSensor *bps, MotorController *mcm){
    if(me->lcActive){ //By doing this in combination with calling checkState after this function, we introduce a 10 ms delay. FIX logic
        me->slipRatioThreeDigits = (sbyte2) (me->slipRatio * 100);
        PID_computeOutput(me->pid, me->slipRatioThreeDigits);
        me->lcTorqueCommand = MCM_getCommandedTorque(mcm) + PID_getOutput(me->pid); // adds the ajusted value from the pid to the torqueval}

        if(MCM_getGroundSpeedKPH(mcm) < 3){
            me->lcTorqueCommand = 20;
        }

        // Update launch control torque command in mcm struct
        MCM_update_LC_torqueLimit(mcm, me->lcTorqueCommand * 10); // Move the mul by 10 to within MCM struct at some point
    }
}

void LaunchControl_checkState(LaunchControl *me, TorqueEncoder *tps, BrakePressureSensor *bps, MotorController *mcm){
    sbyte2 speedKph         = MCM_getGroundSpeedKPH(mcm);
    sbyte2 steeringAngle    = steering_degrees();

    /* LC STATUS CONDITIONS *//*
     * lcReady = FALSE && lcActive = FALSE -> NOTHING HAPPENS
     * lcReady = TRUE  && lcActive = FALSE -> We are in the prep stage for lc, and all entry conditions for being in prep stage have and continue to be monitored
     * lcReady = FALSE && lcActive = TRUE  -> We have left the prep stage by pressing the lc button on the steering wheel, stay in until exit conditions are met
     * AT ALL TIMES, EXIT CONDITIONS ARE CHECKED FOR BOTH STATES
    */

    // SENSOR_LCBUTTON values are reversed: FALSE = TRUE and TRUE = FALSE, due to the VCU internal Pull-Up for the button and the button's Pull-Down on Vehicle
    if(Sensor_LCButton.sensorValue == TRUE && speedKph < 5) {
        me->lcReady = TRUE;
    }

    else if(me->lcReady == TRUE && Sensor_LCButton.sensorValue == FALSE){
        PID_setTotalError(me->pid, 170); // Error should be set here, so for every launch we reset our error to this value (check if this is the best value)
        me->lcTorqueCommand = 0; // On the motorcontroller side, this torque should stay this way regardless of the values by the pedals while LC is ready
        me->lcActive = TRUE;
        me->lcReady = FALSE;
    }

    else if(bps->percent > .35 || steeringAngle > 35 || steeringAngle < -35){
        me->lcReady = FALSE;
    }

    if(tps->travelPercent < 0.90 || bps->percent > 0.05){
        me->lcActive = FALSE;
        me->lcTorqueCommand = NULL;
    }
    //MCM struct only cares about lcActive, so we inform it here
    MCM_update_LaunchControl_state(mcm, me->lcActive);
}

bool LaunchControl_getStatus(LaunchControl *me){ return me->lcActive; }

sbyte2 LaunchControl_getTorqueCommand(LaunchControl *me){ return me->lcTorqueCommand; }

float LaunchControl_getSlipRatio(LaunchControl *me){ return me->slipRatio; }

sbyte2 LaunchControl_getSlipRatioThreeDigits(LaunchControl *me){ return me->slipRatioThreeDigits; }

ubyte1 LaunchControl_getButtonDebug(LaunchControl *me) { return me->buttonDebug; }