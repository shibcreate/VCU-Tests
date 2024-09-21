
#include "IO_Driver.h" //Includes datatypes, constants, etc - should be included in every c file
#include "motorController.h"
#include "PID.h"
#include "hashTable.h"
#include "powerLimit.h"
#include "bms.h"
#include "wheelSpeeds.h"
#include "torqueEncoder.h"
#include "math.h"

#define PL_INIT 70000.0

PowerLimit* PL_new(){
    PowerLimit* me = (PowerLimit*)malloc(sizeof(PowerLimit));
    me->plStatus  = FALSE;
    me->watts      = 0.0;
    me->offset     = 0.0;
    return me;
}

void PL_calculateTorqueOffset(MotorController* mcm, PowerLimit* me, PID* plPID){
    me->watts = (float)MCM_getPower(mcm);
    if(me->watts > PL_INIT) {
        me->plStatus       = TRUE;
        int maxTQ       = MCM_getTorqueMax(mcm);
        sbyte2 commandedTQ = MCM_commands_PL_getTorque(me);
        me->offset         = PID_computeOffset(plPID, me->watts);
        ubyte2 offsetTQ    = (ubyte2)commandedTQ * (1 + (ubyte2)(maxTQ) ((ubyte2)(me->offset / me->watts * 100))); //offsetTQ is the complete torque request
        MCM_updateTorqueOffset(mcm, offsetTQ);
    }
    else { me->plStatus    = FALSE; }
    MCM_updatePowerLimitState(mcm, me->plStatus);
}