#ifndef _POWERLIMIT_H
#define _POWERLIMIT_H
#include "IO_Driver.h" 
#include "motorController.h"
#include "powerLimit.h"
#include "bms.h"
#include "wheelSpeeds.h"
#include "torqueEncoder.h"
#include "mathFunctions.h"
#include "initializations.h"
#include "PID.h"
#include "hashTable.h"


typedef struct _PowerLimit {
     ubyte2 value;
     ubyte2 LUTval;
    bool PLStatus;
    ubyte2 setpoint;
    ubyte2 actual;
    ubyte2 pltorque;
    sbyte2 piderror;

    bool PLMethod; // 0 is tq method, 1 is lut method

}PowerLimit;

PowerLimit* PL_new();
void testing(PowerLimit *me);
void powerLimitTorqueCalculation_1(PowerLimit *me,  MotorController* mcm, PID* pid);
void powerLimitTorqueCalculation_2(PowerLimit *me,  MotorController* mcm, PID* pid, HashTable *table);
void fillhashtable(HashTable *table);
ubyte2 getTorque(PowerLimit* me, HashTable* hashtable, int voltage, int rpm);

#endif