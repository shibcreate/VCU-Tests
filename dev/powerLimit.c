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

#define VOLTAGE_MIN      (int) 280
#define VOLTAGE_MAX      (int) 403//not needed
#define RPM_MIN          (int) 2000
#define RPM_MAX          (int) 6000
#define NUM_V            (int) 25
#define NUM_S            (int) 25
#define VOLTAGE_STEP     (int) 5     //float voltageStep = (Voltage_MAX - Voltage_MIN) / (NUM_V - 1); // 5
#define RPM_STEP         (int) 160 //sbyte4 rpmStep = (RPM_MAX - RPM_MIN) / (NUM_S - 1); // 245.8333
#define PI               (float) 3.14159
#define KWH_LIMIT        (int) 55000  // watts
#define PL_INIT          (int) 55000 // 5kwh buffer to init PL before PL limit is hit
#define UNIT_CONVERSTION (float) 95.49    // 9.549 *10.0 to convert to deci-newtonmeters
#define KWH_THRESHOLD    (float) 50

PowerLimit* PL_new(){
    PowerLimit* me = (PowerLimit*)malloc(sizeof(PowerLimit));
     me->value=0;

    me->PLStatus = FALSE;
    me->LUTval=0;
    me->setpoint=0;
    me->actual= 0;
    me->pltorque= 0;
    me->piderror= 0;
    me->PLMethod = FALSE;
   
    return me;
    }

void testing(PowerLimit *me){
    me->PLStatus = TRUE;
    me->value = 1000;
}

void powerLimitTorqueCalculation_1(PowerLimit *me,  MotorController* mcm, PID* pid){
  float gain = 95.49; //for decinm
  sbyte4 watts = MCM_getPower(mcm);
  int wheelspeed = MCM_getMotorRPM(mcm);
  if(watts > 55000-10){
    me-> PLStatus = TRUE;
    me->PLMethod = FALSE;
    ubyte2 pidsetpoint = (ubyte2)((55000*gain/wheelspeed));
    me->setpoint =pidsetpoint;
    ubyte2 pidactual = (ubyte2)((55000*gain/wheelspeed));
    me->actual= pidactual;
    PID_setpointUpdate(pid,pidsetpoint);
    sbyte2 PIDerror = PID_compute(pid, pidactual);
    me->piderror = PIDerror;
    ubyte2 PLfinaltq = (ubyte2)(pidactual+ PIDerror);
    me->pltorque= PLfinaltq;
  }
  else{
    me->PLStatus= FALSE;
  }
  MCM_update_PL_TorqueLimit(mcm,  me->pltorque); // we need to change this on mcm.c / pl.c/.h 
  MCM_update_PL_State(mcm, me->PLStatus);
}
void fillhashtable(HashTable *table){
    const int lookupTable[26][26] = {
   
    };
     for (int row = 0; row < NUM_S; ++row) {
        for(int column = 0; column < NUM_V; ++column) {
            int noLoadVoltage = VOLTAGE_MIN + column * VOLTAGE_STEP;
            int rpm   = RPM_MIN + row * RPM_STEP;
            int value = lookupTable[(int)row][(int)column];
            insert(table, noLoadVoltage, rpm, value);
        }
    }
}

ubyte2 getTorque(PowerLimit* me, HashTable* hashtable, int voltage, int rpm){ 
    int voltageFloor      = int_lowerStepInterval(voltage,5);
    int voltageCeiling    = int_upperStepInterval(voltage,5);
    int rpmFloor          = int_lowerStepInterval(rpm,160);
    int rpmCeiling        = int_upperStepInterval(rpm,160);
    
    // Calculating these now to speed up interpolation later in method

    // Retrieve torque values from the hash table for the four corners
    int vFloorRFloor      = get(hashtable, voltageFloor, rpmFloor);
    int vFloorRCeiling    = get(hashtable, voltageFloor, rpmCeiling);
    int vCeilingRFloor    = get(hashtable, voltageCeiling, rpmFloor);
    int vCeilingRCeiling  = get(hashtable, voltageCeiling, rpmCeiling);

    // Early escape in case values are the same. May want to make more complex for scenarios such as 2 of the values are the same.
    if(vFloorRFloor == vFloorRCeiling && vCeilingRFloor == vCeilingRCeiling)
    {
        return vFloorRFloor;
    }

    
    int horizontal_Interp = (((vCeilingRFloor - vFloorRFloor) / 5) + ((vCeilingRCeiling - vFloorRCeiling) / 5)) / 2;
    int vertical_Interp = (((vFloorRCeiling - vFloorRFloor) / 160) + ((vCeilingRCeiling - vCeilingRFloor) / 160)) / 2;
    // Calculate interpolation values
   int gainValueHoriz = voltage % 5;
   int gainValueVertical = rpm % 160;

    // Final TQ from LUT
    int TQ =  (gainValueHoriz * horizontal_Interp) + (gainValueVertical * vertical_Interp) + vFloorRFloor;
    ubyte2 interptq = (ubyte2)(TQ);
   return interptq;
}

void powerLimitTorqueCalculation_2(PowerLimit *me,  MotorController* mcm, PID* pid, HashTable *table){
float gain = 95.49; //for decinm
  sbyte4 watts = MCM_getPower(mcm);
  int wheelspeed = MCM_getMotorRPM(mcm);
sbyte4 mcmVoltage = MCM_getDCVoltage(mcm);
sbyte4 mcmCurrent = MCM_getDCCurrent(mcm);
ubyte2 commandedTorque = MCM_getCommandedTorque(mcm);

sbyte4 noLoadVoltage = (mcmCurrent * 27 / 1000 ) + mcmVoltage; // 27 / 100 (0.027) is the estimated IR. Should attempt to revalidate on with new powerpack.
  if(watts > 55000-10){
    me-> PLStatus = TRUE;
    me-> PLMethod = TRUE;

    ubyte2 pidsetpoint = (ubyte2)(getTorque(me, table, noLoadVoltage, wheelspeed));

    if(pidsetpoint == -1)
    {
      me-> PLMethod = FALSE;
      pidsetpoint = (ubyte2)((55000*gain/wheelspeed));

    }
    me->setpoint = pidsetpoint;

    ubyte2 pidactual = commandedTorque;
    me->actual= pidactual;

    PID_setpointUpdate(pid,pidsetpoint);
    sbyte2 PIDerror = PID_compute(pid, pidactual);
    me->piderror = PIDerror;
    ubyte2 PLfinaltq = (ubyte2)(pidactual+ PIDerror);
    me->pltorque= PLfinaltq;
  }
  else{
    me->PLStatus= FALSE;
  }
  MCM_update_PL_TorqueLimit(mcm,  me->pltorque); // we need to change this on mcm.c / pl.c/.h 
  MCM_update_PL_State(mcm, me->PLStatus);
}