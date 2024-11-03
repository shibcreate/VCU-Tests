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
#define NUM_V            (int) 26
#define NUM_S            (int) 26
#define VOLTAGE_STEP     (int) 5     //float voltageStep = (Voltage_MAX - Voltage_MIN) / (NUM_V - 1); // 5
#define RPM_STEP         (int) 160 //sbyte4 rpmStep = (RPM_MAX - RPM_MIN) / (NUM_S - 1); // 245.8333
#define PI               (float) 3.14159
#define KWH_LIMIT        (int) 60000  // watts
#define PL_INIT          (int) 50000 // 5kwh buffer to init PL before PL limit is hit
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
  if(watts > 50000){
    me-> PLStatus = TRUE;
    me->PLMethod = FALSE;
    ubyte2 pidsetpoint = (ubyte2)((58000*gain/wheelspeed));
    me->setpoint =pidsetpoint;
    ubyte2 pidactual = (ubyte2)((watts*gain/wheelspeed));
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
     {2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309},
                {2257, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309},
                {2084, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309},
                {1912, 2208, 2228, 2234, 2237, 2238, 2238, 2238, 2238, 2238, 2238, 2238, 2238, 2238, 2238, 2238, 2238, 2238, 2238, 2238, 2238, 2238, 2238, 2238, 2238, 2238},
                {1754, 2061, 2098, 2104, 2108, 2108, 2110, 2110, 2109, 2109, 2109, 2109, 2109, 2109, 2109, 2109, 2109, 2109, 2109, 2109, 2109, 2109, 2109, 2109, 2109, 2109},
                {1596, 1918, 1983, 1987, 1992, 1993, 1994, 1995, 1994, 1996, 1996, 1996, 1996, 1996, 1996, 1996, 1996, 1996, 1996, 1996, 1996, 1996, 1996, 1996, 1996, 1996},
                {1475, 1783, 1878, 1883, 1887, 1889, 1891, 1892, 1892, 1894, 1890, 1890, 1890, 1890, 1890, 1890, 1890, 1890, 1890, 1890, 1890, 1890, 1890, 1890, 1890, 1890},
                {1331, 1643, 1773, 1789, 1792, 1796, 1799, 1798, 1799, 1799, 1798, 1799, 1799, 1799, 1799, 1799, 1799, 1799, 1799, 1799, 1799, 1799, 1799, 1799, 1799, 1799},
                {1216, 1513, 1682, 1704, 1707, 1709, 1711, 1713, 1714, 1714, 1714, 1712, 1712, 1712, 1712, 1712, 1712, 1712, 1712, 1712, 1712, 1712, 1712, 1712, 1712, 1712},
                {1102, 1394, 1585, 1625, 1629, 1631, 1632, 1635, 1636, 1636, 1638, 1638, 1637, 1637, 1637, 1637, 1637, 1637, 1637, 1637, 1637, 1637, 1637, 1637, 1637, 1637},
                {992, 1292, 1465, 1552, 1556, 1559, 1561, 1562, 1564, 1565, 1566, 1564, 1567, 1568, 1567, 1567, 1567, 1567, 1567, 1567, 1567, 1567, 1567, 1567, 1567, 1567},
                {879, 1184, 1367, 1477, 1491, 1494, 1496, 1497, 1499, 1500, 1501, 1503, 1501, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502},
                {763, 1079, 1265, 1399, 1431, 1433, 1432, 1437, 1445, 1439, 1440, 1441, 1441, 1442, 1442, 1443, 1443, 1443, 1443, 1443, 1443, 1443, 1443, 1443, 1443, 1443},
                {654, 980, 1165, 1303, 1369, 1376, 1379, 1381, 1382, 1383, 1384, 1385, 1386, 1386, 1387, 1387, 1387, 1388, 1387, 1387, 1387, 1387, 1387, 1387, 1387, 1387},
                {536, 881, 1070, 1205, 1316, 1325, 1328, 1329, 1332, 1332, 1333, 1334, 1334, 1334, 1335, 1335, 1336, 1337, 1336, 1337, 1337, 1337, 1337, 1337, 1337, 1337},
                {403, 783, 975, 1126, 1246, 1274, 1280, 1281, 1282, 1284, 1285, 1286, 1285, 1286, 1287, 1287, 1287, 1288, 1286, 1289, 1289, 1289, 1289, 1289, 1289, 1289},
                {231, 684, 885, 1032, 1158, 1226, 1235, 1236, 1237, 1239, 1239, 1239, 1241, 1241, 1242, 1243, 1243, 1244, 1243, 1243, 1244, 1245, 1247, 1247, 1247, 1247},
                {  1, 579, 793, 946, 1070, 1177, 1192, 1185, 1196, 1196, 1197, 1199, 1200, 1200, 1202, 1200, 1202, 1203, 1202, 1203, 1202, 1203, 1202, 1202, 1202, 1202},
                {  1, 466, 700, 858, 987, 1095, 1149, 1156, 1156, 1157, 1158, 1164, 1160, 1161, 1162, 1161, 1163, 1163, 1161, 1164, 1163, 1164, 1164, 1164, 1164, 1165},
                {  1, 335, 603, 772, 902, 1014, 1109, 1116, 1120, 1120, 1121, 1122, 1123, 1123, 1124, 1125, 1125, 1126, 1127, 1126, 1127, 1127, 1127, 1127, 1127, 1128},
                {  1, 147, 500, 684, 816, 935, 1033, 1075, 1084, 1085, 1087, 1087, 1089, 1089, 1090, 1090, 1090, 1092, 1089, 1092, 1094, 1093, 1098, 1093, 1094, 1092},
                {  1,   1, 384, 591, 736, 855, 959, 1040, 1051, 1053, 1054, 1056, 1056, 1056, 1057, 1058, 1058, 1058, 1059, 1059, 1060, 1060, 1060, 1061, 1061, 1061},
                {  1,   1, 239, 493, 650, 776, 880, 973, 1016, 1022, 1023, 1024, 1026, 1026, 1026, 1027, 1027, 1028, 1028, 1028, 1029, 1028, 1030, 1030, 1030, 1030},
                {  1,   1,   1, 383, 562, 693, 805, 901, 982, 991, 994, 995, 996, 998, 997, 998, 999, 1000, 1000, 1003, 999, 1001, 1001, 1001, 1001, 1000},
                {  1,   1,   1, 244, 466, 611, 727, 825, 913, 960, 967, 968, 968, 969, 970, 970, 971, 971, 971, 972, 973, 973, 973, 974, 953, 975},
                {  1,   1,   1,   1, 356, 522, 649, 753, 841, 927, 938, 942, 942, 943, 944, 944, 945, 942, 946, 946, 935, 949, 947, 948, 952, 948}};
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
  if(watts > 55000){
    me-> PLStatus = TRUE;
    me-> PLMethod = TRUE;

    ubyte2 pidsetpoint = (ubyte2)(getTorque(me, table, noLoadVoltage, wheelspeed));

    if(pidsetpoint == -1)
    {
      me-> PLMethod = FALSE;
      pidsetpoint = (ubyte2)((58000*gain/wheelspeed));

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