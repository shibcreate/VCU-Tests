
#include "IO_Driver.h" //Includes datatypes, constants, etc - should be included in every c file
#include "motorController.h"
#include "PID.h"
#include "hashTable.h"
#include "powerLimit.h"
#include "bms.h"
#include "wheelSpeeds.h"
#include "torqueEncoder.h"
#include "mathFunctions.h"

#ifndef CALCS
#define CALCS

#define VOLTAGE_MIN      (int) 280
#define VOLTAGE_MAX      (int) 403//not needed
#define RPM_MIN          (int) 2000
#define RPM_MAX          (int) 6000
#define NUM_V            (int) 25
#define NUM_S            (int) 25
#define VOLTAGE_STEP     (int) 5     //float voltageStep = (Voltage_MAX - Voltage_MIN) / (NUM_V - 1); // 5
#define RPM_STEP         (int) 160 //sbyte4 rpmStep = (RPM_MAX - RPM_MIN) / (NUM_S - 1); // 245.8333
#define PI               (float) 3.14159
#define KWH_LIMIT        (float) 55000.0  // watts
#define PL_INIT          (float) 55000.0  // 5kwh buffer to init PL before PL limit is hit
#define UNIT_CONVERSTION (float) 95.49    // 9.549 *10.0 to convert to deci-newtonmeters
#define KWH_THRESHOLD    (float) 50

#endif

void populatePLHashTable(HashTable* table)
{
    /*
    voltage is x axis
    rpm is y axis 
    values are in tq nM
    */
    int lookupTable[26][26] = {
	{2309, 2309, 1988, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309},
	{2216, 2294, 1805, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309},
	{2053, 2141, 1613, 2280, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309},
	{1870, 1977, 1463, 2140, 2214, 2270, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309},
	{1723, 1804, 1303, 1982, 2052, 2140, 2213, 2257, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309},
	{1573, 1658, 1165, 1821, 1892, 1984, 2052, 2127, 2184, 2256, 2308, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309},
	{1437, 1521, 1031, 1683, 1753, 1826, 1896, 1992, 2052, 2127, 2184, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309},
	{1308, 1377, 898, 1536, 1611, 1699, 1772, 1843, 1928, 1994, 2065, 2215, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309},
	{1183, 1259, 765, 1415, 1500, 1573, 1642, 1715, 1786, 1855, 1921, 2084, 2215, 2264, 2265, 2268, 2269, 2271, 2269, 2272, 2271, 2273, 2273, 2273, 2273, 2273},
	{1064, 1140, 630, 1308, 1383, 1454, 1531, 1594, 1667, 1730, 1805, 1977, 2087, 2159, 2163, 2166, 2168, 2169, 2168, 2165, 2171, 2172, 2172, 2172, 2172, 2172},
	{946, 1036, 484, 1195, 1273, 1331, 1403, 1476, 1553, 1621, 1689, 1846, 1993, 2053, 2069, 2069, 2074, 2076, 2077, 2078, 2077, 2079, 2081, 2081, 2077, 2077},
	{839, 927, 316, 1087, 1164, 1229, 1308, 1375, 1440, 1509, 1574, 1738, 1857, 1965, 1981, 1986, 1988, 1989, 1990, 1992, 1994, 1994, 1994, 1995, 1994, 1993},
	{727, 813, 1, 976, 1057, 1132, 1205, 1274, 1342, 1395, 1467, 1637, 1762, 1857, 1904, 1907, 1909, 1910, 1912, 1913, 1914, 1915, 1915, 1915, 1916, 1916},
	{610, 707, 1, 880, 953, 1032, 1103, 1172, 1244, 1308, 1378, 1534, 1660, 1772, 1818, 1833, 1835, 1837, 1840, 1839, 1840, 1842, 1841, 1842, 1842, 1843},
	{484, 594, 1, 773, 858, 936, 1007, 1077, 1147, 1203, 1277, 1438, 1566, 1659, 1745, 1766, 1767, 1768, 1770, 1771, 1774, 1773, 1774, 1775, 1775, 1774},
	{339, 472, 1, 673, 757, 837, 911, 987, 1049, 1115, 1194, 1347, 1475, 1574, 1697, 1700, 1705, 1706, 1707, 1707, 1709, 1709, 1711, 1711, 1711, 1711},
	{119, 326, 1, 564, 657, 742, 816, 893, 962, 1031, 1102, 1257, 1384, 1497, 1586, 1635, 1644, 1646, 1647, 1648, 1650, 1650, 1651, 1652, 1653, 1652},
	{1, 102, 1, 445, 550, 641, 724, 801, 876, 942, 1019, 1168, 1294, 1395, 1501, 1565, 1586, 1591, 1592, 1595, 1594, 1595, 1596, 1597, 1597, 1598},
	{1, 1, 1, 302, 432, 537, 628, 708, 783, 856, 931, 1091, 1205, 1321, 1407, 1509, 1531, 1539, 1541, 1542, 1543, 1544, 1544, 1545, 1546, 1546},
	{1, 1, 1, 61, 290, 420, 524, 614, 693, 767, 847, 1007, 1133, 1245, 1340, 1418, 1477, 1491, 1492, 1495, 1494, 1495, 1496, 1496, 1497, 1498},
	{1, 1, 1, 1, 36, 279, 409, 511, 599, 679, 762, 928, 1054, 1165, 1263, 1348, 1422, 1439, 1446, 1448, 1448, 1449, 1450, 1451, 1452, 1416},
	{1, 1, 1, 1, 1, 1, 270, 397, 500, 588, 675, 847, 979, 1091, 1187, 1267, 1361, 1392, 1403, 1403, 1405, 1406, 1407, 1408, 1408, 1411},
	{1, 1, 1, 1, 1, 1, 1, 258, 386, 489, 587, 767, 902, 1009, 1113, 1201, 1277, 1344, 1359, 1364, 1365, 1366, 1367, 1368, 1368, 1368},
	{1, 1, 1, 1, 1, 1, 1, 1, 248, 378, 492, 683, 825, 942, 1039, 1131, 1213, 1291, 1317, 1324, 1327, 1328, 1329, 1329, 1329, 1330},
	{1, 1, 1, 1, 1, 1, 1, 1, 1, 238, 384, 602, 752, 870, 968, 1058, 1145, 1212, 1273, 1288, 1290, 1292, 1291, 1292, 1293, 1293},
	{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 249, 513, 673, 797, 901, 993, 1074, 1151, 1217, 1247, 1256, 1257, 1258, 1259, 1258, 1259}};

    for (int row = 0; row < NUM_S; ++row) {
        for(int column = 0; column < NUM_V; ++column) {
            int noLoadVoltage = VOLTAGE_MIN + column * VOLTAGE_STEP;
            int rpm   = RPM_MIN + row * RPM_STEP;
            int value = lookupTable[row][column];
            insert(table, noLoadVoltage, rpm, value);
        }
    }
}

PowerLimit* PL_new(){
    PowerLimit* me = (PowerLimit*)malloc(sizeof(PowerLimit));
    me->hashtable = HashTable_new();
    populatePLHashTable(me->hashtable); 

    me-> PLstatus = FALSE;   
    me->power = 0.0; 
    me->wheelspeed = 0.0; 

    me->piderror = 0.0; 
    me->plfinaltq = 0.0; 
    me->pidsetpoint = 0.0; 
    me->pidactual = 0.0; 
    me->LUTtq=0.0;
     
    return me;
}
//@shaun do this
float noloadvoltagecalc(){
    return -1; 
}
float getTorque(PowerLimit* me, HashTable* torque_hashtable, float voltage, float rpm){ 
    
    // LUT Lower Bounds
    int volt = (int)(voltage);
    int speed = (int)(rpm);
    // Calculating hashtable keys
    int rpmInput         = speed - RPM_MIN;
    int voltageInput     =  volt - VOLTAGE_MIN;
    int voltageFloor     = int_lowerStepInterval(voltageInput, VOLTAGE_STEP) + VOLTAGE_MIN;
    int voltageCeiling   = int_upperStepInterval(voltageInput, VOLTAGE_STEP) + VOLTAGE_MIN;
    int rpmFloor         = int_lowerStepInterval(rpmInput, RPM_STEP) + RPM_MIN;
    int rpmCeiling       = int_upperStepInterval(rpmInput, RPM_STEP) + RPM_MIN;
    
    // Calculating these now to speed up interpolation later in method
    int voltageLowerDiff = volt - voltageFloor;
    int voltageUpperDiff = voltageCeiling - volt;
    int rpmLowerDiff     = speed - rpmFloor;
    int rpmUpperDiff     = rpmCeiling - speed;

    // Retrieve torque values from the hash table for the four corners
    int vFloorRFloor     = get(torque_hashtable, voltageFloor, rpmFloor);
    int vFloorRCeiling   = get(torque_hashtable, voltageFloor, rpmCeiling);
    int vCeilingRFloor   = get(torque_hashtable, voltageCeiling, rpmFloor);
    int vCeilingRCeiling = get(torque_hashtable, voltageCeiling, rpmCeiling);

    // Calculate interpolation values
    int stepDivider          = VOLTAGE_STEP      * RPM_STEP;
    int torqueFloorFloor     = vFloorRFloor      * voltageUpperDiff * rpmUpperDiff;
    int torqueFloorCeiling   = vFloorRCeiling    * voltageUpperDiff * rpmLowerDiff;
    int torqueCeilingFloor   = vCeilingRFloor    * voltageLowerDiff * rpmUpperDiff;
    int torqueCeilingCeiling = vCeilingRCeiling  * voltageLowerDiff * rpmLowerDiff;
    float tq =(float)(torqueFloorFloor + torqueFloorCeiling + torqueCeilingFloor + torqueCeilingCeiling) / stepDivider;
    me->LUTtq= tq;
    // Final TQ from LUT
    return tq;
}


/*
// this is case1: tqpid + equation
void powerLimitTorqueCalculation(TorqueEncoder* tps, MotorController* mcm, PowerLimit* me, BatteryManagementSystem *bms, WheelSpeeds* ws, PID* pid)
{
       // calc stuff//

       ubyte2 maxtq = MCM_getTorqueMax(mcm);
       float appsTqPercent;
       TorqueEncoder_getOutputPercent(tps, &appsTqPercent);
        float gain = 9.549;
        float decitq = 10.0;

    //parameters we need for calculations//
    float watts = (float)(MCM_getPower(mcm)); // divide by 1000 to get watts --> kilowatts
    float driversRequestedtq = appsTqPercent*maxtq; 
    float wheelspeed = (float)MCM_getMotorRPM(mcm);

    
    if(watts > KWH_LIMIT-10)
     {// kwhlimit should be changed to another paramter we make for plthreshold
        me-> PLstatus = TRUE;
      // still need to make/ update all the struct parameters aka values for can validation 
       float pidsetpoint = (float)((KWH_LIMIT*gain/wheelspeed)*decitq);
       float pidactual = (float)((watts*gain/wheelspeed)*decitq);
       PID_setpointUpdate(pid,pidsetpoint);
        //PID_dtUpdate(pid, 0.01);// 10ms this update function sets the dt to the same exact value every iteration. why not just set when initializing the pid and then forgo this set?
       float piderror =  PID_compute(pid, pidactual);
       float PLfinalTQ = pidactual+ piderror;
       

       me->piderror = piderror;
       me->plfinaltq =PLfinalTQ; 
       me->pidsetpoint = pidsetpoint;
       me->pidactual = pidactual;

    }
    else {
        me-> PLstatus = FALSE;
    }

    float plfinaltq=  me->plfinaltq;
    MCM_update_PowerLimit_TorqueLimit(mcm,  plfinaltq); // we need to change this on mcm.c / pl.c/.h 
    MCM_update_PowerLimit_State(mcm, me->PLstatus); 

    // in mcm.c input the if statement for the tps
}
*/

// TODO: write case 3: tqpid+lut
void powerLimitTorqueCalculation(TorqueEncoder* tps, MotorController* mcm, PowerLimit* me, BatteryManagementSystem *bms, WheelSpeeds* ws, PID* pid)
{
       // calc stuff//
        float mcmVoltage = (float)MCM_getDCVoltage(mcm);
        float mcmCurrent =(float) MCM_getDCCurrent(mcm);
       float appsTqPercent;
       TorqueEncoder_getOutputPercent(tps, &appsTqPercent); 
        float gain = 9.549;
        float decitq = 10.0;
        float maxtq = (float)MCM_getMaxTorqueDNm(mcm);

    float noLoadVoltage = (float)((mcmCurrent * 27 / 1000 ) + mcmVoltage);
    float watts = (float)(MCM_getPower(mcm)); // divide by 1000 to get watts --> kilowatts
     me->power= watts;
    float wheelspeed = (float)MCM_getMotorRPM(mcm);
    me->wheelspeed = wheelspeed;
    float commandedTorque = MCM_getCommandedTorque(mcm) * 10; //dnm
     me->pidactual = commandedTorque;
    
    if(watts > KWH_LIMIT-10)
     {// kwhlimit should be changed to another paramter we make for plthreshold
        me-> PLstatus = TRUE;
      // still need to make/ update all the struct parameters aka values for can validation 
       float pidsetpoint = (float)(getTorque(me, me->hashtable,noLoadVoltage,wheelspeed)); // in dnm
        me->pidsetpoint = pidsetpoint;
       PID_setpointUpdate(pid,pidsetpoint);
       PID_dtUpdate(pid, 0.01);// 10ms this update function sets the dt to the same exact value every iteration. why not just set when initializing the pid and then forgo this set?
       float piderror =  PID_compute(pid, commandedTorque);
        me->piderror = piderror;
       float PLfinalTQ = commandedTorque+ piderror;
       if(PLfinalTQ> maxtq)
       {
          PLfinalTQ = maxtq;
       }
       me->plfinaltq =PLfinalTQ; // in dmn
      
    }
    else {
        me-> PLstatus = FALSE;
    }
    MCM_update_PowerLimit_TorqueLimit(mcm, me->plfinaltq); 
    MCM_update_PowerLimit_State(mcm, me->PLstatus); 

}

