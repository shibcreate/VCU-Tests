
#include "IO_Driver.h" //Includes datatypes, constants, etc - should be included in every c file
#include "motorController.h"
#include "PID.h"
#include "hashTable.h"
#include "powerLimit.h"
#include "bms.h"
#include "wheelSpeeds.h"
#include "torqueEncoder.h"
#include "math.h"

void populatePLHashTable(HashTable* table)
{
    /*
voltage is x axis
rpm is y axis 
values are in tq nM

const float4 Voltage_MIN = 283.200;
const float4 Volage_MAX = 403.200;
const sbyte4 RPM_MIN = 100;
const sbyte4 RPM _MAX = 6000;
const ubyte1 NUM_V = 25;
const ubyte1 NUM_S = 25;
*/
    ubyte4 lookupTable[25][25] = {
    {230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85},
	{230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85},
	{230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85},
	{230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85},
	{230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85},
	{230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85},
	{230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85},
	{230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85},
	{229.13, 229.46, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85, 230.85},
	{205.17, 205.54, 205.98, 208.54, 219.56, 219.56, 219.56, 219.56, 219.56, 219.56, 219.56, 219.56, 219.56, 219.56, 219.56, 219.56, 219.56, 219.56, 219.56, 219.56, 219.56, 219.56, 219.56, 219.56, 219.56},
	{185.49, 185.85, 186.12, 186.31, 186.61, 199.56, 199.56, 199.56, 199.56, 199.56, 199.56, 199.56, 199.56, 199.56, 199.56, 199.56, 199.56, 199.56, 199.56, 199.56, 199.56, 199.56, 199.56, 199.56, 199.56},
	{205.67, 169.23, 169.44, 169.65, 169.97, 170.21, 174.93, 182.93, 182.93, 182.93, 182.93, 182.93, 182.93, 182.93, 182.93, 182.93, 182.93, 182.93, 182.93, 182.93, 182.93, 182.93, 182.93, 182.93, 182.93},
	{194.62, 155.20, 155.34, 155.52, 155.64, 155.92, 156.14, 156.39, 166.35, 168.74, 168.74, 168.74, 168.74, 168.74, 168.74, 168.74, 168.74, 168.74, 168.74, 168.74, 168.74, 168.74, 168.74, 168.74, 168.74},
	{184.74, 143.12, 143.18, 143.30, 143.45, 143.63, 144.47, 144.03, 144.26, 144.51, 156.64, 156.64, 156.64, 156.64, 156.64, 156.64, 156.64, 156.64, 156.64, 156.64, 156.64, 156.64, 156.64, 156.64, 156.64},
	{175.71, 187.46, 132.69, 132.78, 132.95, 132.97, 133.12, 133.29, 133.55, 133.72, 133.87, 134.19, 146.10, 146.10, 146.10, 146.10, 146.10, 146.10, 146.10, 146.10, 146.10, 146.10, 146.10, 146.10, 146.10},
	{167.22, 178.99, 123.46, 123.46, 123.50, 123.58, 123.62, 123.85, 123.94, 124.24, 124.51, 124.62, 124.73, 125.10, 136.66, 137.73, 137.73, 137.73, 137.73, 137.73, 137.73, 137.73, 137.73, 137.73, 137.73},
	{159.06, 171.05, 115.32, 115.24, 115.24, 115.40, 115.29, 115.48, 115.53, 115.68, 115.86, 116.00, 116.45, 116.55, 116.71, 117.05, 126.26, 128.56, 128.56, 128.56, 128.56, 128.56, 128.56, 128.56, 128.56},
	{150.90, 163.48, 171.91, 107.97, 107.91, 107.91, 107.87, 107.95, 108.04, 108.23, 108.30, 108.66, 108.64, 108.93, 109.13, 109.27, 109.60, 109.84, 115.86, 121.44, 121.44, 121.44, 121.44, 121.44, 121.44},
	{142.25, 156.06, 164.78, 171.67, 101.42, 101.26, 101.26, 101.42, 101.37, 101.52, 101.57, 101.70, 101.84, 102.02, 102.20, 102.40, 102.61, 102.85, 103.09, 103.34, 105.36, 114.95, 114.95, 114.95, 114.95},
	{131.93, 148.52, 157.81, 164.91, 95.38, 95.25, 95.28, 95.28, 95.25, 95.38, 96.61, 95.56, 95.68, 95.82, 95.98, 96.16, 96.33, 96.47, 96.74, 96.97, 97.23, 97.48, 97.62, 107.50, 109.22},
	{115.50, 140.37, 150.79, 158.29, 164.43, 89.92, 89.86, 89.82, 89.82, 89.86, 89.92, 90.00, 90.08, 90.21, 90.33, 90.47, 90.62, 90.82, 91.00, 91.20, 91.43, 91.65, 91.90, 92.20, 92.39},
	{115.50, 130.39, 143.39, 151.64, 158.11, 163.61, 84.93, 84.86, 84.85, 84.85, 84.86, 84.92, 84.99, 85.06, 85.17, 85.29, 85.44, 85.58, 85.76, 85.82, 86.06, 86.34, 86.43, 86.79, 86.95},
	{115.50, 115.50, 134.95, 144.67, 151.75, 157.54, 80.48, 80.31, 80.33, 80.30, 80.30, 80.23, 79.83, 80.34, 80.50, 80.59, 80.58, 80.83, 80.91, 81.13, 81.29, 81.41, 81.60, 81.81, 82.11},
	{115.50, 115.50, 122.30, 136.94, 145.10, 151.42, 156.73, 76.27, 76.06, 76.08, 75.98, 75.93, 75.98, 76.02, 76.08, 76.15, 76.25, 76.35, 76.56, 76.37, 76.75, 76.92, 77.28, 77.34, 77.54},
	{115.50, 115.50, 115.50, 126.68, 137.79, 145.02, 150.81, 155.76, 72.28, 72.12, 72.07, 72.03, 72.03, 72.13, 72.16, 72.22, 72.28, 72.37, 72.46, 72.57, 72.62, 72.76, 72.99, 73.15, 73.24}
    };
    
      float4 Voltage_MIN = 283.200;
     float4 Voltage_MAX = 403.200;
     sbyte4 RPM_MIN = 100;
     sbyte4 RPM_MAX = 6000;
     ubyte1 NUM_V = 25;
     ubyte1 NUM_S = 25;
    float4 voltageStep = (Voltage_MAX - Voltage_MIN) / (NUM_V - 1);
    sbyte4 rpmStep = (RPM_MAX - RPM_MIN) / (NUM_S - 1);
    for (ubyte1 row = 0; row < NUM_S; row++) {
        for (ubyte1 column = 0; column < NUM_V; column++) {
            float4 voltage = Voltage_MIN + column * voltageStep;
            sbyte4 rpm = RPM_MIN + row * rpmStep;
            ubyte4 value = lookupTable[row][column];
            insert(table, voltage, rpm, value);
        }
    }
}

PowerLimit* PL_new(){
    PowerLimit* me = (PowerLimit*)malloc(sizeof(PowerLimit));
    me->hashtable = HashTable_new();
    populatePLHashTable(me->hashtable); 
    me-> PLstatus = FALSE;
    me->pid = PID_new(1,0,0,0);// fill this in  
     me-> powerLimittq = 0.0; 
     me-> PLstatus = FALSE; 
     me-> error = 0.0; 
    return me;
}
ubyte4 getTorque(PowerLimit* pl, HashTable* torque_hashtable, float4 voltage, sbyte4 rpm) {
    // Define the increment steps
    const float4 voltageIncrement = (403.200 - 283.200) / 24;  // Assuming 25 steps between 283.200 and 403.200
    const sbyte4 rpmIncrement = (6000 - 100) / 24;  // Assuming 25 steps between 100 and 6000
    // Find the floor and ceiling values for voltage and rpm
    float4 voltageFloor = floorToNearestIncrement(voltage, voltageIncrement);
    float4 voltageCeiling = ceilToNearestIncrement(voltage, voltageIncrement);
    sbyte4 rpmFloor = floorToNearestIncrement(rpm, rpmIncrement);
    sbyte4 rpmCeiling = ceilToNearestIncrement(rpm, rpmIncrement);
    // Retrieve torque values from the hash table for the four corners
    ubyte4 floorFloor = get(torque_hashtable, voltageFloor, rpmFloor);
    ubyte4 ceilingFloor = get(torque_hashtable, voltageCeiling, rpmFloor);
    ubyte4 floorCeiling = get(torque_hashtable, voltageFloor, rpmCeiling);
    ubyte4 ceilingCeiling = get(torque_hashtable, voltageCeiling, rpmCeiling);
    // Error check
    if (floorFloor == -1 || ceilingFloor == -1 || floorCeiling == -1 || ceilingCeiling == -1) {
        return -1;
    }
    // Calculate interpolation values
    ubyte4 horizontal_Interp = (((ceilingFloor - floorFloor) / voltageIncrement) + ((ceilingCeiling - floorCeiling) / voltageIncrement)) / 2.0;
    ubyte4 vertical_Interp = (((floorCeiling - floorFloor) / rpmIncrement) + ((ceilingCeiling - ceilingFloor) / rpmIncrement)) / 2.0;
    // Calculate gains
    ubyte4 gainValueHoriz = fmod(voltage, voltageIncrement);
    ubyte4 gainValueVertical = fmod(rpm, rpmIncrement);
    // Combine interpolated values
    ubyte2 calibratedTorque = (gainValueHoriz * horizontal_Interp) + (gainValueVertical * vertical_Interp) + floorFloor;
    return calibratedTorque;  // Adjust gain if necessary
}
void powerLimitTorqueCalculation(TorqueEncoder* tps, MotorController* mcm, PowerLimit* me, BatteryManagementSystem *bms, WheelSpeeds* ws){
  
  sbyte4 wheelspeed = MCM_getMotorRPM(mcm);
  sbyte4 kilowatts =  BMS_getPower_W(bms)/1000; // divide by 1000 to get watts --> kilowatts
  sbyte4 voltage = BMS_getPackVoltage(bms);// CHECK THE UNITS FOR THIS
    if(kilowatts> 78)
    {
        me-> PLstatus = TRUE;
        sbyte2 estimatedtq = (sbyte2) getTorque(me,me->hashtable, voltage  ,wheelspeed);
        sbyte2 tqsetpoint = (sbyte2) get(me->hashtable,78,wheelspeed );
        
    PID_setpointUpdate(me->pid,tqsetpoint);
    PID_dtUpdate(me->pid, 0.01);// 10ms 
    sbyte2 piderror = PID_compute(me->pid, estimatedtq); 
    me->error = piderror; 
    float4 appsTqPercent;
    TorqueEncoder_getOutputPercent(tps, &appsTqPercent);
// the torqueMaximumDNm is 2000, scale it accordingly 
  ubyte2 tq = MCM_getMaxTorqueDNm(mcm);
    me->powerLimittq= (tq * appsTqPercent) + me->error;
    }
    else
    {
        me-> PLstatus = FALSE;
    }
 MCM_update_PowerLimit_TorqueLimit(mcm, me->powerLimittq);
 MCM_update_PowerLimit_State(mcm, me->PLstatus); 
}