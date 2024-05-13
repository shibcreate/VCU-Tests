
#include <stdio.h>
#include "bms.h"
#include <stdlib.h>
#include "IO_Driver.h"
#include "IO_RTC.h"
#include "IO_DIO.h"
#include "serial.h"
#include "mathFunctions.h"

/*********************************************************
 *            *********** CAUTION ***********            *
 * MULTI-BYTE VALUES FOR THE STAFL BMS ARE LITTLE-ENDIAN *
 *                                                       *
 *********************************************************/

void BMS_new(BatteryManagementSystem *me, ubyte2 canMessageBaseID)
{
    me->canMessageBaseId = canMessageBaseID;

    //Repick a new value, maybe 0xFFFF?
    me->highestCellVoltage = 0;
    me->lowestCellVoltage = 9999;
    me->highestCellTemperature = 0;


    me->relayState = FALSE;
}

void BMS_parseCanMessage(BatteryManagementSystem *bms, IO_CAN_DATA_FRAME *bmsCanMessage)
{
    //Subtract BMS Base CAN ID from incoming BMS CAN message ID to get offset
    //Byte extraction DOES NOT INCLUDE SCALING
    //Ex: (bmsCanMessage->id+BMS_MASTER_FAULTS) - bms->canMessageBaseId = BMS_MASTER_FAULTS
    switch (bmsCanMessage->id - bms->canMessageBaseId)
    {
        case BMS_SAFETY_CHECKER:
            bms->faultFlags0 = (ubyte1)bmsCanMessage->data[0];
            break;
        
        case BMS_CELL_SUMMARY:
            bms->highestCellVoltage           = ( ((ubyte1)bmsCanMessage->data[1]) << 8)
                                                | ((ubyte1)bmsCanMessage->data[0]);

            bms->lowestCellVoltage            = ( ((ubyte1)bmsCanMessage->data[3]) << 8)
                                                | ((ubyte1)bmsCanMessage->data[2]);

            bms->highestCellTemperature       = ( ((ubyte1)bmsCanMessage->data[5]) << 8)
                                                | ((ubyte1)bmsCanMessage->data[4]);

            bms->lowestCellTemperature        = ( ((ubyte1)bmsCanMessage->data[7]) << 8)
                                                | ((ubyte1)bmsCanMessage->data[6]);
            break;
            
    }
}

IO_ErrorType BMS_relayControl(BatteryManagementSystem *me)
{
    IO_ErrorType err;
    //There is a fault
    if (me->faultFlags0)
    {
        me->relayState = TRUE;
        err = IO_DO_Set(IO_DO_01, TRUE); //Drive BMS relay true (HIGH)
    }
    //There is no fault
    else
    {
        me->relayState = FALSE;
        err = IO_DO_Set(IO_DO_01, FALSE); //Drive BMS relay false (LOW)
    }
    return err;
}

ubyte2 BMS_getHighestCellVoltage_mV(BatteryManagementSystem *me)
{
    return (me->highestCellVoltage);
}

ubyte2 BMS_getLowestCellVoltage_mV(BatteryManagementSystem *me)
{
    return (me->lowestCellVoltage);
}

//Split into
sbyte2 BMS_getHighestCellTemp_d_degC(BatteryManagementSystem *me)
{
    return (me->highestCellTemperature);
}

sbyte2 BMS_getHighestCellTemp_degC(BatteryManagementSystem *me)
{
    return (me->highestCellTemperature/BMS_TEMPERATURE_SCALE);
}
