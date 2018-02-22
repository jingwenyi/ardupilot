/*
** author:jingwenyi
** date:2017/10
**
*/


#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"

class AP_RangeFinder_nra24 : public AP_RangeFinder_Backend
{
public:
    // constructor
    AP_RangeFinder_nra24(RangeFinder &ranger, uint8_t instance, RangeFinder::RangeFinder_State &_state,
                                           AP_SerialManager &serial_manager);
    // static detection function
    static bool detect(RangeFinder &ranger, uint8_t instance, AP_SerialManager &serial_manager);

    // update state
    void update(void);

private:
    // get a reading
    bool get_reading(uint16_t &reading_cm);

    enum Nra24{
        NRA24_CMD_HIGH_HEAD = 0,   //0XAA
        NRA24_CMD_LOW_HEAD  = 1,    //0XAA
        NRA24_CMD_HIGH_ID   = 2,    //0X0C
        NRA24_CMD_LOW_ID    = 3,    //0X07
        NRA24_CMD_DATA_INDEX= 4,
        NRA24_CMD_DATA_RCS  = 5,
        NRA24_CMD_DATA_RANGEH = 6,
        NRA24_CMD_DATA_RANGEL = 7,
        NRA24_CMD_DATA_RSVD   = 8,
        NRA24_CMD_DATA_VRR    = 9,
        NRA24_CMD_DATA_VRE1H  = 10,
        NRA24_CMD_DATA_CRC    = 11,
        NRA24_CMD_HIGH_END    = 12,  //0X55
        NRA24_CMD_LOW_END     = 13   //0X55
    }nra24_cmd_status;

    AP_HAL::UARTDriver *uart = nullptr;
    uint32_t last_reading_ms = 0;
};

