/*
**
** author: jingwenyi
**
*/
#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"

class AP_RangeFinder_uSharp : public AP_RangeFinder_Backend
{

public:
    // constructor
	AP_RangeFinder_uSharp(RangeFinder &ranger, uint8_t instance, RangeFinder::RangeFinder_State &_state,
                                   AP_SerialManager &serial_manager);

    // static detection function
    static bool detect(RangeFinder &ranger, uint8_t instance, AP_SerialManager &serial_manager);

    // update state
    void update(void);

private:
    // get a reading
    bool get_reading(uint16_t &reading_cm);

    AP_HAL::UARTDriver *uart = nullptr;
    uint32_t last_reading_ms = 0;

    enum uSharp{
        uSharp_CMD_LOW_HEAD     = 0,   //0XFF
        uSharp_CMD_HIGH_HEAD    = 1,    //0XFF
        uSharp_CMD_DATA          = 2,    
        uSharp_CMD_SNR           = 3,    
        uSharp_CMD_CRC           = 4,
    }uSharp_cmd_status;
};

