#pragma once

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4


#include "RGBLed.h"

#define UAVCAN_LED_BRIGHT  0xFF    // full brightness
#define UAVCAN_LED_MEDIUM  0x80    // medium brightness
#define UAVCAN_LED_DIM     0x11    // dim
#define UAVCAN_LED_OFF     0x00    // off

class UAVCANRGBLed : public RGBLed
{
public:
    UAVCANRGBLed();
protected:
    bool hw_init(void) override;
    bool hw_set_rgb(uint8_t r, uint8_t g, uint8_t b) override;

private:
    void _timer(void);
    bool _need_update;
    struct {
        uint8_t r, g, b;
    } rgb;
};


#endif
