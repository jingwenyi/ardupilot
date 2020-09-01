/*
  UAVCAN RGBLed driver
*/

/* LED driver for UAVCANRGBLed */

#include <AP_HAL/AP_HAL.h>


#if CONFIG_HAL_BOARD == HAL_BOARD_PX4


#include "UAVCanRGBLed.h"

#include <utility>


#if HAL_WITH_UAVCAN
#include <AP_HAL_PX4/CAN.h>
#include <AP_UAVCAN/AP_UAVCAN.h>
#endif

extern const AP_HAL::HAL& hal;

UAVCANRGBLed::UAVCANRGBLed():
    RGBLed(UAVCAN_LED_OFF, UAVCAN_LED_BRIGHT, UAVCAN_LED_MEDIUM, UAVCAN_LED_DIM)
{
    
}


bool UAVCANRGBLed::hw_init()
{
    bool ret = true;

    //printf("UAVCANRGBLed::hw_init\n");
    
    return ret;
}

// set_rgb - set color as a combination of red, green and blue values
bool UAVCANRGBLed::hw_set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    rgb = {red, green, blue};
    _need_update = true;

    if(hal.can_mgr != nullptr) {
        if((hal.can_mgr)->is_initialized()) {
            if ((hal.can_mgr)->get_UAVCAN() != nullptr) {
                ((hal.can_mgr)->get_UAVCAN())->do_cyclic_rgbled(rgb.r, rgb.g, rgb.b);
            }
        }
    }
    
    return true;
}
void UAVCANRGBLed::_timer(void)
{
    if (!_need_update) {
        return;
    }

    _need_update = false;
}


#endif
