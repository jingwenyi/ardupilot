#include <AP_HAL/AP_HAL.h>

#if HAL_WITH_UAVCAN

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include "AP_Airspeed.h"
#include "AP_Airspeed_UAVCAN_MS4525.h"

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>

extern const AP_HAL::HAL& hal;

#define debug_as_uavcan(level, fmt, args...) do { if ((level) <= AP_BoardConfig_CAN::get_can_debug()) { printf(fmt, ##args); }} while (0)

/// Constructor
AP_Airspeed_UAVCAN_MS4525::AP_Airspeed_UAVCAN_MS4525(AP_Airspeed &mon, uint8_t _instance) :
    AP_Airspeed_Backend(mon, _instance)
{
	_airspeed_sem = hal.util->new_semaphore();
}

bool AP_Airspeed_UAVCAN_MS4525::init()
{
    if (AP_BoardConfig_CAN::get_can_num_ifaces() != 0) {
        for (uint8_t i = 0; i < MAX_NUMBER_OF_CAN_DRIVERS; i++) {
            if (hal.can_mgr[i] != nullptr) {
                AP_UAVCAN *uavcan = hal.can_mgr[i]->get_UAVCAN();
                if (uavcan != nullptr) {
                    if(uavcan->register_airspeed_listener_to_id(this, get_instance())) {
                        printf("AP_Airspeed_UAVCAN_MS4525 CAN_Driver[%d] registered id: %d\n", i, get_instance());
						return true;
                    } else {
						printf("Failed to register_airspeed_listener_to_id\n");
					}
                } else {
					printf("uavcan is null\n", i);
				}
            } else {
				printf("hal.can_mgr[%d] is null\n", i);
			}
        }
    }
	printf("AP_Airspeed_UAVCAN_MS4525 init error!\n");
	return false;
}

// return the current differential_pressure in Pascal
bool AP_Airspeed_UAVCAN_MS4525::get_differential_pressure(float &pressure)
{
	int32_t current = AP_HAL::millis();
	int32_t timeout = (current - _last_sample_time_ms);
	
	//printf("get_differential_pressure timeout[%d], current[%d], [%d]\n", timeout, current, _last_sample_time_ms);
    if (timeout > 100) {
		//gcs().send_text(MAV_SEVERITY_INFO, "Airspeed get_differential_pressure timeout");
		//printf("get_differential_pressure timeout[%d], current[%d], [%d]\n", timeout, current, _last_sample_time_ms);
        return false;
    }

    if (sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        if (_press_count > 0) {
            _pressure = _press_sum / _press_count;
            _press_count = 0;
            _press_sum = 0;
        }
        sem->give();
    }
    pressure = _pressure;

    return true;
}

// return the current temperature in degrees C, if available
bool AP_Airspeed_UAVCAN_MS4525::get_temperature(float &temperature)
{
    if ((AP_HAL::millis() - _last_sample_time_ms) > 100) {
		//printf("get_temperature timeout\n");
        return false;
    }

    if (sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        if (_temp_count > 0) {
            _temperature = _temp_sum / _temp_count;
            _temp_count = 0;
            _temp_sum = 0;
        }
        sem->give();
    }
    temperature = _temperature;

    return true;
}

/**
   correct for 5V rail voltage if the system_power ORB topic is
   available

   See http://uav.tridgell.net/MS4525/MS4525-offset.png for a graph of
   offset versus voltage for 3 sensors
 */
void AP_Airspeed_UAVCAN_MS4525::_voltage_correction(float &diff_press_pa, float &temperature)
{
	const float slope = 65.0f;
	const float temp_slope = 0.887f;

	/*
	  apply a piecewise linear correction within range given by above graph
	 */
	float voltage_diff = hal.analogin->board_voltage() - 5.0f;

    voltage_diff = constrain_float(voltage_diff, -0.7f, 0.5f);

	diff_press_pa -= voltage_diff * slope;
	temperature -= voltage_diff * temp_slope;
}

/*
  this equation is an inversion of the equation in the
  pressure transfer function figure on page 4 of the datasheet
  
  We negate the result so that positive differential pressures
  are generated when the bottom port is used as the static
  port on the pitot and top port is used as the dynamic port
*/
float AP_Airspeed_UAVCAN_MS4525::_get_pressure(int16_t dp_raw) const
{
    const float P_max = get_psi_range();
    const float P_min = - P_max;
    const float PSI_to_Pa = 6894.757f;

    float diff_press_PSI  = -((dp_raw - 0.1f*16383) * (P_max-P_min)/(0.8f*16383) + P_min);
    float press  = diff_press_PSI * PSI_to_Pa;
    return press;
}

/*
  convert raw temperature to temperature in degrees C
 */
float AP_Airspeed_UAVCAN_MS4525::_get_temperature(int16_t dT_raw) const
{
    float temp  = ((200.0f * dT_raw) / 2047) - 50;
    return temp;
}


void AP_Airspeed_UAVCAN_MS4525::handle_as_msg(uint16_t pressure, uint16_t temperature, uint16_t pressure2, uint16_t temperature2)
{
	//printf("handle as msg pressure[%d], temperature[%d], pressure2[%d], temperature2[%d]\n", pressure, temperature, pressure2, temperature2);
	uint8_t status = ((pressure>>8) & 0xC0) >> 6;
    if (status == 2 || status == 3) {
		//printf("status is error\n");
        return;
    }

	int16_t dp_raw, dT_raw;
	int16_t dp_raw2, dT_raw2;
	
	dp_raw = 0x3FFF & pressure;
	dT_raw = (0xFFE0 & temperature) >> 5;

	dp_raw2 = 0x3FFF & pressure2;
	dT_raw2 = (0xFFE0 & temperature2) >> 5;

	// reject any values that are the absolute minimum or maximums these
	// can happen due to gnd lifts or communication errors on the bus
	if (dp_raw	== 0x3FFF || dp_raw  == 0 || dT_raw  == 0x7FF || dT_raw == 0 ||
    	dp_raw2 == 0x3FFF || dp_raw2 == 0 || dT_raw2 == 0x7FF || dT_raw2 == 0) {
		return;
	}

	// reject any double reads where the value has shifted in the upper more than
	// 0xFF
	if (abs(dp_raw - dp_raw2) > 0xFF || abs(dT_raw - dT_raw2) > 0xFF) {
		return;
	}
	
	float press  = _get_pressure(dp_raw);
	float temp	= _get_temperature(dT_raw);
	float press2  = _get_pressure(dp_raw2);
	float temp2 = _get_temperature(dT_raw2);

	/* close voltage correction */
#if 0
	_voltage_correction(press, temp);
	_voltage_correction(press2, temp2);
#endif
	//printf("_voltage_correction press[%.2f], temp[%.2f]\n", press, temp);

    if (sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        _press_sum += press + press2;
        _temp_sum += temp + temp2;
        _press_count += 2;
        _temp_count += 2;
        sem->give();
    }

    _last_sample_time_ms = AP_HAL::millis();
	//printf("update _last_sample_time_ms %d\n", _last_sample_time_ms);
}

#endif
