#pragma once

#include <AP_UAVCAN/AP_UAVCAN.h>
#include "AP_Airspeed.h"
#include "AP_Airspeed_Backend.h"

#define AP_AIRSPEED_UAVCAN_TIMEOUT_MICROS         5000000 // sensor becomes unhealthy if no successful readings for 5 seconds

class AP_Airspeed_UAVCAN_MS4525 : public AP_Airspeed_Backend
{
public:

    /// Constructor
    AP_Airspeed_UAVCAN_MS4525(AP_Airspeed &mon, uint8_t _instance);

    bool init() override;

    void handle_as_msg(uint16_t pressure, uint16_t temperature, uint16_t pressure2, uint16_t temperature2) override;

	// return the current differential_pressure in Pascal
	bool get_differential_pressure(float &pressure) override;

	// return the current temperature in degrees C, if available
	bool get_temperature(float &temperature) override;

private:
	float _temp_sum;
    float _press_sum;
    uint32_t _temp_count;
    uint32_t _press_count;
    float _temperature;
    float _pressure;
    uint32_t _last_sample_time_ms;
    uint32_t _measurement_started_ms;
	AP_HAL::Semaphore *_airspeed_sem;
	
    float _get_pressure(int16_t dp_raw) const;
	float _get_temperature(int16_t dT_raw) const;
    void _voltage_correction(float &diff_press_pa, float &temperature);
};
