#pragma once
/*
  driver for the analog range of IMUs, including:

  ADIS16375
  ADIS16480
  ADIS16485
  ADIS16488
  
 */

#include <stdint.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/SPIDevice.h>
#include <AP_HAL/utility/OwnPtr.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"
#include "AP_InertialSensor_ADIS16XXX_Registers.h"

#define TAG_NAME "AP_InertialSensor_ADIS16XXX"

#define ADISREG_WHOAMI	ADIS16XXX_REG_PROD_ID

#define ADIS_WHOAMI_16375	0x3FF7	//16375
#define ADIS_WHOAMI_16480	0x4060	//16480

#define ADIS16XXX_SAMPLE_RATE     2640

/** 1 << the bit number */
#define BIT(shift)                     (1UL << (shift))

enum Analog_Type {
	Analog_ADIS16375 = 0,
	Analog_ADIS16480,
	Analog_ADIS16485,
	Analog_ADIS16488,
};

#define ADIS16XXX_DIAG_STAT_XGYRO_FAIL 0
#define ADIS16XXX_DIAG_STAT_YGYRO_FAIL 1
#define ADIS16XXX_DIAG_STAT_ZGYRO_FAIL 2
#define ADIS16XXX_DIAG_STAT_XACCL_FAIL 3
#define ADIS16XXX_DIAG_STAT_YACCL_FAIL 4
#define ADIS16XXX_DIAG_STAT_ZACCL_FAIL 5
#define ADIS16XXX_DIAG_STAT_XMAGN_FAIL 8
#define ADIS16XXX_DIAG_STAT_YMAGN_FAIL 9
#define ADIS16XXX_DIAG_STAT_ZMAGN_FAIL 10
#define ADIS16XXX_DIAG_STAT_BARO_FAIL 11

struct adis16xxx_chip_info {
	uint32_t glob_cmd_reg;
	uint32_t fncio_ctrl_reg;
	uint32_t diag_stat_reg;

	uint32_t gyro_max_val;
	uint32_t gyro_max_scale;
    float _gyro_scale;
	
	uint32_t accel_max_val;
	uint32_t accel_max_scale;
	float _accel_scale;
	
	const char * const *status_error_msgs;
	uint32_t status_error_mask;
};

class AP_InertialSensor_ADIS16XXX : public AP_InertialSensor_Backend
{
public:
	~AP_InertialSensor_ADIS16XXX();
    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                                            enum Rotation rotation);

    /**
     * Configure the sensors and start reading routine.
     */
    void start() override;

    bool update() override;
    int16_t get_imu_type() override;

private:
    AP_InertialSensor_ADIS16XXX(AP_InertialSensor &imu,
                             AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                             enum Rotation rotation);

    /**
     * If the macro ADIS16XXX_DEBUG is defined, check if there are errors reported
     * on the device's error register and panic if so. The implementation is
     * empty if the ADIS16XXX_DEBUG isn't defined.
     */
    uint32_t _check_status();

    /**
     * Try to perform initialization of the ADIS16XXX device.
     *
     * The device semaphore must be taken and released by the caller.
     *
     * @return true on success, false otherwise.
     */
    bool _hardware_init();

    /**
     * Try to initialize this driver.
     *
     * Do sensor and other required initializations.
     *
     * @return true on success, false otherwise.
     */
    bool _init();

    /**
     * Configure accelerometer sensor. The device semaphore must already be
     * taken before calling this function.
     *
     * @return true on success, false otherwise.
     */
    bool _configure_accel();

    /**
     * Configure gyroscope sensor. The device semaphore must already be
     * taken before calling this function.
     *
     * @return true on success, false otherwise.
     */
    bool _configure_gyro();

    /**
     * Configure data io pin as watermark interrupt pin at the level of one sample
     * if using fifo or data-ready pin otherwise.
     *
     * @return true on success, false otherwise.
     */
    bool _configure_data_io();

    /**
     * Configure FIFO.
     *
     * @return true on success, false otherwise.
     */
    bool _configure_fifo();

    /**
     * Device periodic callback to read data from the sensors.
     */
    void _poll_data();

    /**
     * Read samples from fifo.
     */
    void _read_fifo();

    /**
     * 
     */
	void _read_data_gyro();

    /**
     * 
     */
	void _read_data_accel();

	void _read_data_temperature();

    /**
     * 
     */
	bool _check_whoami();

	/**
     * 
     */
	bool _reset();

	/**
     * 
     */
	bool _test_myself();

	/**
     * 
     */
	bool _offset_zero();

	/**
     * 
     */
	bool _set_sample_rate(uint32_t rate);

	/**
     * 
     */
	bool _enable_irq(bool enable);


    /**
     * 
     */
	bool _register_read(uint32_t reg, uint32_t *val, uint32_t size);
	uint32_t _register_read_16(uint32_t reg);
	uint32_t _register_read_32(uint32_t reg);

	/**
     * 
     */
	bool _register_write(uint32_t reg, uint32_t val, uint32_t size);
	bool _register_write_8(uint32_t reg, uint32_t val);
	bool _register_write_16(uint32_t reg, uint32_t val);
	bool _register_write_32(uint32_t reg, uint32_t val);
	

    AP_HAL::OwnPtr<AP_HAL::SPIDevice> _dev;

	enum Analog_Type _adis_type;

	int32_t _current_page;

	struct adis16xxx_chip_info _adis_chip_info;

    uint8_t _gyro_instance;
    uint8_t _accel_instance;

	enum Rotation _rotation;

    uint16_t dev_type;
};
