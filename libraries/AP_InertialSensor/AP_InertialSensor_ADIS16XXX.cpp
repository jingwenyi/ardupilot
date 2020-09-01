/*
 * ADIS16375 and similar IMUs driver
 *
 * Copyright 2018 UAVRS.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <assert.h>
#include <utility>
#include <stdio.h>

#include <AP_HAL/AP_HAL.h>

#include "AP_InertialSensor_ADIS16XXX.h"

extern const AP_HAL::HAL& hal;

static const char * const adis16xxx_status_error_msgs[] = {
	"X-axis gyroscope self-test failure",
	"Y-axis gyroscope self-test failure",
	"Z-axis gyroscope self-test failure",
	"X-axis accelerometer self-test failure",
	"Y-axis accelerometer self-test failure",
	"Z-axis accelerometer self-test failure",
	"Unknow failure",
	"Unknow failure",
	"X-axis magnetometer self-test failure",
	"Y-axis magnetometer self-test failure",
	"Z-axis magnetometer self-test failure",
	"Barometer self-test failure",
};

static const struct adis16xxx_chip_info chip_info[] = {
	[Analog_ADIS16375] = {
		.glob_cmd_reg = ADIS16XXX_REG_GLOB_CMD,
		.fncio_ctrl_reg = ADIS16XXX_REG_FNCTIO_CTRL,
		.diag_stat_reg = ADIS16XXX_REG_DIAG_STS,
		
		.gyro_max_val = 22887,
		.gyro_max_scale = 300,
		._gyro_scale = 0.013108f,
		
		.accel_max_val = 21973,
		.accel_max_scale = 18,
		._accel_scale = 0.8192f,
			
		.status_error_msgs = adis16xxx_status_error_msgs,
		.status_error_mask = BIT(ADIS16XXX_DIAG_STAT_XGYRO_FAIL) |
			BIT(ADIS16XXX_DIAG_STAT_YGYRO_FAIL) |
			BIT(ADIS16XXX_DIAG_STAT_ZGYRO_FAIL) |
			BIT(ADIS16XXX_DIAG_STAT_XACCL_FAIL) |
			BIT(ADIS16XXX_DIAG_STAT_YACCL_FAIL) |
			BIT(ADIS16XXX_DIAG_STAT_ZACCL_FAIL) |
			BIT(ADIS16XXX_DIAG_STAT_XMAGN_FAIL) |
			BIT(ADIS16XXX_DIAG_STAT_YMAGN_FAIL) |
			BIT(ADIS16XXX_DIAG_STAT_ZMAGN_FAIL) |
			BIT(ADIS16XXX_DIAG_STAT_BARO_FAIL),
	},
	[Analog_ADIS16480] = {},
	[Analog_ADIS16485] = {},
	[Analog_ADIS16488] = {},
};

AP_InertialSensor_ADIS16XXX::AP_InertialSensor_ADIS16XXX(AP_InertialSensor &imu,
                                                   AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                                                   enum Rotation rotation)
    : AP_InertialSensor_Backend(imu)
    , _dev(std::move(dev))
    , _current_page(-1)
    , _adis_type(Analog_ADIS16375)
    , _rotation(rotation)
{
}

AP_InertialSensor_ADIS16XXX::~AP_InertialSensor_ADIS16XXX()
{
}


AP_InertialSensor_Backend *
AP_InertialSensor_ADIS16XXX::probe(AP_InertialSensor &imu,
                                AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                                enum Rotation rotation)
{
	printf("%s:: %s.\n", TAG_NAME, __FUNCTION__);

    if (!dev) {
        return nullptr;
    }
	
    AP_InertialSensor_ADIS16XXX *sensor = new AP_InertialSensor_ADIS16XXX(imu, std::move(dev), rotation);

    if (!sensor || !sensor->_init()) {
		delete sensor;
        return nullptr;
    }

    return sensor;
}

void AP_InertialSensor_ADIS16XXX::start()
{
	printf("%s:: %s.\n", TAG_NAME, __FUNCTION__);

    _accel_instance = _imu.register_accel(1000, _dev->get_bus_id_devtype(DEVTYPE_ADIS16375));
    _gyro_instance = _imu.register_gyro(1000,   _dev->get_bus_id_devtype(DEVTYPE_ADIS16375));

    dev_type = DEVTYPE_ADIS16375;

	// setup sensor rotations from probe()
    set_gyro_orientation(_gyro_instance, _rotation);
    set_accel_orientation(_accel_instance, _rotation);

    /* Call _poll_data() at 1kHz */
    _dev->register_periodic_callback(1000, FUNCTOR_BIND_MEMBER(&AP_InertialSensor_ADIS16XXX::_poll_data, void));
}

bool AP_InertialSensor_ADIS16XXX::update()
{
	//printf("%s::%d\n", __FUNCTION__, __LINE__);

    update_accel(_accel_instance);
    update_gyro(_gyro_instance);
	
	return true;
}


int16_t AP_InertialSensor_ADIS16XXX::get_imu_type()
{
    return dev_type;
}


void AP_InertialSensor_ADIS16XXX::_poll_data()
{
	//printf("%s:: %s.\n", TAG_NAME, __FUNCTION__);
	_read_fifo();
}

void AP_InertialSensor_ADIS16XXX::_read_fifo()
{
	uint32_t page = 0;
	uint8_t tx[2] = {0};
	uint8_t rx[2] = {0};

	if(!hal.gpio->imu_data_ready()) {
		//printf("%s:: imu data is not ready.\n", TAG_NAME);
		goto check_registers;
	}

	// 切换到第0页
	// select a page what do you want
	if (_current_page != page) {
		printf("_read_fifo::Change page to 0x%02x.\n", page);
		tx[1] = page;
		tx[0] = ADIS_WRITE_REG(ADIS_REG_PAGE_ID);
		_dev->transfer_fullduplex(tx, rx, 2);
		//printf("%s:: write#transfer1[0x%x]\n", TAG_NAME, tx[0]<<8|tx[1]);
		_current_page = 0;
	}

	_read_data_gyro();
	_read_data_accel();
	//_read_data_temperature();

check_registers:
	// check next register value for correctness
	_dev->set_speed(AP_HAL::Device::SPEED_LOW);
    if (!_dev->check_next_register()) {
	    _inc_gyro_error_count(_gyro_instance);
	    _inc_accel_error_count(_accel_instance);
	}
	_dev->set_speed(AP_HAL::Device::SPEED_HIGH);
}

/*
 *  read from the data registers and update filtered data
 */
void AP_InertialSensor_ADIS16XXX::_read_data_gyro()
{
	uint8_t i, sign;
	uint16_t gyro[3] = {0};
	float gryo_con[3];
	
	#if 1
	// 高速时钟读取数据
	uint8_t reg[2] = {0};
	uint8_t res[2] = {0};
	uint8_t pTxFinal[2] = {0x0, 0x83};
	reg[0] = ADIS16XXX_REG_X_GYRO_OUT;
	_dev->transfer_fullduplex(reg, res, 2);
	gyro[0] = res[0]<<8|res[1];

	reg[0] = ADIS16XXX_REG_Y_GYRO_OUT;
	_dev->transfer_fullduplex(reg, res, 2);
	gyro[0] = res[0]<<8|res[1];

	reg[0] = ADIS16XXX_REG_Z_GYRO_OUT;
	_dev->transfer_fullduplex(reg, res, 2);
	gyro[1] = res[0]<<8|res[1];

	_dev->transfer_fullduplex(pTxFinal, res, 2);
	gyro[2] = res[0]<<8|res[1];
	#else
	// 低速时钟读取数据
	gyro[0] = _register_read_16(ADIS16XXX_REG_X_GYRO_OUT);
	gyro[1] = _register_read_16(ADIS16XXX_REG_Y_GYRO_OUT);
	gyro[2] = _register_read_16(ADIS16XXX_REG_Z_GYRO_OUT);
	//printf("gyro x[0x%04x] y[0x%04x] z[0x%04x].\n", gyro[0], gyro[1], gyro[2]);
	#endif
	
	//陀螺仪数据转换
	for(i = 0; i < 3; i++) {
		sign = gyro[i]&0x8000;
		if(sign)
			gryo_con[i] = (-(~(short )gyro[i]+1)) * _adis_chip_info._gyro_scale * DEG_TO_RAD;
		else
			gryo_con[i] = ((short )gyro[i]) * _adis_chip_info._gyro_scale * DEG_TO_RAD;

		//printf("gryo_con[%d] [%d].\n", i, gryo_con[i]);
	}
	
    Vector3f gyro_data(gryo_con[0], -gryo_con[1], -gryo_con[2]);
	//printf("gyrol_data Vector3f x[%3.3f] y[%3.3f] z[%3.3f].\n", gyro_data.x, gyro_data.y, gyro_data.z);

    _rotate_and_correct_gyro(_gyro_instance, gyro_data);
    _notify_new_gyro_raw_sample(_gyro_instance, gyro_data, AP_HAL::micros64());
}

void AP_InertialSensor_ADIS16XXX::_read_data_accel()
{
	uint8_t i, sign;
	uint16_t accel[3] = {0};
	float accel_con[3];
	
	#if 1
	// 高速时钟读取数据
	uint8_t reg[2] = {0};
	uint8_t res[2] = {0};
	uint8_t pTxFinal[2] = {0x0, 0x83};
	reg[0] = ADIS16XXX_REG_X_ACCEL_OUT;
	_dev->transfer_fullduplex(reg, res, 2);
	accel[0] = res[0]<<8|res[1];

	reg[0] = ADIS16XXX_REG_Y_ACCEL_OUT;
	_dev->transfer_fullduplex(reg, res, 2);
	accel[0] = res[0]<<8|res[1];

	reg[0] = ADIS16XXX_REG_Z_ACCEL_OUT;
	_dev->transfer_fullduplex(reg, res, 2);
	accel[1] = res[0]<<8|res[1];

	_dev->transfer_fullduplex(pTxFinal, res, 2);
	accel[2] = res[0]<<8|res[1];
	#else
	// 低速时钟读取数据
	accel[0] = _register_read_16(ADIS16XXX_REG_X_ACCEL_OUT);
	accel[1] = _register_read_16(ADIS16XXX_REG_Y_ACCEL_OUT);
	accel[2] = _register_read_16(ADIS16XXX_REG_Z_ACCEL_OUT);
	#endif
	//printf("accel x[0x%04x] y[0x%04x] z[0x%04x].\n", accel[0], accel[1], accel[2]);

	//加速度数据转换
	for(i = 0; i < 3; i++) {
		sign = accel[i]&0x8000;
		if(sign) {
			accel_con[i] = (-(~(short )accel[i]+1)) * _adis_chip_info._accel_scale *GRAVITY_MSS/1000.0f;
		} else {
			accel_con[i] = ((short )accel[i]) * _adis_chip_info._accel_scale *GRAVITY_MSS/1000.0f;
		}
		//printf("accel_con[%d] [%f].\n", i, accel_con[i]);
	}

    Vector3f accel_data(accel_con[0], -accel_con[1], -accel_con[2]);
	//printf("accel_data Vector3f x[%3.3f] y[%3.3f] z[%3.3f].\n", accel_data.x, accel_data.y, accel_data.z);

    _rotate_and_correct_accel(_accel_instance, accel_data);
    _notify_new_accel_raw_sample(_accel_instance, accel_data, AP_HAL::micros64());
}

void AP_InertialSensor_ADIS16XXX::_read_data_temperature()
{
	uint8_t sign;
	uint16_t temp = 0;
	float temperature = 0;
	
	temp = _register_read_16(ADIS16XXX_REG_TEMP_OUT);

	sign = temp&0x8000;
	if(sign)
		temperature = (-(~(short )temp+1))*0.00565+25;
	else
		temperature = ((short )temp)*0.00565+25;
	
	printf("temperature[%d] [%2.1f].\n", temp, temperature);
}

bool AP_InertialSensor_ADIS16XXX::_init()
{	
	printf("%s:: %s.\n", TAG_NAME, __FUNCTION__);

	// Init adis data.
	memcpy(&_adis_chip_info, &chip_info[_adis_type], sizeof(struct adis16xxx_chip_info));

	_dev->set_speed(AP_HAL::Device::SPEED_LOW);
		
	if(!_reset()) {
		printf("%s:: Failed to reset.\n", TAG_NAME);
		return false;
	}
	hal.scheduler->delay(70);

	if(!_test_myself()) {
		printf("%s:: Failed to test myselsf.\n", TAG_NAME);
		return false;
	}
	hal.scheduler->delay(30);

	_check_status();

	for(int i = 0; i < 5; i++) {
		if(!_check_whoami()) {
			printf("%s:: Failed to check myself.\n", TAG_NAME);
			if(i==4) {
				return false;
			}
		} else {
			break;
		}
	}

	if(!_hardware_init()) {
		printf("%s:: Failed to init hardware.\n", TAG_NAME);
		return false;
	}

	_dev->set_speed(AP_HAL::Device::SPEED_HIGH);
	
	return true;
}

bool AP_InertialSensor_ADIS16XXX::_reset()
{
	printf("%s:: %s.\n", TAG_NAME, __FUNCTION__);

	return _register_write_8(_adis_chip_info.glob_cmd_reg, ADIS_GLOB_CMD_SW_RESET);
}

bool AP_InertialSensor_ADIS16XXX::_test_myself()
{
	printf("%s:: %s.\n", TAG_NAME, __FUNCTION__);

	return _register_write_16(_adis_chip_info.glob_cmd_reg, ADIS_GLOB_CMD_TEST_MYSELF);
}

uint32_t AP_InertialSensor_ADIS16XXX::_check_status()
{
	printf("%s:: %s.\n", TAG_NAME, __FUNCTION__);

	uint32_t status;
	int i;

	status = _register_read_16(_adis_chip_info.diag_stat_reg);

	status &= _adis_chip_info.status_error_mask;

	if (status == 0)
		return 0;

	for (i = 0; i < 16; ++i) {
		if (status & BIT(i)) {
			printf("%s:: %s.\n",_adis_chip_info.status_error_msgs[i], TAG_NAME);
			printf("%s:: error???????.\n", TAG_NAME);
		}
	}
	
	return status;
}

bool AP_InertialSensor_ADIS16XXX::_hardware_init()
{	
	printf("%s:: %s.\n", TAG_NAME, __FUNCTION__);

#if 0
	if(!_enable_irq(true)) {
		printf("%s:: Failed to enable irq.\n", TAG_NAME);
		return false;
	}

	if(!_configure_data_io()) {
		printf("%s:: Failed to configure data io.\n", TAG_NAME);
		return false;
	}

	if(!_set_sample_rate()) {
		printf("%s:: Failed to set sample rate.\n", TAG_NAME);
		return false;
	}
#endif

	if(!_offset_zero()) {
		printf("%s:: Failed to offset zero.\n", TAG_NAME);
		return false;
	}

    if (!_configure_gyro()) {
        printf("%s:: Failed to configure gyroscope.\n", TAG_NAME);
    }

    if (!_configure_accel()) {
        printf("%s:: Failed to configure accelerometer.\n", TAG_NAME);
    }

	return true;
}

bool AP_InertialSensor_ADIS16XXX::_check_whoami()
{
#if 0
#if 0
	uint8_t val = 0;
	uint8_t reg = 0x75|0x80;
	_dev->transfer(&reg, 1, &val, 1);
	printf("%s::whoami[0x%02x]\n", __FUNCTION__, val);

	uint8_t tx[2] = {0};
	if (_current_page != 0) {
		printf("Change page to 0x%02x.\n", 0);
		tx[1] = 0;
		tx[0] = ADIS_WRITE_REG(ADIS_REG_PAGE_ID);
		_dev->transfer(tx, 2, nullptr, 0);
		printf("%s:: write#transfer1[0x%x]\n", TAG_NAME, tx[0]<<8|tx[1]);
		_current_page = 0;
	}
#else

	uint8_t regs[3] = {0};
	uint8_t rx[3] = {0};
	regs[0] = 0x7E;
	regs[1] = 0x00;
	_dev->transfer_fullduplex(regs, rx, 2);
	//_dev->transfer(&regs[0], 1, &rx[0], 1);
	//_dev->transfer(&regs[1], 1, &rx[1], 1);
	//_dev->transfer(&regs[2], 1, &rx[2], 1);
	printf("%s::whoami[0x%02x][0x%02x][0x%02x]\n", __FUNCTION__, rx[0], rx[1], rx[2]);
	//_dev->transfer(nullptr, 0, rx, 2);
	//printf("%s::whoami[0x%02x][0x%02x]\n", __FUNCTION__, rx[0], rx[1]);
	return true;
#endif
#endif
	printf("%s:: %s.\n", TAG_NAME, __FUNCTION__);

    uint16_t whoami = _register_read_16(ADISREG_WHOAMI);
	printf("%s:: product id[%d]\n", TAG_NAME, whoami);
	
    switch (whoami) {
	    case ADIS_WHOAMI_16375:
	        _adis_type = Analog_ADIS16375;
	        return true;
	    case ADIS_WHOAMI_16480:
	        _adis_type = Analog_ADIS16480;
	        return true;
		default:
			return false;
    }
	
    // not a value WHOAMI result
    return false;
}

bool AP_InertialSensor_ADIS16XXX::_offset_zero()
{
	printf("%s:: %s.\n", TAG_NAME, __FUNCTION__);

	return _register_write_16(_adis_chip_info.glob_cmd_reg, ADIS_GLOB_CMD_OFFSET_ZERO);
}

bool AP_InertialSensor_ADIS16XXX::_set_sample_rate(uint32_t rate)
{
	return _register_write_16(ADIS16XXX_REG_DEC_RATE, (ADIS16XXX_SAMPLE_RATE/rate)-1);
}

bool AP_InertialSensor_ADIS16XXX::_enable_irq(bool enable)
{
	uint32_t funcio_ctrl = _register_read_16(_adis_chip_info.fncio_ctrl_reg);

	printf("fncio_ctrl_reg[0x%04x].\n", funcio_ctrl);

	// high level trigger
	funcio_ctrl |= ADIS_FNCIO_CTRL_DATA_RDY_POL_HIGH;

	// set data ready io with DIO2
	funcio_ctrl &= ~(BIT(1));
	funcio_ctrl |= ADIS_FNCIO_CTRL_DATA_RDY_DIO2;

	// enable data ready trigger
	if (enable)
		funcio_ctrl |= ADIS_FNCIO_CTRL_DATA_RDY_EN;
	else
		funcio_ctrl &= ~ADIS_FNCIO_CTRL_DATA_RDY_EN;

	//printf("fncio_ctrl_reg[0x%04x].\n", funcio_ctrl);
	return _register_write_16(_adis_chip_info.fncio_ctrl_reg, funcio_ctrl);
}

bool AP_InertialSensor_ADIS16XXX::_configure_data_io()
{
	return true;
}

bool AP_InertialSensor_ADIS16XXX::_configure_gyro()
{
	return true;
}

bool AP_InertialSensor_ADIS16XXX::_configure_accel()
{
	return true;
}

bool AP_InertialSensor_ADIS16XXX::_configure_fifo()
{
	return true;
}

static uint16_t __get_unaligned_be16(const uint8_t *p)
{
	return p[0] << 8 | p[1];
}

static uint32_t __get_unaligned_be32(const uint8_t *p)
{
	return p[0] << 24 | p[1] << 16 | p[2] << 8 | p[3];
}

static uint16_t get_unaligned_be16(const void *p)
{
	return __get_unaligned_be16((const uint8_t *)p);
}

static  uint32_t get_unaligned_be32(const void *p)
{
	return __get_unaligned_be32((const uint8_t *)p);
}

bool AP_InertialSensor_ADIS16XXX::_register_read(uint32_t reg, uint32_t *val, uint32_t size)
{
	bool ret = false;
	
	uint32_t page = reg / ADIS16XXX_PAGE_SIZE;

	uint8_t tx[6] = {0};
	uint8_t rx[4] = {0};

	// select a page what do you want
	if (_current_page != page) {
		printf("Change page to 0x%02x.\n", page);
		tx[1] = page;
		tx[0] = ADIS_WRITE_REG(ADIS_REG_PAGE_ID);
		_dev->transfer(tx, 2, nullptr, 0);
		//printf("%s:: write#transfer1[0x%x]\n", TAG_NAME, tx[0]<<8|tx[1]);
	}

	switch(size) {
		case 4:
			tx[5] = 0;
			tx[4] = ADIS_READ_REG(reg + 2);
		case 2:
			tx[3] = 0;
			tx[2] = ADIS_READ_REG(reg);
			break;
		default:
			break;
	}

	switch(size) {
		case 4:
			ret = _dev->transfer(tx+2, 4, rx, 4);
			break;
		case 2:
			ret = _dev->transfer(tx+2, 2, rx, 2);
			//printf("%s:: [%s]#transfer1[0x%x]-[0x%02x][0x%02x]\n", TAG_NAME, __FUNCTION__, tx[2]<<8|tx[3], rx[0], rx[1]);
			break;
		default:
			break;
	}

	_current_page = page;

	switch (size) {
		case 4:
			*val = get_unaligned_be32(rx);
			break;
		case 2:
			*val = get_unaligned_be16(rx);
			break;
	}
	
	return ret;
}

uint32_t AP_InertialSensor_ADIS16XXX::_register_read_16(uint32_t reg)
{
	uint32_t val = 0;
	
	_register_read(reg, &val, 2);

	return val;
}

uint32_t AP_InertialSensor_ADIS16XXX::_register_read_32(uint32_t reg)
{
	uint32_t val = 0;
	
	_register_read(reg, &val, 4);

	return val;
}

bool AP_InertialSensor_ADIS16XXX::_register_write(uint32_t reg, uint32_t val, uint32_t size)
{
	bool ret = false;
	
	uint32_t page = reg / ADIS16XXX_PAGE_SIZE;

	uint8_t tx[10] = {0};

	// select a page what do you want
	if (_current_page != page) {
		printf("Change page to 0x%02x.\n", page);
		tx[1] = page;
		tx[0] = ADIS_WRITE_REG(ADIS_REG_PAGE_ID);
		_dev->transfer(tx, 2, nullptr, 0);
		//printf("%s:: write#transfer1[0x%x]\n", TAG_NAME, tx[0]<<8|tx[1]);
	}

	switch(size) {
		case 4:
			tx[9] = (val>>24) & 0xff;
			tx[8] = ADIS_WRITE_REG(reg + 3);
			tx[7] = (val>>16) & 0xff;
			tx[6] = ADIS_WRITE_REG(reg + 2);
		case 2:
			tx[5] = (val>>8) & 0xff;
			tx[4] = ADIS_WRITE_REG(reg + 1);
		case 1:
			tx[3] = val & 0xff;
			tx[2] = ADIS_WRITE_REG(reg);
			break;
		default:
			break;
	}

	switch(size) {
		case 4:
			ret = _dev->transfer(tx+2, 8, nullptr, 0);
			break;
		case 2:
			ret = _dev->transfer(tx+2, 4, nullptr, 0);
			break;
		case 1:
			ret = _dev->transfer(tx+2, 2, nullptr, 0);
			//printf("%s:: [%s]#transfer2[0x%02x][0x%02x]\n", TAG_NAME, __FUNCTION__, tx[2], tx[3]);
			break;
		default:
			break;
	}

	_current_page = page;

	return ret;
}

bool AP_InertialSensor_ADIS16XXX::_register_write_8(uint32_t reg, uint32_t val)
{
	return _register_write(reg, val, 1);
}

bool AP_InertialSensor_ADIS16XXX::_register_write_16(uint32_t reg, uint32_t val)
{
	return _register_write(reg, val, 2);
}

bool AP_InertialSensor_ADIS16XXX::_register_write_32(uint32_t reg, uint32_t val)
{
	return _register_write(reg, val, 4);
}

