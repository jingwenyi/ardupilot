/*
** author: jingwenyi
**
** date: 2017/10
*/

#include <AP_HAL/AP_HAL.h>
#include "AP_RangeFinder_nra24.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>

extern const AP_HAL::HAL& hal;


AP_RangeFinder_nra24::AP_RangeFinder_nra24(RangeFinder &_ranger, uint8_t instance,
                                                               RangeFinder::RangeFinder_State &_state,
                                                               AP_SerialManager &serial_manager) :
    AP_RangeFinder_Backend(_ranger, instance, _state),
    nra24_cmd_status(NRA24_CMD_HIGH_HEAD)
{
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Lidar, 0);
    if (uart != nullptr) {
        uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Lidar, 0));
    }
}


bool AP_RangeFinder_nra24::detect(RangeFinder &_ranger, uint8_t instance, AP_SerialManager &serial_manager)
{
    return serial_manager.find_serial(AP_SerialManager::SerialProtocol_Lidar, 0) != nullptr;
}

bool AP_RangeFinder_nra24::get_reading(uint16_t &reading_cm)
{
    if (uart == nullptr) {
        return false;
    }

    float sum = 0;
    uint16_t count = 0;
    uint16_t crc_sum = 0;
    uint8_t data_buffer[2];
    int16_t nbytes = uart->available();

    while (nbytes-- > 0) {
          uint8_t c = uart->read();

          if(nra24_cmd_status == NRA24_CMD_HIGH_HEAD && c == 0xAA){
             nra24_cmd_status = NRA24_CMD_LOW_HEAD;
          }else if(nra24_cmd_status == NRA24_CMD_LOW_HEAD){
            if(c == 0xAA){
                crc_sum = 0;
                nra24_cmd_status = NRA24_CMD_HIGH_ID;
                memset(data_buffer, 0, sizeof(data_buffer));
            }else{
                nra24_cmd_status = NRA24_CMD_HIGH_HEAD;
            }
          }else if(nra24_cmd_status == NRA24_CMD_HIGH_ID){
            if(c == 0x0C){
                nra24_cmd_status = NRA24_CMD_LOW_ID;
            }else{
                nra24_cmd_status = NRA24_CMD_HIGH_HEAD;
            }
          }else if(nra24_cmd_status == NRA24_CMD_LOW_ID){
            if(c == 0x07){
                nra24_cmd_status = NRA24_CMD_DATA_INDEX;
            }else{
                nra24_cmd_status = NRA24_CMD_HIGH_HEAD;
            }
          }else if(nra24_cmd_status == NRA24_CMD_DATA_INDEX){
            crc_sum += c;
            nra24_cmd_status = NRA24_CMD_DATA_RCS;
          }else if(nra24_cmd_status == NRA24_CMD_DATA_RCS){
            crc_sum += c;
            nra24_cmd_status = NRA24_CMD_DATA_RANGEH;
          }else if(nra24_cmd_status == NRA24_CMD_DATA_RANGEH){
            crc_sum += c;
            data_buffer[0] = c;
            nra24_cmd_status = NRA24_CMD_DATA_RANGEL;
          }else if(nra24_cmd_status == NRA24_CMD_DATA_RANGEL){
            crc_sum += c;
            data_buffer[1] = c;
            nra24_cmd_status = NRA24_CMD_DATA_RSVD;
          }else if(nra24_cmd_status == NRA24_CMD_DATA_RSVD){
            crc_sum += c;
            nra24_cmd_status = NRA24_CMD_DATA_VRR;
          }else if(nra24_cmd_status == NRA24_CMD_DATA_VRR){
            crc_sum += c;
            nra24_cmd_status = NRA24_CMD_DATA_VRE1H;
          }else if(nra24_cmd_status == NRA24_CMD_DATA_VRE1H){
            crc_sum += c;
            nra24_cmd_status = NRA24_CMD_DATA_CRC;
          }else if(nra24_cmd_status == NRA24_CMD_DATA_CRC){
            if(c == (crc_sum  & 0xFF)){
                nra24_cmd_status = NRA24_CMD_HIGH_END;
            }else{
                nra24_cmd_status = NRA24_CMD_HIGH_HEAD;
            }
            
          }else if(nra24_cmd_status == NRA24_CMD_HIGH_END){
            if(c == 0x55){
                nra24_cmd_status = NRA24_CMD_LOW_END;
            }else{
                nra24_cmd_status = NRA24_CMD_HIGH_HEAD;
            }
          }else if(nra24_cmd_status == NRA24_CMD_LOW_END){
            
            if(c == 0x55){
                sum += data_buffer[0] * 256 + data_buffer[1];
                count++;
            }
            nra24_cmd_status = NRA24_CMD_HIGH_HEAD;
          }
         
    }

    nra24_cmd_status = NRA24_CMD_HIGH_HEAD;
    
    // we need to write a byte to prompt another reading
    uart->write('d');
    
    if (count == 0) {
        return false;
    }
    reading_cm = sum / count;
    return true;
}


void AP_RangeFinder_nra24::update(void)
{
    if (get_reading(state.distance_cm)) {
        // update range_valid state based on distance measured
        last_reading_ms = AP_HAL::millis();
        update_status();
    } else if (AP_HAL::millis() - last_reading_ms > 200) {
        set_status(RangeFinder::RangeFinder_NoData);
    }
}



