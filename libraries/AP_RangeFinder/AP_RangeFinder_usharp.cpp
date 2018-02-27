/*
**
** author: jingwenyi
**
*/


#include <AP_HAL/AP_HAL.h>
#include "AP_RangeFinder_usharp.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>

extern const AP_HAL::HAL& hal;


AP_RangeFinder_uSharp::AP_RangeFinder_uSharp(RangeFinder &_ranger, uint8_t instance,
                                                             RangeFinder::RangeFinder_State &_state,
                                                             AP_SerialManager &serial_manager) :
    AP_RangeFinder_Backend(_ranger, instance, _state),
    uSharp_cmd_status(uSharp_CMD_LOW_HEAD)
{
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Lidar, 0);
    if (uart != nullptr) {
        uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Lidar, 0));
    }
}


bool AP_RangeFinder_uSharp::detect(RangeFinder &_ranger, uint8_t instance, AP_SerialManager &serial_manager)
{
    return serial_manager.find_serial(AP_SerialManager::SerialProtocol_Lidar, 0) != nullptr;
}


bool AP_RangeFinder_uSharp::get_reading(uint16_t &reading_cm)
{
    if (uart == nullptr) {
        return false;
    }
 
    //float sum = 0;
    //uint16_t count = 0;
    uint8_t data_cnt  = 0;
    uint8_t snr_cnt = 0;
    uint16_t crc_sum = 0;
    uint8_t data_buffer[10];
    uint8_t snr_buffer[5];
    int16_t nbytes = uart->available();
    uint16_t read_distance = 0;

    static uint16_t filter_distance = 0;

    while (nbytes-- > 0) {
        uint8_t c = uart->read();

        if(uSharp_cmd_status == uSharp_CMD_LOW_HEAD && c == 0xFF){
             uSharp_cmd_status = uSharp_CMD_HIGH_HEAD;
             crc_sum = 0xFF;
         }else if(uSharp_cmd_status == uSharp_CMD_HIGH_HEAD){
            if(c == 0xFF){
                crc_sum += 0xFF;
                data_cnt = 0;
                snr_cnt = 0;
                uSharp_cmd_status = uSharp_CMD_DATA;
                memset(data_buffer, 0, sizeof(data_buffer));
                memset(snr_buffer, 0, sizeof(data_buffer));
            }else{
                uSharp_cmd_status = uSharp_CMD_LOW_HEAD;
            }
         }else if(uSharp_cmd_status == uSharp_CMD_DATA){
            data_buffer[data_cnt] = c;
            crc_sum += c;
            if(++data_cnt == 10){
                uSharp_cmd_status = uSharp_CMD_SNR;
            }
         }else if(uSharp_cmd_status == uSharp_CMD_SNR){
            snr_buffer[snr_cnt] = c;
            crc_sum += c;
            if(++snr_cnt == 5){
                uSharp_cmd_status = uSharp_CMD_CRC;
            }
         }else if(uSharp_cmd_status == uSharp_CMD_CRC){
            if(c == (crc_sum & 0xFF)){
                //Get the minimum distance from 5 distances
                uint16_t min_distance = data_buffer[0] + data_buffer[1] * 256;

                for(int i=1; i<5; i++){
                      uint16_t distance = data_buffer[i*2] + data_buffer[i*2+1] * 256;

                      if(distance > 0 && (distance < min_distance || min_distance == 0)){
                         min_distance = distance;
                      }
                }

                if(min_distance > 0 && (min_distance < read_distance || read_distance == 0)){
                    read_distance = min_distance;
                }
            }

            uSharp_cmd_status = uSharp_CMD_LOW_HEAD;
    
         }

    }

    uSharp_cmd_status = uSharp_CMD_LOW_HEAD;

    // we need to write a byte to prompt another reading
    uart->write('d');

    if(read_distance == 0){
        return false;
    }

    if(filter_distance == 0){
        filter_distance =  read_distance;
    }

    if(read_distance <= filter_distance){
        filter_distance = 0.98f * read_distance + 0.02f * filter_distance;
    }else{
        filter_distance = 0.02f * read_distance + 0.98f * filter_distance;
    }

    reading_cm = filter_distance;
    return true;
}

/*
   update the state of the sensor
*/
void AP_RangeFinder_uSharp::update(void)
{
    if (get_reading(state.distance_cm)) {
        // update range_valid state based on distance measured
        last_reading_ms = AP_HAL::millis();
        update_status();
    } else if (AP_HAL::millis() - last_reading_ms > 200) {
        set_status(RangeFinder::RangeFinder_NoData);
    }
}

