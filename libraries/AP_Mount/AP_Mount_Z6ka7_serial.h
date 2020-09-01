/*
  Z6ka7 serial mount using serial protocol backend class
 */
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>

#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include <AP_GPS/AP_GPS.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <RC_Channel/RC_Channel.h>
#include "AP_Mount_Backend.h"

#define AP_MOUNT_Z6KA7_SERIAL_RESEND_MS   1000    // resend angle targets to gimbal once per second


#define SPEED_UNITS         0.122f                      //units:0.122degree/sec
#define ANGLE_UNITS         0.02197f                    //units:0.02197degree


class AP_Mount_Z6ka7_serial : public AP_Mount_Backend
{
public:
    // Constructor
    AP_Mount_Z6ka7_serial(AP_Mount &frontend, AP_Mount::mount_state &state, uint8_t instance);

    // init - performs any required initialisation for this instance
    virtual void init(const AP_SerialManager& serial_manager);

    // update mount position - should be called periodically
    virtual void update();

    // has_pan_control - returns true if this mount can control it's pan (required for multicopters)
    virtual bool has_pan_control() const;

    // set_mode - sets mount's mode
    virtual void set_mode(enum MAV_MOUNT_MODE mode);

    // status_msg - called to allow mounts to send their status to GCS using the MOUNT_STATUS message
    virtual void status_msg(mavlink_channel_t chan);

private:
    enum control_mode{
        NO_CONTROL = 0X00,
        SPEED_MODE = 0X01,
        ANGLE_MODE = 0X02,
        SPEED_ANGLE_MODE = 0X03,
        RC_MODE = 0X04,
        MODE_ANGLE_REL_FRAME = 0X05
    };
    

    struct PACKED Combine_long_cmd_control {
        uint8_t header1; //FF
        uint8_t header2; //01
        uint8_t header3; //0F
        uint8_t header4; //10
        uint8_t roll_control_mode;
        uint8_t pitch_control_mode;
        uint8_t yaw_control_mode;
        uint8_t roll_speed_low_byte;
        uint8_t roll_speed_high_byte;
        uint8_t roll_angle_low_byte;
        uint8_t roll_angle_high_byte;
        uint8_t pitch_speed_low_byte;
        uint8_t pitch_speed_high_byte;
        uint8_t pitch_angle_low_byte;
        uint8_t pitch_angle_high_byte;
        uint8_t yaw_speed_low_byte;
        uint8_t yaw_speed_high_byte;
        uint8_t yaw_angle_low_byte;
        uint8_t yaw_angle_high_byte;
        //form roll_control_mode to yaw_angle_high_byte
        uint8_t check_sum;
    };

    void angle_can_send();
    
    // internal variables
    AP_HAL::UARTDriver *_port;
    
    bool _initialised;              // true once the driver has been initialised
    uint32_t _last_send;            // system time of last do_mount_control sent to gimbal

    Vector3f _angle_bf_output_deg; 
};
