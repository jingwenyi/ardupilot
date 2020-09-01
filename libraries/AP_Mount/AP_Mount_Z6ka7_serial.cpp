#include "AP_Mount_Z6ka7_serial.h"
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <GCS_MAVLink/include/mavlink/v2.0/checksum.h>
#include <AP_HAL/utility/RingBuffer.h>

extern const AP_HAL::HAL& hal;

AP_Mount_Z6ka7_serial::AP_Mount_Z6ka7_serial(AP_Mount &frontend, AP_Mount::mount_state &state, uint8_t instance) :
    AP_Mount_Backend(frontend, state, instance),
    _port(nullptr),
    _initialised(false),
    _last_send(0)
{}



// init - performs any required initialisation for this instance
void AP_Mount_Z6ka7_serial::init(const AP_SerialManager& serial_manager)
{
    _port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Z6ka7, 0);
    if (_port) {
        _initialised = true;
        set_mode((enum MAV_MOUNT_MODE)_state._default_mode.get());
    }
}


// set_mode - sets mount's mode
void AP_Mount_Z6ka7_serial::set_mode(enum MAV_MOUNT_MODE mode)
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    // record the mode change
    _state._mode = mode;
}



// update mount position - should be called periodically
void AP_Mount_Z6ka7_serial::update()
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    // flag to trigger sending target angles to gimbal
    bool resend_now = false;

    // update based on mount mode
    switch(get_mode()) {
        // move mount to a "retracted" position.  To-Do: remove support and replace with a relaxed mode?
        case MAV_MOUNT_MODE_RETRACT:
        {
            _angle_bf_output_deg = _state._retract_angles.get();
            break;
        }
 
        // move mount to a neutral position, typically pointing forward
        case MAV_MOUNT_MODE_NEUTRAL:
        {
            _angle_bf_output_deg = _state._neutral_angles.get();
            break;
        }

        // point to the angles given by a mavlink message
        case MAV_MOUNT_MODE_MAVLINK_TARGETING:
            // do nothing because earth-frame angle targets (i.e. _angle_ef_target_rad) should have already been set by a MOUNT_CONTROL message from GCS
            break;

        // RC radio manual angle control, but with stabilization from the AHRS
        case MAV_MOUNT_MODE_RC_TARGETING:
            // do nothing
            break;

        // point mount to a GPS point given by the mission planner
        case MAV_MOUNT_MODE_GPS_POINT:
            //do nothing
            break;

        default:
            // we do not know this mode so do nothing
            break;
    }
    
    // resend target angles at least once per second
    resend_now = resend_now || ((AP_HAL::millis() - _last_send) > AP_MOUNT_Z6KA7_SERIAL_RESEND_MS);

    if(resend_now){
        angle_can_send();
    }
    
}



// status_msg - called to allow mounts to send their status to GCS using the MOUNT_STATUS message
void AP_Mount_Z6ka7_serial::status_msg(mavlink_channel_t chan)
{
    // return target angles as gimbal's actual attitude.
    mavlink_msg_mount_status_send(chan, 0, 0, _angle_bf_output_deg.y, _angle_bf_output_deg.x, _angle_bf_output_deg.z);
}



// has_pan_control - returns true if this mount can control it's pan (required for multicopters)
bool AP_Mount_Z6ka7_serial::has_pan_control() const
{
    // we do not have yaw control
    return true;
}

void AP_Mount_Z6ka7_serial::angle_can_send()
{
    static Combine_long_cmd_control cmd_set_angles_data = {
        0xFF,
        0x01,
        0x0F,
        0x10,
        MODE_ANGLE_REL_FRAME, // roll_control_mode
        MODE_ANGLE_REL_FRAME, // pitch_control_mode
        MODE_ANGLE_REL_FRAME, // yaw_contorl_mode
        0, // RSL
        0, // RSH
        0, // RAL
        0, // RAH
        0, // PSL
        0, // PSH
        0, // PAL
        0, // PAH
        0, // YSL
        0, // YSH
        0, // YAL
        0, // YAH
        0, // CS
    };

    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    if ((size_t)_port->txspace() < sizeof(cmd_set_angles_data)) {
        return;
    }


    uint16_t roll_deg = (uint16_t)_angle_bf_output_deg.x / ANGLE_UNITS;
    uint16_t pitch_deg = (uint16_t)_angle_bf_output_deg.y / ANGLE_UNITS;
    uint16_t yaw_deg = (uint16_t)_angle_bf_output_deg.z / ANGLE_UNITS;
   

    // send CMD_SETANGLE
    cmd_set_angles_data.roll_angle_low_byte = roll_deg & 0x0f;
    cmd_set_angles_data.roll_angle_high_byte = roll_deg >> 4;
    cmd_set_angles_data.pitch_angle_low_byte = pitch_deg & 0x0f;
    cmd_set_angles_data.pitch_angle_high_byte = pitch_deg >> 4;
    cmd_set_angles_data.yaw_angle_low_byte = yaw_deg & 0x0f;
    cmd_set_angles_data.yaw_angle_high_byte = yaw_deg >> 4;
    

    uint8_t* buf = (uint8_t*)&cmd_set_angles_data;

    cmd_set_angles_data.check_sum = crc_calculate(&buf[4], sizeof(cmd_set_angles_data)-6);

    for (uint8_t i = 0;  i != sizeof(cmd_set_angles_data) ; i++) {
        _port->write(buf[i]);
    }

    // store time of send
    _last_send = AP_HAL::millis();    
}

