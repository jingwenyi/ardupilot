#include "AP_Mount_Z6ka7_Servo.h"

extern const AP_HAL::HAL& hal;

// init - performs any required initialisation for this instance
void AP_Mount_Z6ka7_Servo::init(const AP_SerialManager& serial_manager)
{
    if (_instance == 0) {
        _roll_idx = SRV_Channel::k_mount_roll;
        _tilt_idx = SRV_Channel::k_mount_tilt;
        _pan_idx  = SRV_Channel::k_mount_pan;
    } else {
        // this must be the 2nd mount
        _roll_idx = SRV_Channel::k_mount2_roll;
        _tilt_idx = SRV_Channel::k_mount2_tilt;
        _pan_idx  = SRV_Channel::k_mount2_pan;
    }

    // check which servos have been assigned
    check_servo_map();
}

// update mount position - should be called periodically
void AP_Mount_Z6ka7_Servo::update()
{
    // check servo map every three seconds to allow users to modify parameters
    uint32_t now = AP_HAL::millis();
    if (now - _last_check_servo_map_ms > 3000) {
        check_servo_map();
        _last_check_servo_map_ms = now;
    }

    switch(get_mode()) {
        // move mount to a neutral position, typically pointing forward
        case MAV_MOUNT_MODE_NEUTRAL:
        {
            _angle_bf_output_deg = _state._neutral_angles.get();
            break;
        }
        default:
            //do nothing
            break;
    }

    SRV_Channels::set_output_pwm(_roll_idx, _angle_bf_output_deg.x);
    SRV_Channels::set_output_pwm(_tilt_idx, _angle_bf_output_deg.y);
    SRV_Channels::set_output_pwm(_pan_idx, _angle_bf_output_deg.z);
    
}

// set_mode - sets mount's mode
void AP_Mount_Z6ka7_Servo::set_mode(enum MAV_MOUNT_MODE mode)
{
    // record the mode change and return success
    _state._mode = mode;
}

// private methods

// check_servo_map - detects which axis we control using the functions assigned to the servos in the RC_Channel_aux
//  should be called periodically (i.e. 1hz or less)
void AP_Mount_Z6ka7_Servo::check_servo_map()
{
    _flags.roll_control = SRV_Channels::function_assigned(_roll_idx);
    _flags.tilt_control = SRV_Channels::function_assigned(_tilt_idx);
    _flags.pan_control = SRV_Channels::function_assigned(_pan_idx);
}
