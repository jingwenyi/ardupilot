#include "Plane.h"

// motor test definitions
#define SERVO_MOTOR_TEST_PWM_MIN              800     // min pwm value accepted by the test
#define SERVO_MOTOR_TEST_PWM_MAX              2200    // max pwm value accepted by the test
#define SERVO_MOTOR_TEST_TIMEOUT_MS_MAX       30000   // max timeout is 30 seconds

// servo_motor_test_output - checks for timeout and sends updates to motors objects
void Plane::servo_motor_test_output()
{
    // exit immediately if the motor test is not running
    if (!servo_motor_test.running) {
        return;
    }

    // check for test timeout
    uint32_t now = AP_HAL::millis();
    if ((now - servo_motor_test.start_ms) >= servo_motor_test.timeout_ms) {
        // stop motor test
        servo_motor_test_stop();
        return;
    }

    if(servo_motor_test.value < SERVO_MOTOR_TEST_PWM_MIN || servo_motor_test.value > SERVO_MOTOR_TEST_PWM_MAX){
        servo_motor_test_stop();
        return;
    }

    uint8_t chan;
            
    switch(servo_motor_test.servo_type){

        case SERVO_MOTOR_TEST_AILERON:
        {
            if(true == SRV_Channels::find_channel(SRV_Channel::k_aileron, chan)){
        
                hal.rcout->write(chan, servo_motor_test.value);
            }
            break;
        }
        case SERVO_MOTOR_TEST_ELEVATOR:
        {
            if(true == SRV_Channels::find_channel(SRV_Channel::k_elevator, chan)){
        
                    hal.rcout->write(chan, servo_motor_test.value);
            }
            break;
        }
        case SERVO_MOTOR_TEST_RUDDER:
        {
            if(true == SRV_Channels::find_channel(SRV_Channel::k_rudder, chan)){
        
                hal.rcout->write(chan, servo_motor_test.value);
            }
            break;
        }

        case SERVO_MOTOR_TEST_THROTTLE:
        {
            if(true == SRV_Channels::find_channel(SRV_Channel::k_throttle, chan)){
        
                hal.rcout->write(chan, servo_motor_test.value);
            }
            break;

        }
        case SERVO_MOTOR_TEST_VTAIL_LEFT:
        {
            
            if(true == SRV_Channels::find_channel(SRV_Channel::k_vtail_left, chan)){
                    
                hal.rcout->write(chan, servo_motor_test.value);
            }
            break;
        }
        case SERVO_MOTOR_TEST_VTAIL_RIGHT:
        {
            
            if(true == SRV_Channels::find_channel(SRV_Channel::k_vtail_right, chan)){
                    
                hal.rcout->write(chan, servo_motor_test.value);
            }
            break;
        }
        
       default:
            servo_motor_test_stop();
    }

}





// mavlink_motor_test_start - start motor test - spin a single motor at a specified pwm
//  returns MAV_RESULT_ACCEPTED on success, MAV_RESULT_FAILED on failure
uint8_t Plane::mavlink_servo_motor_test_start(mavlink_channel_t chan, uint8_t type,
                                            uint16_t value, float timeout_sec)
{
    if (arming.is_armed()) {
        gcs().send_text(MAV_SEVERITY_INFO, "Must be disarmed for motor test");
        return MAV_RESULT_FAILED;
    }
    // if test has not started try to start it
    if (!servo_motor_test.running) {
        // start test
        servo_motor_test.running = true;
    }

    // set timeout
    servo_motor_test.start_ms = AP_HAL::millis();
    servo_motor_test.timeout_ms = MIN(timeout_sec * 1000, SERVO_MOTOR_TEST_TIMEOUT_MS_MAX);

    // store required output
    servo_motor_test.servo_type = type;
    servo_motor_test.value = value;

    // return success
    return MAV_RESULT_ACCEPTED;
}

// motor_test_stop - stops the motor test
void Plane::servo_motor_test_stop()
{
    // exit immediately if the test is not running
    if (!servo_motor_test.running) {
        return;
    }

    // flag test is complete
    servo_motor_test.running = false;

    // reset timeout
    servo_motor_test.start_ms = 0;
    servo_motor_test.timeout_ms = 0;
}

