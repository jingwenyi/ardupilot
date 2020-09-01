#include "Plane.h"

void Plane::failsafe_short_on_event(enum failsafe_state fstype, mode_reason_t reason)
{
    // This is how to handle a short loss of control signal failsafe.
    failsafe.state = fstype;
    failsafe.ch3_timer_ms = millis();
    gcs().send_text(MAV_SEVERITY_WARNING, "Failsafe. Short event on, ");
    switch(control_mode)
    {
    case MANUAL:
    case STABILIZE:
    case ACRO:
    case FLY_BY_WIRE_A:
    case AUTOTUNE:
    case FLY_BY_WIRE_B:
    case CRUISE:
    case TRAINING:
        failsafe.saved_mode = control_mode;
        failsafe.saved_mode_set = 1;
        if(g.short_fs_action == 2) {
            set_mode(FLY_BY_WIRE_A, reason);
        } else {
            set_mode(CIRCLE, reason);
        }
        break;

    case QSTABILIZE:
    case QLOITER:
    case QHOVER:
        failsafe.saved_mode = control_mode;
        failsafe.saved_mode_set = 1;
        set_mode(QLAND, reason);
        break;
        
    case AUTO:
    case AVOID_ADSB:
    case GUIDED:
    case LOITER:
        if(g.short_fs_action != 0) {
            failsafe.saved_mode = control_mode;
            failsafe.saved_mode_set = 1;
            if(g.short_fs_action == 2) {
                set_mode(FLY_BY_WIRE_A, reason);
            } else {
                set_mode(CIRCLE, reason);
            }
        }
        break;

    case CIRCLE:
    case RTL:
    case QLAND:
    case QRTL:
    default:
        break;
    }
    gcs().send_text(MAV_SEVERITY_INFO, "Flight mode = %u", (unsigned)control_mode);
}

void Plane::failsafe_long_on_event(enum failsafe_state fstype, mode_reason_t reason)
{
    // This is how to handle a long loss of control signal failsafe.
    gcs().send_text(MAV_SEVERITY_WARNING, "Failsafe. Long event on, ");
    //  If the GCS is locked up we allow control to revert to RC
    hal.rcin->clear_overrides();
    failsafe.state = fstype;
    switch(control_mode)
    {
    case MANUAL:
    case STABILIZE:
    case ACRO:
    case FLY_BY_WIRE_A:
    case AUTOTUNE:
    case FLY_BY_WIRE_B:
    case CRUISE:
    case TRAINING:
    case CIRCLE:
        if(g.long_fs_action == 3) {
#if PARACHUTE == ENABLED
            parachute_release();
#endif
        } else if (g.long_fs_action == 2) {
            set_mode(FLY_BY_WIRE_A, reason);
        } else {
            set_mode(RTL, reason);
        }
        break;

    case QSTABILIZE:
    case QHOVER:
    case QLOITER:
        set_mode(QLAND, reason);
        break;
        
    case AUTO:
    case AVOID_ADSB:
    case GUIDED:
    case LOITER:
        if(g.long_fs_action == 3) {
#if PARACHUTE == ENABLED
            parachute_release();
#endif
        } else if (g.long_fs_action == 2) {
            set_mode(FLY_BY_WIRE_A, reason);
        } else if (g.long_fs_action == 1 && !emergency_return) {
            //set_mode(RTL, reason);
            if(control_mode == GUIDED && mission.starts_check_misson_cmd()){
                set_mode(AUTO, reason);
            }

            if(control_mode == AUTO && mission.starts_check_misson_cmd()){
                if(mission.get_current_nav_index() < mission.num_commands() - 9){
                    uint16_t seq = mission.num_commands() - 9;
                    camera.set_trigger_distance(0);
                    mission.set_current_cmd(seq);
                }
            }else{
                set_mode(RTL, reason);
            }

            event_report = PLANE_EVENT_REPORT_RC_GCS_LOSE;
            gcs().send_message(MSG_EVENT_REPORT);
        }
        break;

    case RTL:
    case QLAND:
    case QRTL:
    default:
        break;
    }
    if (fstype == FAILSAFE_GCS) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "No GCS heartbeat");
    }
    gcs().send_text(MAV_SEVERITY_INFO, "Flight mode = %u", (unsigned)control_mode);
}

void Plane::failsafe_short_off_event(mode_reason_t reason)
{
    // We're back in radio contact
    gcs().send_text(MAV_SEVERITY_WARNING, "Failsafe. Short event off");
    failsafe.state = FAILSAFE_NONE;

    // re-read the switch so we can return to our preferred mode
    // --------------------------------------------------------
    if (control_mode == CIRCLE && failsafe.saved_mode_set) {
        failsafe.saved_mode_set = 0;
        set_mode(failsafe.saved_mode, reason);
    }
}

void Plane::low_battery_event(void)
{
    if (failsafe.low_battery || emergency_return || flight_stage !=AP_Vehicle::FixedWing::FLIGHT_NORMAL) {
        if(emergency_return){
            failsafe.low_battery = true;
            AP_Notify::flags.failsafe_battery = true;
        }
        return;
    }
    gcs().send_text(MAV_SEVERITY_WARNING, "one-level Low battery %.2fV used %.0f mAh",
                      (double)battery.voltage(), (double)battery.current_total_mah());

    //Return to emergency point
    if(control_mode == GUIDED && mission.starts_check_misson_cmd() && arming.is_armed()){
        set_mode(AUTO, MODE_REASON_BATTERY_FAILSAFE);
    }

    if(control_mode == AUTO && arming.is_armed() && mission.starts_check_misson_cmd()){
        camera.set_trigger_distance(0);
        if(mission.get_current_nav_index() <= 2){
            uint16_t seq = mission.num_commands() - 7;
            mission.set_current_cmd(seq);
        }else if(mission.get_current_nav_index() < mission.num_commands() - 9){
            uint16_t seq = mission.num_commands() - 9;
            mission.set_current_cmd(seq);
        }
    }else if(control_mode != NO_GPS_RTL){
        set_mode(RTL, MODE_REASON_BATTERY_FAILSAFE);
    }

    event_report = PLANE_EVENT_REPORT_LOW_BATT_ONE;
    gcs().send_message(MSG_EVENT_REPORT);

    failsafe.low_battery = true;
    AP_Notify::flags.failsafe_battery = true;
}

void Plane::low_battery_event2(void)
{
    if (failsafe.low_battery2 || emergency_return || flight_stage !=AP_Vehicle::FixedWing::FLIGHT_NORMAL) {
        if(emergency_return){
            failsafe.low_battery2 = true;
            AP_Notify::flags.failsafe_battery = true;
        }
        return;
    }
    gcs().send_text(MAV_SEVERITY_WARNING, "two-levelLow battery %.2fV used %.0f mAh",
                      (double)battery.voltage(), (double)battery.current_total_mah());

    
    //Return to emergency point
    if(control_mode == GUIDED && mission.starts_check_misson_cmd() && arming.is_armed()){
        set_mode(AUTO, MODE_REASON_BATTERY_FAILSAFE);
    }

    if(control_mode == AUTO && arming.is_armed() && mission.starts_check_misson_cmd()){
        if(mission.get_current_nav_index() < mission.num_commands() - 6){
            camera.set_trigger_distance(0);
            uint16_t seq = mission.num_commands() - 6;
            mission.set_current_cmd(seq);
        }
    }else if(control_mode != NO_GPS_RTL){
        set_mode(RTL, MODE_REASON_BATTERY_FAILSAFE);
    }

	emergency_return = true;
    
    event_report = PLANE_EVENT_REPORT_LOW_BATT_TWO;
    gcs().send_message(MSG_EVENT_REPORT);

    failsafe.low_battery2 = true;
    AP_Notify::flags.failsafe_battery = true;
}


void Plane::update_events(void)
{
    ServoRelayEvents.update_events();
}
