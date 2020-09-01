/*
  additional arming checks for plane
 */
#include "AP_Arming.h"
#include "Plane.h"

const AP_Param::GroupInfo AP_Arming_Plane::var_info[] = {
    // variables from parent vehicle
    AP_NESTEDGROUPINFO(AP_Arming, 0),

    // @Param: RUDDER
    // @DisplayName: Rudder Arming
    // @Description: Control arm/disarm by rudder input. When enabled arming is done with right rudder, disarming with left rudder. Rudder arming only works in manual throttle modes with throttle at zero +- deadzone (RCx_DZ)
    // @Values: 0:Disabled,1:ArmingOnly,2:ArmOrDisarm
    // @User: Advanced
    AP_GROUPINFO("RUDDER",       3,     AP_Arming_Plane,  rudder_arming_value,     ARMING_RUDDER_ARMDISARM),

    AP_GROUPEND
};

enum HomeState AP_Arming_Plane::home_status() const
{
    return plane.home_is_set;
}

/*
  additional arming checks for plane

 */
bool AP_Arming_Plane::pre_arm_checks(bool report)
{
    // call parent class checks
    bool ret = AP_Arming::pre_arm_checks(report);

    // Check airspeed sensor
    ret &= AP_Arming::airspeed_checks(report);

    if(plane.load_param_flag != Plane::LOAD_PARAM_OK){
        if (report) {
            if(plane.load_param_flag == Plane::LOAD_PARAM_FAILED){
                gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm:init load parameter error");
            }else if(plane.load_param_flag == Plane::PARAM_VERSION_ERR){
                gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm:init parameter version error");
            }
        }
        ret = false;
    }

    //check rtk heading

    float rtk_heading = 0.0f;
    float compass_heading = 0.0f;
    bool use_gps_head = plane.gps.have_heading() && plane.gps.heading_status() == AP_GPS::GPS_OK_FIX_3D_RTK_FIXED;
    if(use_gps_head){
        rtk_heading = plane.gps.get_heading();
    }
    
    if(plane.compass.use_for_yaw()){
        compass_heading = plane.compass.calculate_heading(plane.ahrs.get_rotation_body_to_ned())*RAD_TO_DEG;
        if(compass_heading < 0 ){
            compass_heading = 360.0f + compass_heading;
        }
    }

    float heading_error = fabs(rtk_heading - compass_heading);
    heading_error = heading_error > 180 ? 360 - heading_error : heading_error;

    if(!use_gps_head || (plane.compass.use_for_yaw() && use_gps_head && heading_error > 90.0f)){
        if (report) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: rtk heading is not heathly");
        }
        ret = false;
    }



    if (plane.aparm.roll_limit_cd < 300) {
        if (report) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: LIM_ROLL_CD too small (%u)", plane.aparm.roll_limit_cd);
        }
        ret = false;        
    }

    if (plane.aparm.pitch_limit_max_cd < 300) {
        if (report) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: LIM_PITCH_MAX too small (%u)", plane.aparm.pitch_limit_max_cd);
        }
        ret = false;        
    }

    if (plane.aparm.pitch_limit_min_cd > -300) {
        if (report) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: LIM_PITCH_MIN too large (%u)", plane.aparm.pitch_limit_min_cd);
        }
        ret = false;        
    }

    if (plane.channel_throttle->get_reverse() && 
        plane.g.throttle_fs_enabled &&
        plane.g.throttle_fs_value < 
        plane.channel_throttle->get_radio_max()) {
        if (report) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: Invalid THR_FS_VALUE for rev throttle");
        }
        ret = false;
    }

    if (plane.quadplane.available() && plane.scheduler.get_loop_rate_hz() < 100) {
        if (report) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: quadplane needs SCHED_LOOP_RATE > 100");
        }
        ret = false;
    }

    if (plane.control_mode == AUTO && plane.mission.num_commands() <= 1) {
        if (report) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: No mission loaded");
        }
        ret = false;
    }

    // check adsb avoidance failsafe
    if (plane.failsafe.adsb) {
        if (report) {
            gcs().send_text(MAV_SEVERITY_CRITICAL,"PreArm: ADSB threat detected");
        }
        ret = false;
    }

#if HAVE_PX4_MIXER
    if (plane.last_mixer_crc == -1) {
        if (report) {
            // if you ever get this error, a reboot is recommended.
            gcs().send_text(MAV_SEVERITY_CRITICAL,"PreArm: Mixer error");
        }
        ret = false;
    }
#endif // CONFIG_HAL_BOARD

    return ret;
}

bool AP_Arming_Plane::ins_checks(bool report)
{
    // call parent class checks
    if (!AP_Arming::ins_checks(report)) {
        return false;
    }

    // additional plane specific checks
    if ((checks_to_perform & ARMING_CHECK_ALL) ||
        (checks_to_perform & ARMING_CHECK_INS)) {
        if (!ahrs.healthy()) {
            if (report) {
                const char *reason = ahrs.prearm_failure_reason();
                if (reason) {
                    gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: %s", reason);
                } else {
                    gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: AHRS not healthy");
                }
            }
            return false;
        }
    }

    return true;
}
