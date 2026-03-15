#include "Copter.h"

#if MODE_THROW_ENABLED

const AP_Param::GroupInfo ModeDrop::var_info[] = {

    // @Param: _ACCZ
    // @DisplayName: Vertical acceleration threshold for freefall detection
    // @Description: The vertical acceleration threshold for freefall detection. If the vertical acceleration is greater than this threshold, the copter is considered to be in freefall. Expressed in g.
    // @User: Standard
    AP_GROUPINFO("_ACCZ", 0, ModeDrop, _free_fall_accz, 0.8f),

    // @Param: _VZ_FALL
    // @DisplayName: Vertical velocity threshold for initial drop detection
    // @Description: Downward velocity threshold (m/s) used to latch the start of the drop. Set to 0 to use a small default threshold.
    // @User: Standard
    AP_GROUPINFO("_VZ_FALL", 1, ModeDrop, _free_fall_vz, 0.0f),

    // @Param: _ALT_DROP
    // @DisplayName: Minimum altitude dropped to trigger recovery
    // @Description: Recovery can start when the vehicle has dropped at least this amount (m) from the release point.
    // @User: Standard
    AP_GROUPINFO("_ALT_DROP", 2, ModeDrop, _alt_drop, 0.5f),

    // @Param: _VZ_RECOVERY
    // @DisplayName: Vertical velocity threshold for recovery initiation
    // @Description: The vertical velocity threshold for recovery initiation. If the vertical velocity is below this threshold, the copter will initiate recovery.
    // @User: Standard
    AP_GROUPINFO("_VZ_RECOVERY", 3, ModeDrop, _vz_recovery, 1.0f),

    // @Param: _TIME_DROP
    // @DisplayName: Time threshold for recovery initiation
    // @Description: Recovery can start this many milliseconds after drop detection.
    // @Units: ms
    // @User: Standard
    AP_GROUPINFO("_TIME_DROP", 4, ModeDrop, _t_drop_ms, 300),

    // @Param: _ALT_MIN
    // @DisplayName: Drop mode minimum recovery altitude
    // @Description: Minimum altitude above which Drop mode will initiate recovery. Set to 0 to disable.
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("_ALT_MIN", 5, ModeDrop, _altitude_min, 0),

    // @Param: _ALT_MAX
    // @DisplayName: Drop mode maximum recovery altitude
    // @Description: Maximum altitude under which Drop mode will initiate recovery. Set to 0 to disable.
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("_ALT_MAX", 6, ModeDrop, _altitude_max, 0),

    // @Param: _NEXTMODE
    // @DisplayName: Drop mode's follow up mode
    // @Description: Vehicle will switch to this mode after the drop is successfully completed.
    // @Values: 3:Auto,4:Guided,5:Loiter,6:RTL,9:Land,17:Brake,29:Drop
    // @User: Standard
    AP_GROUPINFO("_NEXTMODE", 7, ModeDrop, _nextmode, 29),

    // @Param: _OVRD_CH
    // @DisplayName: Recovery RC override channel
    // @Description: RC channel to use for override of Drop mode recovery. Set to 0 to disable.
    // @Range: 1 16
    // @User: Standard
    AP_GROUPINFO("_OVRD_CH", 8, ModeDrop, _override_channel, 10),

    // @Param: _RCV_THR
    // @DisplayName: Throttle output during initial recovery stage
    // @Description: Throttle output during initial recovery stage.
    // @Range: 0.0 1.0
    // @User: Standard
    AP_GROUPINFO("_RCV_THR", 9, ModeDrop, _recovery_throttle, 0.5f),

    AP_GROUPEND
};

ModeDrop::ModeDrop(void) : Mode()
{
    AP_Param::setup_object_defaults(this, var_info);
}

// helper: altitude above home in meters
static float drop_altitude_above_home_m(const AP_AHRS &ahrs, const AP_InertialNav &inertial_nav)
{
    float altitude_above_home;
    if (ahrs.home_is_set()) {
        ahrs.get_relative_position_D_home(altitude_above_home);
        altitude_above_home = -altitude_above_home;   // returned as negative down
    } else {
        altitude_above_home = inertial_nav.get_position_z_up_cm() * 0.01f;
    }
    return altitude_above_home;
}

// initialise drop controller
bool ModeDrop::init(bool ignore_checks)
{
#if FRAME_CONFIG == HELI_FRAME
    // do not allow helis to use throw to start
    return false;
#endif

    // require at least one recovery trigger
    if (_vz_recovery <= 0.0f && _alt_drop <= 0.0f && _t_drop_ms <= 0) {
        gcs().send_text(MAV_SEVERITY_ERROR, "DROP init failed: no recovery trigger");
        return false;
    }

    // must enter while disarmed
    if (motors->armed()) {
        gcs().send_text(MAV_SEVERITY_ERROR, "DROP init failed: arm after mode select");
        return false;
    }

    free_fall_start_ms = 0;
    free_fall_start_velz = 0.0f;
    free_fall_start_alt = 0.0f;

    // init state
    stage = Throw_Disarmed;
    nextmode_attempted = false;

    // initialise pos controller speed and acceleration
    pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), BRAKE_MODE_DECEL_RATE);
    pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(), BRAKE_MODE_DECEL_RATE);

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(BRAKE_MODE_SPEED_Z, BRAKE_MODE_SPEED_Z, BRAKE_MODE_DECEL_RATE);
    pos_control->set_correction_speed_accel_z(BRAKE_MODE_SPEED_Z, BRAKE_MODE_SPEED_Z, BRAKE_MODE_DECEL_RATE);

    return true;
}

// runs the drop controller
// should be called at 100hz or more
void ModeDrop::run()
{
    /* Throw State Machine
    Throw_Disarmed - motors are off
    Throw_Detecting -  motors are on and we are waiting for the throw
    Throw_Uprighting - the throw has been detected and the copter is being uprighted
    Throw_HgtStabilise - the copter is kept level and  height is stabilised about the target height
    Throw_PosHold - the copter is kept at a constant position and height
    */

    if (!motors->armed()) {
        // state machine entry is always from a disarmed state
        stage = Throw_Disarmed;
        free_fall_start_ms = 0;
        nextmode_attempted = false;

    } else if (stage == Throw_Disarmed && motors->armed()) {

        // prevent disarm while suspended and waiting
        copter.set_auto_armed(true);
        copter.set_land_complete(false);

        gcs().send_text(MAV_SEVERITY_INFO, "Waiting for drop");
        stage = Throw_Detecting;

    } else if (stage == Throw_Detecting && throw_detected()) {
        gcs().send_text(MAV_SEVERITY_INFO, "Initiating recovery - spooling motors");
        copter.set_land_complete(false);
        copter.set_auto_armed(true);
        stage = Throw_Wait_Throttle_Unlimited;

        // Cancel the waiting for throw tone sequence
        AP_Notify::flags.waiting_for_throw = false;

    } else if (stage == Throw_Wait_Throttle_Unlimited &&
               motors->get_spool_state() == AP_Motors::SpoolState::THROTTLE_UNLIMITED) {
        gcs().send_text(MAV_SEVERITY_INFO, "Throttle is unlimited - uprighting");
        stage = Throw_Uprighting;

    } else if (stage == Throw_Uprighting && throw_attitude_good()) {
        gcs().send_text(MAV_SEVERITY_INFO, "Uprighted - controlling height");
        stage = Throw_HgtStabilise;

        // initialise the z controller
        pos_control->init_z_controller_no_descent();

        // initialise the demanded height to 3m above the current height
        // we want to rapidly clear surrounding obstacles
        pos_control->set_pos_desired_z_cm(inertial_nav.get_position_z_up_cm() + 300.0f);

        // Set the auto_arm status to true to avoid a possible automatic disarm caused by selection of an auto mode with throttle at minimum
        copter.set_auto_armed(true);

    } else if (stage == Throw_HgtStabilise && throw_height_good()) {
        gcs().send_text(MAV_SEVERITY_INFO, "Height achieved - controlling position");
        stage = Throw_PosHold;

        // initialise position controller
        pos_control->init_xy_controller();

        // Set the auto_arm status to true to avoid a possible automatic disarm caused by selection of an auto mode with throttle at minimum
        copter.set_auto_armed(true);

    } else if (stage == Throw_PosHold && throw_position_good()) {
        if (!nextmode_attempted) {
            switch ((Mode::Number)_nextmode.get()) {
                case Mode::Number::AUTO:
                case Mode::Number::GUIDED:
                case Mode::Number::RTL:
                case Mode::Number::LAND:
                case Mode::Number::BRAKE:
                case Mode::Number::LOITER:
                    set_mode((Mode::Number)_nextmode.get(), ModeReason::THROW_COMPLETE);
                    break;
                default:
                    // do nothing
                    break;
            }
            nextmode_attempted = true;
        }
    }

    // Throw State Processing
    switch (stage) {

    case Throw_Disarmed:

        // prevent motors from rotating before the throw is detected unless enabled by the user
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);

        // demand zero throttle (motors will be stopped anyway) and continually reset the attitude controller
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_throttle_out(0.0f, true, g.throttle_filt);
        break;

    case Throw_Detecting:

        // prevent motors from rotating before the throw is detected unless enabled by the user
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);

        // Hold throttle at zero during the throw and continually reset the attitude controller
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_throttle_out(0.0f, true, g.throttle_filt);

        // Play the waiting for throw tone sequence to alert the user
        AP_Notify::flags.waiting_for_throw = true;
        copter.set_auto_armed(true);
        break;

    case Throw_Wait_Throttle_Unlimited:

        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        break;

    case Throw_Uprighting:

        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // demand a level roll/pitch attitude with zero yaw rate
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0.0f, 0.0f, 0.0f);

        // output 50% throttle and turn off angle boost to maximise righting moment
        attitude_control->set_throttle_out(_recovery_throttle.get(), false, g.throttle_filt);

        break;

    case Throw_HgtStabilise:

        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0.0f, 0.0f, 0.0f);

        // call height controller
        pos_control->set_pos_target_z_from_climb_rate_cm(0.0f);
        pos_control->update_z_controller();

        break;

    case Throw_PosHold:

        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // use position controller to stop
        Vector2f vel;
        Vector2f accel;
        pos_control->input_vel_accel_xy(vel, accel);
        pos_control->update_xy_controller();

        // call attitude controller
        attitude_control->input_thrust_vector_rate_heading(pos_control->get_thrust_vector(), 0.0f);

        // call height controller
        pos_control->set_pos_target_z_from_climb_rate_cm(0.0f);
        pos_control->update_z_controller();

        break;
    }

#if HAL_LOGGING_ENABLED
    // log at 10hz or if stage changes
    uint32_t now = AP_HAL::millis();
    if ((stage != prev_stage) || (now - last_log_ms) > 100) {
        prev_stage = stage;
        last_log_ms = now;

        const float velocity = inertial_nav.get_velocity_neu_cms().length() * 0.01f;
        const float velocity_z = inertial_nav.get_velocity_z_up_cms() * 0.01f;
        const float accel = copter.ins.get_accel().length();
        const float ef_accel_z = ahrs.get_accel_ef().z;
        const bool throw_detect = (stage > Throw_Detecting) || throw_detected();
        const bool attitude_ok = (stage > Throw_Uprighting) || throw_attitude_good();
        const bool height_ok = (stage > Throw_HgtStabilise) || throw_height_good();
        const bool pos_ok = (stage > Throw_PosHold) || throw_position_good();

// @LoggerMessage: DROP
// @Description: Drop Mode messages
// @URL: https://ardupilot.org/copter/docs/throw-mode.html
// @Field: TimeUS: Time since system startup
// @Field: Stage: Current stage of the Throw Mode
// @Field: Vel: Magnitude of the velocity vector
// @Field: VelZ: Vertical Velocity
// @Field: Acc: Magnitude of the vector of the current acceleration
// @Field: AccEfZ: Vertical earth frame accelerometer value
// @Field: Throw: True if a throw has been detected since entering this mode
// @Field: AttOk: True if the vehicle is upright 
// @Field: HgtOk: True if the vehicle is within 50cm of the demanded height
// @Field: PosOk: True if the vehicle is within 50cm of the demanded horizontal position

        AP::logger().WriteStreaming(
            "DROP",
            "TimeUS,Stage,Vel,VelZ,Acc,AccEfZ,Throw,AttOk,HgtOk,PosOk",
            "s-nnoo----",
            "F-0000----",
            "QBffffbbbb",
            AP_HAL::micros64(),
            (uint8_t)stage,
            (double)velocity,
            (double)velocity_z,
            (double)accel,
            (double)ef_accel_z,
            throw_detect,
            attitude_ok,
            height_ok,
            pos_ok);
    }
#endif  // HAL_LOGGING_ENABLED
}

bool ModeDrop::throw_detected()
{
    // Check that we have a valid navigation solution
    nav_filter_status filt_status = inertial_nav.get_filter_status();
    if (!filt_status.flags.attitude || !filt_status.flags.vert_pos) {
        return false;
    }

    // RC override
    if (_override_channel.get() != 0 && _override_channel.get() <= 16) {
        // Valid override channel configured
        RC_Channel *ch = rc().channel((uint8_t)_override_channel.get() - 1);
        if (ch != nullptr) {
            const int switch_pwm = ch->get_radio_in();
            if (switch_pwm > 1500) {
                // Immediately trigger recovery
                gcs().send_text(MAV_SEVERITY_NOTICE, "Drop recovery override activated");
                return true;
            }
        }
    }

    const float altitude_above_home = drop_altitude_above_home_m(ahrs, inertial_nav);

    // Check the vertical acceleration is greater than the free fall threshold. Use the earth frame z acceleration which is less noisy than the body frame measurement for this check
    bool free_falling = ahrs.get_accel_ef().z > -(1.0f - _free_fall_accz.get()) * GRAVITY_MSS;

    if (_free_fall_vz > 0.0f) {
        free_falling = free_falling && (inertial_nav.get_velocity_z_up_cms() < -_free_fall_vz.get() * 100.0f);
    }

    if (free_falling && free_fall_start_ms == 0) {
        free_fall_start_ms = AP_HAL::millis();
        free_fall_start_velz = inertial_nav.get_velocity_z_up_cms();
        free_fall_start_alt = altitude_above_home;
        gcs().send_text(MAV_SEVERITY_INFO, "Drop detected");
    }

    // if drop was not latched yet, do not recover yet
    if (free_fall_start_ms == 0) {
        return false;
    }

    // Check if the accel length is < 1.2g indicating that any throw action is complete and the copter has been released
    const bool no_throw_action = ahrs.get_accel_ef().length() < (GRAVITY_MSS * 1.2f);

    const bool changing_height =
        (_vz_recovery > 0.0f) && (inertial_nav.get_velocity_z_up_cms() < -_vz_recovery * 100.0f);

    const bool dropped_altitude =
        (_alt_drop > 0.0f) && ((free_fall_start_alt - altitude_above_home) > _alt_drop);

    const bool time_elapsed =
        (_t_drop_ms > 0) &&
        ((AP_HAL::millis() - free_fall_start_ms) > (uint32_t)_t_drop_ms);

    const bool height_within_params =
        (_altitude_min == 0 || altitude_above_home > _altitude_min) &&
        (_altitude_max == 0 || altitude_above_home < _altitude_max);

    return free_falling && no_throw_action  && height_within_params && (changing_height || dropped_altitude || time_elapsed);
}

bool ModeDrop::throw_attitude_good() const
{
    // Check that we have uprighted the copter
    const Matrix3f &rotMat = ahrs.get_rotation_body_to_ned();
    return (rotMat.c.z > 0.866f); // is_upright
}

bool ModeDrop::throw_height_good() const
{
    const float pos_err_z_cm = pos_control->get_pos_error_z_cm();
    const float vel_z_up_cms = inertial_nav.get_velocity_z_up_cms();

    // Only declare height recovered if we are close to target
    // and no longer descending significantly
    return (pos_err_z_cm < 50.0f) && (vel_z_up_cms > -50.0f);
}

bool ModeDrop::throw_position_good() const
{
    const float pos_err_xy_cm = pos_control->get_pos_error_xy_cm();
    const Vector3f vel_neu_cms = inertial_nav.get_velocity_neu_cms();
    const float horiz_speed_cms = sqrtf(vel_neu_cms.x * vel_neu_cms.x + vel_neu_cms.y * vel_neu_cms.y);
    return (pos_err_xy_cm < 50.0f) && (horiz_speed_cms < 100.0f);
}

#endif
