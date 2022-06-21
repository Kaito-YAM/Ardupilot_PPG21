#include "Plane.h"

#define LOCATION_SCALING_FACTOR 0.011131884502145034f
/*
  calculate speed scaling number for control surfaces. This is applied
  to PIDs to change the scaling of the PID with speed. At high speed
  we move the surfaces less, and at low speeds we move them more.
 */
float Plane::calc_speed_scaler(void)
{
	float aspeed, speed_scaler;
	if (ahrs.airspeed_estimate(aspeed)) {
		if (aspeed > auto_state.highest_airspeed) {
			auto_state.highest_airspeed = aspeed;
		}
		if (aspeed > 0.0001f) {
			speed_scaler = g.scaling_speed / aspeed;
		} else {
			speed_scaler = 2.0;
		}
		// ensure we have scaling over the full configured airspeed
		float scale_min = MIN(0.5, (0.5 * aparm.airspeed_min) / g.scaling_speed);
		float scale_max = MAX(2.0, (1.5 * aparm.airspeed_max) / g.scaling_speed);
		speed_scaler = constrain_float(speed_scaler, scale_min, scale_max);

#if HAL_QUADPLANE_ENABLED
		if (quadplane.in_vtol_mode() && hal.util->get_soft_armed()) {
			// when in VTOL modes limit surface movement at low speed to prevent instability
			float threshold = aparm.airspeed_min * 0.5;
			if (aspeed < threshold) {
				float new_scaler = linear_interpolate(0.001, g.scaling_speed / threshold, aspeed, 0, threshold);
				speed_scaler = MIN(speed_scaler, new_scaler);

				// we also decay the integrator to prevent an integrator from before
				// we were at low speed persistint at high speed
				rollController.decay_I();
				pitchController.decay_I();
				yawController.decay_I();
			}
		}
#endif
	} else if (hal.util->get_soft_armed()) {
		// scale assumed surface movement using throttle output
		float throttle_out = MAX(SRV_Channels::get_output_scaled(SRV_Channel::k_throttle), 1);
		speed_scaler = sqrtf(THROTTLE_CRUISE / throttle_out);
		// This case is constrained tighter as we don't have real speed info
		speed_scaler = constrain_float(speed_scaler, 0.6f, 1.67f);
	} else {
		// no speed estimate and not armed, use a unit scaling
		speed_scaler = 1;
	}
	if (!plane.ahrs.airspeed_sensor_enabled()  &&
			(plane.g2.flight_options & FlightOptions::SURPRESS_TKOFF_SCALING) &&
			(plane.flight_stage == AP_Vehicle::FixedWing::FLIGHT_TAKEOFF)) { //scaling is surpressed during climb phase of automatic takeoffs with no airspeed sensor being used due to problems with inaccurate airspeed estimates
		return MIN(speed_scaler, 1.0f) ;
	}
	return speed_scaler;
}

/*
  return true if the current settings and mode should allow for stick mixing
 */
bool Plane::stick_mixing_enabled(void)
{
#if AC_FENCE == ENABLED
	const bool stickmixing = fence_stickmixing();
#else
	const bool stickmixing = true;
#endif
#if HAL_QUADPLANE_ENABLED
	if (control_mode == &mode_qrtl &&
			quadplane.poscontrol.get_state() >= QuadPlane::QPOS_POSITION1) {
		// user may be repositioning
		return false;
	}
	if (quadplane.in_vtol_land_poscontrol()) {
		// user may be repositioning
		return false;
	}
#endif
	if (control_mode->does_auto_throttle() && plane.control_mode->does_auto_navigation()) {
		// we're in an auto mode. Check the stick mixing flag
		if (g.stick_mixing != StickMixing::NONE &&
				g.stick_mixing != StickMixing::VTOL_YAW &&
				stickmixing &&
				failsafe.state == FAILSAFE_NONE &&
				!rc_failsafe_active()) {
			// we're in an auto mode, and haven't triggered failsafe
			return true;
		} else {
			return false;
		}
	}

	if (failsafe.rc_failsafe && g.fs_action_short == FS_ACTION_SHORT_FBWA) {
		// don't do stick mixing in FBWA glide mode
		return false;
	}

	// non-auto mode. Always do stick mixing
	return true;
}


/*
  this is the main roll stabilization function. It takes the
  previously set nav_roll calculates roll servo_out to try to
  stabilize the plane at the given roll
 */
void Plane::stabilize_roll(float speed_scaler)
{
	if (fly_inverted()) {
		// we want to fly upside down. We need to cope with wrap of
		// the roll_sensor interfering with wrap of nav_roll, which
		// would really confuse the PID code. The easiest way to
		// handle this is to ensure both go in the same direction from
		// zero
		nav_roll_cd += 18000;
		if (ahrs.roll_sensor < 0) nav_roll_cd -= 36000;
	}

	const float roll_out = stabilize_roll_get_roll_out(speed_scaler);
	SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, roll_out);
}

float Plane::stabilize_roll_get_roll_out(float speed_scaler)
{
#if HAL_QUADPLANE_ENABLED
	if (!quadplane.use_fw_attitude_controllers()) {
		// use the VTOL rate for control, to ensure consistency
		const auto &pid_info = quadplane.attitude_control->get_rate_roll_pid().get_pid_info();
		const float roll_out = rollController.get_rate_out(degrees(pid_info.target), speed_scaler);
		/* when slaving fixed wing control to VTOL control we need to decay the integrator to prevent
           opposing integrators balancing between the two controllers
		 */
		rollController.decay_I();
		return roll_out;
	}
#endif

	bool disable_integrator = false;
	if (control_mode == &mode_stabilize && channel_roll->get_control_in() != 0) {
		disable_integrator = true;
	}
	return rollController.get_servo_out(nav_roll_cd - ahrs.roll_sensor, speed_scaler, disable_integrator,
			ground_mode && !(plane.g2.flight_options & FlightOptions::DISABLE_GROUND_PID_SUPPRESSION));
}

/*
  this is the main pitch stabilization function. It takes the
  previously set nav_pitch and calculates servo_out values to try to
  stabilize the plane at the given attitude.
 */
void Plane::stabilize_pitch(float speed_scaler)
{
	int8_t force_elevator = takeoff_tail_hold();
	if (force_elevator != 0) {
		// we are holding the tail down during takeoff. Just convert
		// from a percentage to a -4500..4500 centidegree angle
		SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, 45*force_elevator);
		return;
	}

	const float pitch_out = stabilize_pitch_get_pitch_out(speed_scaler);
	SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, pitch_out);
}

float Plane::stabilize_pitch_get_pitch_out(float speed_scaler)
{
#if HAL_QUADPLANE_ENABLED
	if (!quadplane.use_fw_attitude_controllers()) {
		// use the VTOL rate for control, to ensure consistency
		const auto &pid_info = quadplane.attitude_control->get_rate_pitch_pid().get_pid_info();
		const int32_t pitch_out = pitchController.get_rate_out(degrees(pid_info.target), speed_scaler);
		/* when slaving fixed wing control to VTOL control we need to decay the integrator to prevent
           opposing integrators balancing between the two controllers
		 */
		pitchController.decay_I();
		return pitch_out;
	}
#endif
	// if LANDING_FLARE RCx_OPTION switch is set and in FW mode, manual throttle,throttle idle then set pitch to LAND_PITCH_CD if flight option FORCE_FLARE_ATTITUDE is set
#if HAL_QUADPLANE_ENABLED
	const bool quadplane_in_transition = quadplane.in_transition();
#else
	const bool quadplane_in_transition = false;
#endif

	int32_t demanded_pitch = nav_pitch_cd + g.pitch_trim_cd + SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) * g.kff_throttle_to_pitch;
	bool disable_integrator = false;
	if (control_mode == &mode_stabilize && channel_pitch->get_control_in() != 0) {
		disable_integrator = true;
	}
	/* force landing pitch if:
       - flare switch high
       - throttle stick at zero thrust
       - in fixed wing non auto-throttle mode
	 */
	if (!quadplane_in_transition &&
			!control_mode->is_vtol_mode() &&
			!control_mode->does_auto_throttle() &&
			flare_mode == FlareMode::ENABLED_PITCH_TARGET &&
			throttle_at_zero()) {
		demanded_pitch = landing.get_pitch_cd();
	}

	return pitchController.get_servo_out(demanded_pitch - ahrs.pitch_sensor, speed_scaler, disable_integrator,
			ground_mode && !(plane.g2.flight_options & FlightOptions::DISABLE_GROUND_PID_SUPPRESSION));
}

/*
  this gives the user control of the aircraft in stabilization modes
 */
void Plane::stabilize_stick_mixing_direct()
{
	if (!stick_mixing_enabled() ||
			control_mode == &mode_acro ||
			control_mode == &mode_fbwa ||
			control_mode == &mode_autotune ||
			control_mode == &mode_fbwb ||
			control_mode == &mode_cruise ||
#if HAL_QUADPLANE_ENABLED
			control_mode == &mode_qstabilize ||
			control_mode == &mode_qhover ||
			control_mode == &mode_qloiter ||
			control_mode == &mode_qland ||
			control_mode == &mode_qrtl ||
			control_mode == &mode_qacro ||
#if QAUTOTUNE_ENABLED
			control_mode == &mode_qautotune ||
#endif
#endif
			control_mode == &mode_training) {
		return;
	}
	float aileron = SRV_Channels::get_output_scaled(SRV_Channel::k_aileron);
	aileron = channel_roll->stick_mixing(aileron);
	SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, aileron);

	float elevator = SRV_Channels::get_output_scaled(SRV_Channel::k_elevator);
	elevator = channel_pitch->stick_mixing(elevator);
	SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, elevator);
}

/*
  this gives the user control of the aircraft in stabilization modes
  using FBW style controls
 */
void Plane::stabilize_stick_mixing_fbw()
{
	if (!stick_mixing_enabled() ||
			control_mode == &mode_acro ||
			control_mode == &mode_fbwa ||
			control_mode == &mode_autotune ||
			control_mode == &mode_fbwb ||
			control_mode == &mode_cruise ||
#if HAL_QUADPLANE_ENABLED
			control_mode == &mode_qstabilize ||
			control_mode == &mode_qhover ||
			control_mode == &mode_qloiter ||
			control_mode == &mode_qland ||
			control_mode == &mode_qacro ||
#if QAUTOTUNE_ENABLED
			control_mode == &mode_qautotune ||
#endif
#endif  // HAL_QUADPLANE_ENABLED
			control_mode == &mode_training) {
		return;
	}
	// do FBW style stick mixing. We don't treat it linearly
	// however. For inputs up to half the maximum, we use linear
	// addition to the nav_roll and nav_pitch. Above that it goes
	// non-linear and ends up as 2x the maximum, to ensure that
	// the user can direct the plane in any direction with stick
	// mixing.
	float roll_input = channel_roll->norm_input();
	if (roll_input > 0.5f) {
		roll_input = (3*roll_input - 1);
	} else if (roll_input < -0.5f) {
		roll_input = (3*roll_input + 1);
	}
	nav_roll_cd += roll_input * roll_limit_cd;
	nav_roll_cd = constrain_int32(nav_roll_cd, -roll_limit_cd, roll_limit_cd);

	float pitch_input = channel_pitch->norm_input();
	if (pitch_input > 0.5f) {
		pitch_input = (3*pitch_input - 1);
	} else if (pitch_input < -0.5f) {
		pitch_input = (3*pitch_input + 1);
	}
	if (fly_inverted()) {
		pitch_input = -pitch_input;
	}
	if (pitch_input > 0) {
		nav_pitch_cd += pitch_input * aparm.pitch_limit_max_cd;
	} else {
		nav_pitch_cd += -(pitch_input * pitch_limit_min_cd);
	}
	nav_pitch_cd = constrain_int32(nav_pitch_cd, pitch_limit_min_cd, aparm.pitch_limit_max_cd.get());
}


/*
  stabilize the yaw axis. There are 3 modes of operation:

    - hold a specific heading with ground steering
    - rate controlled with ground steering
    - yaw control for coordinated flight    
 */
void Plane::stabilize_yaw(float speed_scaler)
{
	if (landing.is_flaring()) {
		// in flaring then enable ground steering
		steering_control.ground_steering = true;
	} else {
		// otherwise use ground steering when no input control and we
		// are below the GROUND_STEER_ALT
		steering_control.ground_steering = (channel_roll->get_control_in() == 0 &&
				fabsf(relative_altitude) < g.ground_steer_alt);
		if (!landing.is_ground_steering_allowed()) {
			// don't use ground steering on landing approach
			steering_control.ground_steering = false;
		}
	}


	/*
      first calculate steering_control.steering for a nose or tail
      wheel. We use "course hold" mode for the rudder when either performing
      a flare (when the wings are held level) or when in course hold in
      FBWA mode (when we are below GROUND_STEER_ALT)
	 */
	if (landing.is_flaring() ||
			(steer_state.hold_course_cd != -1 && steering_control.ground_steering)) {
		calc_nav_yaw_course();
	} else if (steering_control.ground_steering) {
		calc_nav_yaw_ground();
	}

	/*
      now calculate steering_control.rudder for the rudder
	 */
	calc_nav_yaw_coordinated(speed_scaler);
}


/*
  a special stabilization function for training mode
 */
void Plane::stabilize_training(float speed_scaler)
{
	const float rexpo = roll_in_expo(false);
	const float pexpo = pitch_in_expo(false);
	if (training_manual_roll) {
		SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, rexpo);
	} else {
		// calculate what is needed to hold
		stabilize_roll(speed_scaler);
		if ((nav_roll_cd > 0 && rexpo < SRV_Channels::get_output_scaled(SRV_Channel::k_aileron)) ||
				(nav_roll_cd < 0 && rexpo > SRV_Channels::get_output_scaled(SRV_Channel::k_aileron))) {
			// allow user to get out of the roll
			SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, rexpo);
		}
	}

	if (training_manual_pitch) {
		SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, pexpo);
	} else {
		stabilize_pitch(speed_scaler);
		if ((nav_pitch_cd > 0 && pexpo < SRV_Channels::get_output_scaled(SRV_Channel::k_elevator)) ||
				(nav_pitch_cd < 0 && pexpo > SRV_Channels::get_output_scaled(SRV_Channel::k_elevator))) {
			// allow user to get back to level
			SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, pexpo);
		}
	}

	stabilize_yaw(speed_scaler);
}


/*
  this is the ACRO mode stabilization function. It does rate
  stabilization on roll and pitch axes
 */
void Plane::stabilize_acro(float speed_scaler)
{
	const float rexpo = roll_in_expo(true);
	const float pexpo = pitch_in_expo(true);
	float roll_rate = (rexpo/SERVO_MAX) * g.acro_roll_rate;
	float pitch_rate = (pexpo/SERVO_MAX) * g.acro_pitch_rate;

	/*
      check for special roll handling near the pitch poles
	 */
	if (g.acro_locking && is_zero(roll_rate)) {
		/*
          we have no roll stick input, so we will enter "roll locked"
          mode, and hold the roll we had when the stick was released
		 */
		if (!acro_state.locked_roll) {
			acro_state.locked_roll = true;
			acro_state.locked_roll_err = 0;
		} else {
			acro_state.locked_roll_err += ahrs.get_gyro().x * G_Dt;
		}
		int32_t roll_error_cd = -ToDeg(acro_state.locked_roll_err)*100;
		nav_roll_cd = ahrs.roll_sensor + roll_error_cd;
		// try to reduce the integrated angular error to zero. We set
		// 'stabilze' to true, which disables the roll integrator
		SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, rollController.get_servo_out(roll_error_cd,
				speed_scaler,
				true, false));
	} else {
		/*
          aileron stick is non-zero, use pure rate control until the
          user releases the stick
		 */
		acro_state.locked_roll = false;
		SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, rollController.get_rate_out(roll_rate,  speed_scaler));
	}

	if (g.acro_locking && is_zero(pitch_rate)) {
		/*
          user has zero pitch stick input, so we lock pitch at the
          point they release the stick
		 */
		if (!acro_state.locked_pitch) {
			acro_state.locked_pitch = true;
			acro_state.locked_pitch_cd = ahrs.pitch_sensor;
		}
		// try to hold the locked pitch. Note that we have the pitch
		// integrator enabled, which helps with inverted flight
		nav_pitch_cd = acro_state.locked_pitch_cd;
		SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, pitchController.get_servo_out(nav_pitch_cd - ahrs.pitch_sensor,
				speed_scaler,
				false, false));
	} else {
		/*
          user has non-zero pitch input, use a pure rate controller
		 */
		acro_state.locked_pitch = false;
		SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, pitchController.get_rate_out(pitch_rate, speed_scaler));
	}

	steering_control.steering = rudder_input();

	if (g.acro_yaw_rate > 0 && yawController.rate_control_enabled()) {
		// user has asked for yaw rate control with yaw rate scaled by ACRO_YAW_RATE
		const float rudd_expo = rudder_in_expo(true);
		const float yaw_rate = (rudd_expo/SERVO_MAX) * g.acro_yaw_rate;
		steering_control.steering = steering_control.rudder = yawController.get_rate_out(yaw_rate,  speed_scaler, false);
	} else if (plane.g2.flight_options & FlightOptions::ACRO_YAW_DAMPER) {
		// use yaw controller
		calc_nav_yaw_coordinated(speed_scaler);
	} else {
		/*
          manual rudder
		 */
		steering_control.rudder = steering_control.steering;
	}
}

/*
  main stabilization function for all 3 axes
 */
void Plane::stabilize()
{
	if (control_mode == &mode_manual) {
		// reset steering controls
		steer_state.locked_course = false;
		steer_state.locked_course_err = 0;
		return;
	}
	float speed_scaler = get_speed_scaler();

	uint32_t now = AP_HAL::millis();
	bool allow_stick_mixing = true;
#if HAL_QUADPLANE_ENABLED
	if (quadplane.available()) {
		quadplane.transition->set_FW_roll_pitch(nav_pitch_cd, nav_roll_cd, allow_stick_mixing);
	}
#endif

	if (now - last_stabilize_ms > 2000) {
		// if we haven't run the rate controllers for 2 seconds then
		// reset the integrators
		rollController.reset_I();
		pitchController.reset_I();
		yawController.reset_I();

		// and reset steering controls
		steer_state.locked_course = false;
		steer_state.locked_course_err = 0;
	}
	last_stabilize_ms = now;

	if (control_mode == &mode_training) {
		stabilize_training(speed_scaler);
#if AP_SCRIPTING_ENABLED
	} else if ((control_mode == &mode_auto &&
			mission.get_current_nav_cmd().id == MAV_CMD_NAV_SCRIPT_TIME) || (nav_scripting.enabled && nav_scripting.current_ms > 0)) {
		// scripting is in control of roll and pitch rates and throttle
		const float aileron = rollController.get_rate_out(nav_scripting.roll_rate_dps, speed_scaler);
		const float elevator = pitchController.get_rate_out(nav_scripting.pitch_rate_dps, speed_scaler);
		SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, aileron);
		SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, elevator);
		if (yawController.rate_control_enabled()) {
			const float rudder = yawController.get_rate_out(nav_scripting.yaw_rate_dps, speed_scaler, false);
			steering_control.rudder = rudder;
		}
		if (AP_HAL::millis() - nav_scripting.current_ms > 50) { //set_target_throttle_rate_rpy has not been called from script in last 50ms
			nav_scripting.current_ms = 0;
		}
#endif
	} else if (control_mode == &mode_acro) {
		stabilize_acro(speed_scaler);
#if HAL_QUADPLANE_ENABLED
	} else if (control_mode->is_vtol_mode() && !quadplane.tailsitter.in_vtol_transition(now)) {
		// run controlers specific to this mode
		plane.control_mode->run();

		// we also stabilize using fixed wing surfaces
		if (plane.control_mode->mode_number() == Mode::Number::QACRO) {
			plane.stabilize_acro(speed_scaler);
		} else {
			plane.stabilize_roll(speed_scaler);
			plane.stabilize_pitch(speed_scaler);
		}
#endif
	} else {
		if (allow_stick_mixing && g.stick_mixing == StickMixing::FBW && control_mode != &mode_stabilize) {
			stabilize_stick_mixing_fbw();
		}
		stabilize_roll(speed_scaler);
		stabilize_pitch(speed_scaler);
		if (allow_stick_mixing && (g.stick_mixing == StickMixing::DIRECT || control_mode == &mode_stabilize)) {
			stabilize_stick_mixing_direct();
		}
		stabilize_yaw(speed_scaler);
	}

	/*
      see if we should zero the attitude controller integrators. 
	 */
	if (is_zero(get_throttle_input()) &&
			fabsf(relative_altitude) < 5.0f &&
			fabsf(barometer.get_climb_rate()) < 0.5f &&
			ahrs.groundspeed() < 3) {
		// we are low, with no climb rate, and zero throttle, and very
		// low ground speed. Zero the attitude controller
		// integrators. This prevents integrator buildup pre-takeoff.
		rollController.reset_I();
		pitchController.reset_I();
		yawController.reset_I();

		// if moving very slowly also zero the steering integrator
		if (ahrs.groundspeed() < 1) {
			steerController.reset_I();
		}
	}
}


void Plane::calc_throttle()
{
	//commentouted by hatae, 2022.3
	/*
    if (aparm.throttle_cruise <= 1) {
        // user has asked for zero throttle - this may be done by a
        // mission which wants to turn off the engine for a parachute
        // landing
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0.0);
        return;
    }

    float commanded_throttle = TECS_controller.get_throttle_demand();

    // Received an external msg that guides throttle in the last 3 seconds?
    if (control_mode->is_guided_mode() &&
            plane.guided_state.last_forced_throttle_ms > 0 &&
            millis() - plane.guided_state.last_forced_throttle_ms < 3000) {
        commanded_throttle = plane.guided_state.forced_throttle;
    }
	 */
	commanded_throttle =TLAB_Throttle_Controller(); //added by hatae, 2022.3

	SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, commanded_throttle);
}

/*****************************************
 * Calculate desired roll/pitch/yaw angles (in medium freq loop)
 *****************************************/

/*
  calculate yaw control for coordinated flight
 */
void Plane::calc_nav_yaw_coordinated(float speed_scaler)
{
	//commentouted by hatae, 2022.3
	/*
    bool disable_integrator = false;
    int16_t rudder_in = rudder_input();

    int16_t commanded_rudder;

    // Received an external msg that guides yaw in the last 3 seconds?
    if (control_mode->is_guided_mode() &&
            plane.guided_state.last_forced_rpy_ms.z > 0 &&
            millis() - plane.guided_state.last_forced_rpy_ms.z < 3000) {
        commanded_rudder = plane.guided_state.forced_rpy_cd.z;
    } else if (control_mode == &mode_autotune && g.acro_yaw_rate > 0 && yawController.rate_control_enabled()) {
        // user is doing an AUTOTUNE with yaw rate control
        const float rudd_expo = rudder_in_expo(true);
        const float yaw_rate = (rudd_expo/SERVO_MAX) * g.acro_yaw_rate;
        commanded_rudder = yawController.get_rate_out(yaw_rate,  speed_scaler, false);
    } else {
        if (control_mode == &mode_stabilize && rudder_in != 0) {
            disable_integrator = true;
        }

        commanded_rudder = yawController.get_servo_out(speed_scaler, disable_integrator);

        // add in rudder mixing from roll
        commanded_rudder += SRV_Channels::get_output_scaled(SRV_Channel::k_aileron) * g.kff_rudder_mix;
        commanded_rudder += rudder_in;
    }

    steering_control.rudder = constrain_int16(commanded_rudder, -4500, 4500);
	 */
	TLAB_CMD_index = mission.get_current_nav_index(); // コマンドインデックスの更新： 主にWP通過判定に用いる

	// Commented out by Kaito Yamamoto 2022.04.02.
	/*
	int16_t commanded_rudder=TLAB_Line_Trace_Controller();//added by hatae, 2022.3
	steering_control.rudder=constrain_int16(commanded_rudder, -4500, 4500);
	*/

	// Added by Kaito Yamamoto 2022.04.02.
	if (g.TPARAM_Bar_Control_Mode == 1) {
	   	// 過去の直線追従コントローラ  "g.TPARAM_switch_mo" でコントローラ切り替え
	   	steering_control.rudder = constrain_int16(TLAB_Line_Trace_Controller(), -4500, 4500);
	} else if (g.TPARAM_Bar_Control_Mode == 2) {
		// 一定出力モード： MPで指定する "g.TPARAM_s_neutral" [deg]の一定値をサーボモーター角度として出力する
	    steering_control.rudder = constrain_int16(TLAB_Constant_Output(), -4500, 4500);
	} else {
		// Added by Kaito Yamamoto 2022.04.02.
	    // 多様な2次元経路追従コントローラ
	    steering_control.rudder = constrain_int16(TLAB_2D_Trace_Controller(), -4500, 4500);
	}
}

/*
  calculate yaw control for ground steering with specific course
 */
void Plane::calc_nav_yaw_course(void)
{
	// holding a specific navigation course on the ground. Used in
	// auto-takeoff and landing
	int32_t bearing_error_cd = nav_controller->bearing_error_cd();
	steering_control.steering = steerController.get_steering_out_angle_error(bearing_error_cd);
	if (stick_mixing_enabled()) {
		steering_control.steering = channel_rudder->stick_mixing(steering_control.steering);
	}
	steering_control.steering = constrain_int16(steering_control.steering, -4500, 4500);
}

/*
  calculate yaw control for ground steering
 */
void Plane::calc_nav_yaw_ground(void)
{
	if (gps.ground_speed() < 1 &&
			is_zero(get_throttle_input()) &&
			flight_stage != AP_Vehicle::FixedWing::FLIGHT_TAKEOFF &&
			flight_stage != AP_Vehicle::FixedWing::FLIGHT_ABORT_LAND) {
		// manual rudder control while still
		steer_state.locked_course = false;
		steer_state.locked_course_err = 0;
		steering_control.steering = rudder_input();
		return;
	}

	// if we haven't been steering for 1s then clear locked course
	const uint32_t now_ms = AP_HAL::millis();
	if (now_ms - steer_state.last_steer_ms > 1000) {
		steer_state.locked_course = false;
	}
	steer_state.last_steer_ms = now_ms;

	float steer_rate = (rudder_input()/4500.0f) * g.ground_steer_dps;
	if (flight_stage == AP_Vehicle::FixedWing::FLIGHT_TAKEOFF ||
			flight_stage == AP_Vehicle::FixedWing::FLIGHT_ABORT_LAND) {
		steer_rate = 0;
	}
	if (!is_zero(steer_rate)) {
		// pilot is giving rudder input
		steer_state.locked_course = false;
	} else if (!steer_state.locked_course) {
		// pilot has released the rudder stick or we are still - lock the course
		steer_state.locked_course = true;
		if (flight_stage != AP_Vehicle::FixedWing::FLIGHT_TAKEOFF &&
				flight_stage != AP_Vehicle::FixedWing::FLIGHT_ABORT_LAND) {
			steer_state.locked_course_err = 0;
		}
	}

	if (!steer_state.locked_course) {
		// use a rate controller at the pilot specified rate
		steering_control.steering = steerController.get_steering_out_rate(steer_rate);
	} else {
		// use a error controller on the summed error
		int32_t yaw_error_cd = -ToDeg(steer_state.locked_course_err)*100;
		steering_control.steering = steerController.get_steering_out_angle_error(yaw_error_cd);
	}
	steering_control.steering = constrain_int16(steering_control.steering, -4500, 4500);
}


/*
  calculate a new nav_pitch_cd from the speed height controller
 */
void Plane::calc_nav_pitch()
{
	// Calculate the Pitch of the plane
	// --------------------------------
	int32_t commanded_pitch = TECS_controller.get_pitch_demand();

	// Received an external msg that guides roll in the last 3 seconds?
	if (control_mode->is_guided_mode() &&
			plane.guided_state.last_forced_rpy_ms.y > 0 &&
			millis() - plane.guided_state.last_forced_rpy_ms.y < 3000) {
		commanded_pitch = plane.guided_state.forced_rpy_cd.y;
	}

	nav_pitch_cd = constrain_int32(commanded_pitch, pitch_limit_min_cd, aparm.pitch_limit_max_cd.get());
}


/*
  calculate a new nav_roll_cd from the navigation controller
 */
void Plane::calc_nav_roll()
{
	int32_t commanded_roll = nav_controller->nav_roll_cd();

	// Received an external msg that guides roll in the last 3 seconds?
	if (control_mode->is_guided_mode() &&
			plane.guided_state.last_forced_rpy_ms.x > 0 &&
			millis() - plane.guided_state.last_forced_rpy_ms.x < 3000) {
		commanded_roll = plane.guided_state.forced_rpy_cd.x;
#if OFFBOARD_GUIDED == ENABLED
		// guided_state.target_heading is radians at this point between -pi and pi ( defaults to -4 )
	} else if ((control_mode == &mode_guided) && (guided_state.target_heading_type != GUIDED_HEADING_NONE) ) {
		uint32_t tnow = AP_HAL::millis();
		float delta = (tnow - guided_state.target_heading_time_ms) * 1e-3f;
		guided_state.target_heading_time_ms = tnow;

		float error = 0.0f;
		if (guided_state.target_heading_type == GUIDED_HEADING_HEADING) {
			error = wrap_PI(guided_state.target_heading - AP::ahrs().yaw);
		} else {
			Vector2f groundspeed = AP::ahrs().groundspeed_vector();
			error = wrap_PI(guided_state.target_heading - atan2f(-groundspeed.y, -groundspeed.x) + M_PI);
		}

		float bank_limit = degrees(atanf(guided_state.target_heading_accel_limit/GRAVITY_MSS)) * 1e2f;

		g2.guidedHeading.update_error(error); // push error into AC_PID , possible improvement is to use update_all instead.?
		g2.guidedHeading.set_dt(delta);

		float i = g2.guidedHeading.get_i(); // get integrator TODO
		if (((is_negative(error) && !guided_state.target_heading_limit_low) || (is_positive(error) && !guided_state.target_heading_limit_high))) {
			i = g2.guidedHeading.get_i();
		}

		float desired = g2.guidedHeading.get_p() + i + g2.guidedHeading.get_d();
		guided_state.target_heading_limit_low = (desired <= -bank_limit);
		guided_state.target_heading_limit_high = (desired >= bank_limit);
		commanded_roll = constrain_float(desired, -bank_limit, bank_limit);
#endif // OFFBOARD_GUIDED == ENABLED
	}

	nav_roll_cd = constrain_int32(commanded_roll, -roll_limit_cd, roll_limit_cd);
	update_load_factor();
}

/*
  adjust nav_pitch_cd for STAB_PITCH_DOWN_CD. This is used to make
  keeping up good airspeed in FBWA mode easier, as the plane will
  automatically pitch down a little when at low throttle. It makes
  FBWA landings without stalling much easier.
 */
void Plane::adjust_nav_pitch_throttle(void)
{
	int8_t throttle = throttle_percentage();
	if (throttle >= 0 && throttle < aparm.throttle_cruise && flight_stage != AP_Vehicle::FixedWing::FLIGHT_VTOL) {
		float p = (aparm.throttle_cruise - throttle) / (float)aparm.throttle_cruise;
		nav_pitch_cd -= g.stab_pitch_down * 100.0f * p;
	}
}


/*
  calculate a new aerodynamic_load_factor and limit nav_roll_cd to
  ensure that the load factor does not take us below the sustainable
  airspeed
 */
void Plane::update_load_factor(void)
{
	float demanded_roll = fabsf(nav_roll_cd*0.01f);
	if (demanded_roll > 85) {
		// limit to 85 degrees to prevent numerical errors
		demanded_roll = 85;
	}
	aerodynamic_load_factor = 1.0f / safe_sqrt(cosf(radians(demanded_roll)));

#if HAL_QUADPLANE_ENABLED
	if (quadplane.available() && quadplane.transition->set_FW_roll_limit(roll_limit_cd)) {
		nav_roll_cd = constrain_int32(nav_roll_cd, -roll_limit_cd, roll_limit_cd);
		return;
	}
#endif

	if (!aparm.stall_prevention) {
		// stall prevention is disabled
		return;
	}
	if (fly_inverted()) {
		// no roll limits when inverted
		return;
	}
#if HAL_QUADPLANE_ENABLED
	if (quadplane.tailsitter.active()) {
		// no limits while hovering
		return;
	}
#endif

	float max_load_factor = smoothed_airspeed / MAX(aparm.airspeed_min, 1);
	if (max_load_factor <= 1) {
		// our airspeed is below the minimum airspeed. Limit roll to
		// 25 degrees
		nav_roll_cd = constrain_int32(nav_roll_cd, -2500, 2500);
		roll_limit_cd = MIN(roll_limit_cd, 2500);
	} else if (max_load_factor < aerodynamic_load_factor) {
		// the demanded nav_roll would take us past the aerodymamic
		// load limit. Limit our roll to a bank angle that will keep
		// the load within what the airframe can handle. We always
		// allow at least 25 degrees of roll however, to ensure the
		// aircraft can be maneuvered with a bad airspeed estimate. At
		// 25 degrees the load factor is 1.1 (10%)
		int32_t roll_limit = degrees(acosf(sq(1.0f / max_load_factor)))*100;
		if (roll_limit < 2500) {
			roll_limit = 2500;
		}
		nav_roll_cd = constrain_int32(nav_roll_cd, -roll_limit, roll_limit);
		roll_limit_cd = MIN(roll_limit_cd, roll_limit);
	}
}


//TanakaLab
//added by hatae, 2022.3
int32_t Plane::TLAB_Throttle_Controller(void){
	//取り出し
	int32_t z_cm=current_loc.alt;
	int32_t z_r_cm=next_WP_loc.alt;
	z=z_cm*0.01f;
	z_r=z_r_cm*0.01f;
	current_time_Th=AP_HAL::micros64();

	//状態変数
	e_m=z-z_r;
	de_m=-gps.velocity().z;
	pitch_r=ahrs.pitch-g.TPARAM_Pitche/180.0f*M_PI;
	dpitch=wrap_PI(ahrs.get_gyro().y);


	//推力の推定計算(1次遅れ)と誤差の積分(PID)を行う
	if (firsttime_Th){
		firsttime_Th=false;
		err_count = 0;
		prev_time_Th=current_time_Th;
		hat_T=g.TPARAM_Te;
		ie_m=0;
		e_m_old=e_m;
		hat_T_old=g.TPARAM_Te;
		motor_Th_N=0;
		motor_Th_N_old=0;
	}
	past_time_Th=current_time_Th-prev_time_Th;
	float past_time_Th_f=(float)past_time_Th;
	past_time_Th_f*=1e-6;
	if (past_time_Th==0){
		err_count++;
		prev_time_Th=current_time_Th;
	} else {
		hat_T=(1.0f-past_time_Th_f/g.TPARAM_tau)*hat_T_old+past_time_Th_f/g.TPARAM_tau*motor_Th_N_old;
		ie_m+=1.0f/2.0f*past_time_Th_f*(e_m_old+e_m);
		prev_time_Th=current_time_Th;
		hat_T_old=hat_T;
	}

	//制御器本体
	if (g.TPARAM_cha_th==1){
		motor_Th_N=TLAB_Hatae_Controller(e_m, de_m, pitch_r, dpitch, (hat_T-g.TPARAM_Te));
	} else if (g.TPARAM_cha_th==2){
		motor_Th_N=TLAB_LQR_Controller_alt(e_m, de_m, pitch_r, dpitch, (hat_T-g.TPARAM_Te));
	} else {
		motor_Th_N=TLAB_PID_Controller_alt(e_m, ie_m, de_m);
	}
	motor_Th_N+=g.TPARAM_Te;
	motor_Th_N_old=motor_Th_N;

	//値を返す
	motor_per = TLAB_thrust_to_percent(motor_Th_N);

	//スロットルのデバッグ
	if(g.TPARAM_debug_th!=0){
		motor_per=g.TPARAM_debug_per;
	}

	return motor_per;
}

//added by hatae, 2022.3
int32_t Plane::TLAB_Line_Trace_Controller(void){
	TLAB_Control_flag = g.TPARAM_control_mode;
	const_k = g.TPARAM_k;
	v_a_TL = g.TPARAM_Va;
	Vg_min = g.TPARAM_Vg_min;
	Vg_max = g.TPARAM_Vg_max;
	alpha_min = g.TPARAM_alpha_min*M_PI/180.0f;
	alpha_max = g.TPARAM_alpha_max*M_PI/180.0f;
	U_min = g.TPARAM_U_min;
	U_max = g.TPARAM_U_max;
	currenttime_dir=AP_HAL::micros64();

	Dist_currWP2UAV=Plane::TLAB_get_distance(current_loc, prev_WP_loc);
	rad_prevWP2UAV=(float)(Plane::TLAB_get_bearing_cd(prev_WP_loc, current_loc)*0.01*M_PI/180.0);
	rad_prevWP2currWP = (float)(Plane::TLAB_get_bearing_cd(prev_WP_loc, next_WP_loc)*0.01*M_PI/180.0);
	float rad_WPline2UAV = rad_prevWP2UAV - rad_prevWP2currWP;
	state_UAV_x = Dist_currWP2UAV*cosf(rad_WPline2UAV);
	state_UAV_y = Dist_currWP2UAV*sinf(rad_WPline2UAV);
	state_UAV_phi = wrap_PI(ahrs.yaw_sensor*0.01*M_PI/180.0 - rad_prevWP2currWP);
	state_UAV_GCRS = wrap_PI(gps.ground_course_cd()*0.01*M_PI/180.0 - rad_prevWP2currWP);

	v_g_TL = gps.ground_speed();
	if(v_g_TL > Vg_max){
		Vg_limited = Vg_max;
	}else if(v_g_TL < Vg_min){
		Vg_limited = Vg_min;
	}else{
		Vg_limited = v_g_TL;
	}

	alpha=wrap_PI(state_UAV_phi-state_UAV_GCRS);
	if(alpha > alpha_max){
		alpha = alpha_max;
	}else if(alpha < alpha_min){
		alpha = alpha_min;
	}
	L_conv = Vg_limited/(v_a_TL*cosf(alpha));

	//ydotの計算を行う
	if (firsttime_dir){
		firsttime_dir=false;
		err_count_dir = 0;
		prev_time_dir=currenttime_dir;
		UAV_dy=0.0f;
		UAV_y_old=state_UAV_y;
	}
	past_time_dir=currenttime_dir-prev_time_dir;
	float pasttime_dir_f=(float)past_time_dir;
	pasttime_dir_f*=1e-6;
	if (past_time_dir==0){
		err_count_dir++;
		UAV_y_old=state_UAV_y;
		UAV_dy=0.0f;
		prev_time_dir=currenttime_dir;
	} else {
		UAV_dy=(state_UAV_y-UAV_y_old)/pasttime_dir_f;
		UAV_y_old=state_UAV_y;
		prev_time_dir=currenttime_dir;
	}

	if (g.TPARAM_debug_s==1){
		u = g.TPARAM_bar_neutral*M_PI/180.0;
		servo = static_cast<int32_t>(asinf(constrain_float(58.0f / 29.0f * sinf(u), -1.0f, 1.0f))*100.0f*180.0f/M_PI);
	} else if (g.TPARAM_cha_dir==1){
		u = -(g.TPARAM_F_1b*state_UAV_y+g.TPARAM_F_2b*UAV_dy)+g.TPARAM_bar_neutral*M_PI/180.0;
		servo = static_cast<int32_t>(asinf(constrain_float(58.0f / 29.0f * sinf(u), -1.0f, 1.0f))*100.0f*180.0f/M_PI);
	} else {
		u = constrain_float(L_conv/const_k*u_star,U_min*M_PI/180.0,U_max*M_PI/180.0) + g.TPARAM_bar_neutral*M_PI/180.0;
		servo = static_cast<int32_t>(asinf(constrain_float(58.0f / 29.0f * sinf(u), -1.0f, 1.0f))*100.0f*180.0f/M_PI);
	}

	return servo;
}

//added by hatae, 2022.3
float Plane::TLAB_Hatae_Controller(float x_1, float x_2, float x_3, float x_4, float x_5){
	//波多江の設計した縦のLMI制御器(Hはかぶりを防ぐため)2022_3_8ver

	//パラメータ
	float Hm_1=1.70f;
	float Hm_2=1.197f;
	float HC_L=0.485f;
	float HC_D=0.125f;
	float Hrho=1.293f;
	float HS=0.9424f;
	float Hg=9.80665f;
	float Hl_1=0.123f;
	float Hl_2=0.8358f;
	float HI_y=0.137f;
	float Hgamma=5.0f/180.0f*M_PI;
	float Hv_nx=g.TPARAM_V_nx;
	float Htheta_e=g.TPARAM_Pitche/180.0f*M_PI;

	//非線形要素
	float Hphi_d=atanf((x_2-Hl_2*x_4*sinf(x_3+Htheta_e+Hgamma))/(Hv_nx-Hl_2*x_4*cosf(x_3+Htheta_e+Hgamma)));
	float Hz1=1.0f/2.0f*Hrho*HC_L*x_2+cosf(Hphi_d)\
			-Hl_2*Hrho*HC_L*HS*x_4*sinf(x_3+Htheta_e+Hgamma)*cosf(Hphi_d)\
			-1.0f/(2.0f*(Hv_nx-Hl_2*x_4*cosf(x_3+Htheta_e+Hgamma)))*Hrho*HC_D*HS*powf(Hv_nx,2.0f)\
			-1.0f/2.0f*Hrho*HC_D*HS*x_2*sinf(Hphi_d)\
			+Hl_2*Hrho*HC_D*HS*x_4*sinf(x_3+Htheta_e+Hgamma)*sinf(Hphi_d);
	Hz1=Hz1/(Hm_1+Hm_2);
	float Hz2=-Hl_2*Hrho*HC_L*HS*Hv_nx*cosf(x_3+Htheta_e+Hgamma)*cosf(Hphi_d)\
			+1.0f/2.0f*powf(Hl_2,2.0f)*Hrho*HC_L*HS*x_4*cosf(Hphi_d)\
			+1.0f/(2.0f*(Hv_nx-Hl_2*x_4*cosf(x_3+Htheta_e+Hgamma)))*Hl_2*Hrho*HC_D*HS*powf(Hv_nx,2.0f)*sinf(x_3+Htheta_e+Hgamma)\
			+Hl_2*Hrho*HC_D*HS*Hv_nx*cosf(x_3+Htheta_e+Hgamma)*sinf(Hphi_d)\
			-1.0f/2.0f*powf(Hl_2,2.0f)*Hrho*HC_D*HS*x_4*sinf(Hphi_d);
	Hz2=Hz2/(Hm_1+Hm_2);
	float Hz3=sinf(x_3+Htheta_e);
	Hz3=Hz3/(Hm_1+Hm_2);
	float Hz4=-1.0f/2.0f*Hl_2*Hrho*HC_L*HS*x_2*sinf(x_3-Hphi_d+Htheta_e+Hgamma)\
			+powf(Hl_2,2.0f)*Hrho*HC_L*HS*x_4*sinf(x_3-Hphi_d+Htheta_e+Hgamma)*sinf(x_3-Hphi_d+Htheta_e+Hgamma)\
			+1.0f/2.0f*Hl_2*Hrho*HC_D*HS*x_2*cosf(x_3+Htheta_e+Hgamma)\
			-powf(Hl_2,2.0f)*Hrho*HC_D*HS*x_4*sinf(x_3+Htheta_e+Hgamma)*cosf(x_3+Htheta_e+Hgamma)\
			+1.0f/(2.0f*(Hv_nx-Hl_2*x_4*cosf(x_3+Htheta_e+Hgamma)))*Hl_2*Hrho*HC_L*HS*powf(Hv_nx,2.0f)*Plane::TLAB_sincf(x_3-Hphi_d)\
			+1.0f/(4.0f*(Hv_nx-Hl_2*x_4*cosf(x_3+Htheta_e+Hgamma)))*Hl_2*Hrho*HC_L*HS*powf(Hv_nx,2.0f)*Hphi_d*sinf(Htheta_e+Hgamma)\
			-1.0f/(4.0f*(Hv_nx-Hl_2*x_4*cosf(x_3+Htheta_e+Hgamma)))*Hl_2*Hrho*HC_D*HS*powf(Hv_nx,2.0f)*Hphi_d*cosf(Htheta_e+Hgamma)\
			+1.0f/(2.0f*(Hv_nx-Hl_2*x_4*cosf(x_3+Htheta_e+Hgamma)))*Hl_2*Hrho*HC_D*HS*powf(Hv_nx,2.0f)*Plane::TLAB_sincf(x_3-Hphi_d);
	Hz4=Hz4/HI_y;
	float Hz5=-1.0f/2.0f*Hl_2*Hrho*HC_L*HS*powf(Hv_nx,2.0f)*Plane::TLAB_sincf(x_3-Hphi_d)\
			+1.0f/4.0f*Hl_2*Hrho*HC_L*HS*powf(Hv_nx,2.0f)*(x_3-2*Hphi_d)*sinf(Htheta_e+Hgamma)\
			+1.0f/4.0f*Hl_2*Hrho*HC_D*HS*powf(Hv_nx,2.0f)*(x_3-2*Hphi_d)*cosf(Htheta_e+Hgamma)\
			-1.0f/2.0f*Hl_2*Hrho*HC_D*HS*powf(Hv_nx,2.0f)*Plane::TLAB_sincf(x_3-Hphi_d)\
			-Hm_2*Hg*Hl_1*Plane::TLAB_sincf(x_3)*cosf(Htheta_e)\
			+Hm_1*Hg*Hl_2*Plane::TLAB_sincf(x_3)*cosf(Htheta_e);
	Hz5=Hz5/HI_y;
	float Hz6=powf(Hl_2,2.0f)*Hrho*HC_L*HS*Hv_nx*cosf(x_3+Htheta_e+Hgamma)*sinf(x_3-Hphi_d+Htheta_e+Hgamma)\
			-1.0f/2.0f*powf(Hl_2,3.0f)*Hrho*HC_L*HS*x_4*sinf(x_3-Hphi_d+Htheta_e+Hgamma)\
			-powf(Hl_2,2.0f)*Hrho*HC_D*HS*Hv_nx*cosf(x_3+Htheta_e+Hgamma)*cosf(x_3-Hphi_d+Htheta_e+Hgamma)\
			+1.0f/2.0f*powf(Hl_2,3.0f)*Hrho*HC_D*HS*x_4*cosf(x_3-Hphi_d+Htheta_e+Hgamma)\
			-(Hl_2*sinf(x_3+Htheta_e+Hgamma))/(2.0f*(Hv_nx-Hl_2*x_4*cosf(x_3+Htheta_e+Hgamma)))*Hl_2*Hrho*HC_L*HS*powf(Hv_nx,2.0f)*Plane::TLAB_sincf(x_3-Hphi_d)\
			-(Hl_2*sinf(x_3+Htheta_e+Hgamma))/(4.0f*(Hv_nx-Hl_2*x_4*cosf(x_3+Htheta_e+Hgamma)))*Hl_2*Hrho*HC_L*HS*powf(Hv_nx,2.0f)*Hphi_d*sinf(Htheta_e+Hgamma)\
			+(Hl_2*sinf(x_3+Htheta_e+Hgamma))/(4.0f*(Hv_nx-Hl_2*x_4*cosf(x_3+Htheta_e+Hgamma)))*Hl_2*Hrho*HC_D*HS*powf(Hv_nx,2.0f)*Hphi_d*cosf(Htheta_e+Hgamma)\
			-(Hl_2*sinf(x_3+Htheta_e+Hgamma))/(2.0f*(Hv_nx-Hl_2*x_4*cosf(x_3+Htheta_e+Hgamma)))*Hl_2*Hrho*HC_D*HS*powf(Hv_nx,2.0f)*Plane::TLAB_sincf(x_3-Hphi_d);
	Hz6=Hz6/HI_y;

	//最大値最小値の設定, ゲインの読み込み
	TLAB_Hatae_Controller_gain(g.TPARAM_cha_gain_h);
	TLAB_Hatae_Controller_Maxmin(g.TPARAM_cha_gain_h);

	//メンバシップ関数
	float Hmem_I[2];
	float Hmem_J[2];
	float Hmem_K[2];
	float Hmem_L[2];
	float Hmem_M[2];
	float Hmem_N[2];
	float Hh[64];

	Hmem_I[0]=constrain_float((Hmaxmin_z[0][0]-Hz1)/(Hmaxmin_z[0][0]-Hmaxmin_z[0][1]),0.0f,1.0f);
	Hmem_I[1]=constrain_float((Hz1-Hmaxmin_z[0][1])/(Hmaxmin_z[0][0]-Hmaxmin_z[0][1]),0.0f,1.0f);
	Hmem_J[0]=constrain_float((Hmaxmin_z[1][0]-Hz2)/(Hmaxmin_z[1][0]-Hmaxmin_z[1][1]),0.0f,1.0f);
	Hmem_J[1]=constrain_float((Hz2-Hmaxmin_z[1][1])/(Hmaxmin_z[1][0]-Hmaxmin_z[1][1]),0.0f,1.0f);
	Hmem_K[0]=constrain_float((Hmaxmin_z[2][0]-Hz3)/(Hmaxmin_z[2][0]-Hmaxmin_z[2][1]),0.0f,1.0f);
	Hmem_K[1]=constrain_float((Hz3-Hmaxmin_z[2][1])/(Hmaxmin_z[2][0]-Hmaxmin_z[2][1]),0.0f,1.0f);
	Hmem_L[0]=constrain_float((Hmaxmin_z[3][0]-Hz4)/(Hmaxmin_z[3][0]-Hmaxmin_z[3][1]),0.0f,1.0f);
	Hmem_L[1]=constrain_float((Hz4-Hmaxmin_z[3][1])/(Hmaxmin_z[3][0]-Hmaxmin_z[3][1]),0.0f,1.0f);
	Hmem_M[0]=constrain_float((Hmaxmin_z[4][0]-Hz5)/(Hmaxmin_z[4][0]-Hmaxmin_z[4][1]),0.0f,1.0f);
	Hmem_M[1]=constrain_float((Hz5-Hmaxmin_z[4][1])/(Hmaxmin_z[4][0]-Hmaxmin_z[4][1]),0.0f,1.0f);
	Hmem_N[0]=constrain_float((Hmaxmin_z[5][0]-Hz6)/(Hmaxmin_z[5][0]-Hmaxmin_z[5][1]),0.0f,1.0f);
	Hmem_N[1]=constrain_float((Hz6-Hmaxmin_z[5][1])/(Hmaxmin_z[5][0]-Hmaxmin_z[5][1]),0.0f,1.0f);

	int Hnum_loop=0;
	for (int Hi=0;Hi<2;Hi++){
		for (int Hj=0;Hj<2;Hj++){
			for (int Hk=0;Hk<2;Hk++){
				for (int Hl=0;Hl<2;Hl++){
					for (int Hm=0;Hm<2;Hm++){
						for (int Hn=0;Hn<2;Hn++){
							Hh[Hnum_loop]=Hmem_I[Hi]*Hmem_J[Hj]*Hmem_K[Hk]*Hmem_L[Hl]*Hmem_M[Hm]*Hmem_N[Hn];
							Hnum_loop++;
						}
					}
				}
			}
		}
	}

	//推力の計算
	float x_r[5]={x_1,x_2,x_3,x_4,x_5};
	int Motor_N=0.0f;
	float Motor_N_i;
	for (int i=0;i<64;i++){
		Motor_N_i=0;
		for (int j=0;j<5;j++){
			Motor_N_i+=Hf_LMI[i][j]*x_r[j];
		}
		Motor_N-=Hh[i]*Motor_N_i;
	}
	return Motor_N;
}

//added by hatae, 2022.3
float Plane::TLAB_LQR_Controller_alt(float x_1, float x_2, float x_3, float x_4, float x_5){
	float f_LQR[5]={g.TPARAM_LQR_f1,g.TPARAM_LQR_f2,g.TPARAM_LQR_f3,g.TPARAM_LQR_f4,g.TPARAM_LQR_f5};
	float x_LQR[5]={x_1,x_2,x_3,x_4,x_5};
	float Motor_N=0.0f;
	for (int i=0;i<5;i++){
		Motor_N+=-f_LQR[i]*x_LQR[i];
	}
	return Motor_N;
}

//added by hatae, 2022.3
float Plane::TLAB_PID_Controller_alt(float x_1,float x_2,float x_3){
	float f_PID[3]={g.TPARAM_PID_p,g.TPARAM_PID_i,g.TPARAM_PID_d};
	float x_PID[3]={x_1,x_2,x_3};
	float Motor_N=0.0f;
	for (int i=0;i<3;i++){
		Motor_N+=-f_PID[i]*x_PID[i];
	}
	return Motor_N;
}

//added by hatae, 2022.3
int32_t Plane::TLAB_thrust_to_percent(float thrust){
	float a=0.3983f*0.0013f;
	float b=0.3983f*0.0622f;

	value=static_cast<int32_t>(constrain_float((sqrtf(b*b+4.0f*a*motor_Th_N)-b)/(2.0f*a),0.0f,80.0f));

	return value;
}

//added by hatae, 2022.3
float Plane::TLAB_get_distance(const struct Location &loc1, const struct Location &loc2){
	float dlat=(float)(loc2.lat - loc1.lat);
	float dlong=((float)(loc2.lng - loc1.lng))*TLAB_longitude_scale(loc2);
	return norm(dlat, dlong)*LOCATION_SCALING_FACTOR;
}

//added by hatae, 2022.3
float Plane::TLAB_longitude_scale(const struct Location &loc){
	float scale=cosf(loc.lat*1.0e-7f*DEG_TO_RAD);
	return constrain_float(scale, 0.01f, 1.0f);
}

//added by hatae, 2022.3
float Plane::TLAB_get_bearing_cd(const struct Location &loc1, const struct Location &loc2){
	int32_t off_x = loc2.lng - loc1.lng;
	int32_t off_y = (loc2.lat - loc1.lat) / TLAB_longitude_scale(loc2);
	int32_t bearing = 9000 + atan2f(-off_y, off_x) * 5729.57795f;
	if (bearing < 0) bearing += 36000;
	return bearing;
}

//added by hatae, 2022.3
float Plane::TLAB_sincf(float x_1){
	if (x_1<0.01){
		return 0.0f;
	} else {
		return sinf(x_1)/x_1;
	}
}

//added by hatae, 2022.3
void Plane::TLAB_Hatae_Controller_gain(int x_1){
	if (x_1==1){
		//後で追加
	} else {
		float F_gain[64][5]={
				{1.35,80.63,-340.9,17.53,2.73},
				{1.69,89.56,-341.55,21.78,3.02},
				{1.37,82.46,-393.37,24.61,2.8},
				{1.64,89.06,-392.59,24.09,3},
				{1.39,76.87,-343.57,15.46,2.66},
				{1.77,86.01,-344.4,20.78,2.99},
				{1.5,78.67,-395.09,23.88,2.77},
				{1.75,85.44,-394.3,23.4,2.98},
				{1.68,86.72,-321.83,25.02,1.86},
				{1.8,89.09,-329.96,19.64,1.94},
				{1.18,76.53,-400.05,28.74,1.35},
				{1.62,85.58,-391.17,20.82,1.75},
				{1.7,78.85,-338.8,17.7,1.89},
				{1.86,83.78,-340.26,16.99,2},
				{1.67,76.71,-409.54,26.06,1.81},
				{1.8,82.03,-395.97,19.34,1.92},
				{1.35,80.04,-343.58,14.04,2.67},
				{1.73,90.46,-343.45,20.51,3.03},
				{1.41,82.9,-389.97,21.67,2.79},
				{1.7,90.22,-392.12,23.01,3.02},
				{1.35,76.29,-345.91,11.43,2.59},
				{1.81,87.01,-346.21,19.44,2.99},
				{1.51,79.39,-390.97,20.66,2.76},
				{1.8,86.7,-393.39,22.18,3},
				{1.68,85.77,-327.28,20.8,1.83},
				{1.82,89.36,-331.46,18,1.92},
				{1.36,80.14,-399.06,26.3,1.53},
				{1.67,86.52,-388.85,19.53,1.77},
				{1.59,76.89,-341.57,11.85,1.75},
				{1.87,84.14,-341.38,15.14,1.96},
				{1.68,77.81,-403.16,22.4,1.81},
				{1.83,82.97,-393.08,17.83,1.91},
				{1.43,78.56,-342.3,17.7,2.73},
				{1.76,87.27,-343.08,21.77,3.02},
				{1.45,80.32,-395.13,24.87,2.79},
				{1.71,86.77,-394.09,24.12,3},
				{1.5,75.26,-344.76,16.26,2.68},
				{1.85,83.96,-345.72,20.99,2.99},
				{1.58,76.74,-397.4,24.59,2.77},
				{1.82,83.34,-395.96,23.63,2.98},
				{1.79,84.47,-329.19,26.21,2.04},
				{1.91,86.87,-336.41,20.82,2.11},
				{1.45,77.65,-410.73,30.92,1.69},
				{1.77,84.11,-398.64,22.38,1.96},
				{1.8,76.68,-340.46,20.19,2.03},
				{1.94,81.22,-343.09,18.35,2.13},
				{1.74,73.85,-413.41,27.98,1.91},
				{1.88,79.47,-400.23,20.74,2.05},
				{1.43,78.04,-345.01,14.35,2.68},
				{1.81,88.17,-345,20.55,3.03},
				{1.49,80.79,-391.95,22.05,2.79},
				{1.77,87.89,-393.45,23,3.02},
				{1.46,74.81,-347.19,12.45,2.62},
				{1.89,84.96,-347.57,19.71,3},
				{1.6,77.5,-393.65,21.53,2.77},
				{1.87,84.58,-395.17,22.46,3},
				{1.79,83.6,-333.79,22.29,2.01},
				{1.94,87.29,-338.15,19.34,2.1},
				{1.55,79.5,-407.7,28.21,1.78},
				{1.82,85.04,-396.8,21.16,1.98},
				{1.73,75.59,-343.62,15.48,1.94},
				{1.96,81.81,-344.66,16.76,2.11},
				{1.76,75.03,-408.41,24.78,1.93},
				{1.92,80.52,-398.19,19.42,2.05},
		};
		for (int i=0;i<64;i++){
			Hf_LMI[i][0]=F_gain[i][0];
			Hf_LMI[i][1]=F_gain[i][1];
			Hf_LMI[i][2]=F_gain[i][2];
			Hf_LMI[i][3]=F_gain[i][3];
			Hf_LMI[i][4]=F_gain[i][4];
		}
	}
}

//added by hatae, 2022.3
void Plane::TLAB_Hatae_Controller_Maxmin(int x_1){
	if (x_1==1){

	} else{
		float minmax[6][2]={
				{-0.61,-0.14},
				{-2.4,-1.93},
				{0.03,0.23},
				{9.77,12.39},
				{-102.48,-75.5},
				{-6.86,1.5}
		};
		for (int i=0;i<6;i++){
			Hmaxmin_z[i][0]=minmax[i][0];
			Hmaxmin_z[i][1]=minmax[i][1];
		}
	}
}


/* ##### Added by Kaito Yamamoto 2022.04.02. #####
 * 概要: 一定のサーボモーター角度を出力する
*/
int32_t Plane::TLAB_Constant_Output(void){
	// 変数の初期設定
	if (!yet_init) {
		servo = 0;
		yet_init = true;
	}
	servo = static_cast<int32_t>(g.TPARAM_bar_neutral*100);  // [cdeg]
	return servo;
}


/* ##### Added by Kaito Yamamoto 2022.04.02. #####
 * 概要: Plane::TLAB_2D_Trace_Controller(void)内の変数の初期化
*/
void Plane::init_TLAB_2D_Trace_Controller(void){
	// UAVに固有な定数
	v_a_TL = g.TPARAM_v_a;  // 対気速度の大きさ [m/s]: const
	k_prop_const = g.TPARAM_k_prop_const;  // コントロールバー角度 [rad]と旋回速度 [rad/s]の比例定数(proportional constant) [1/s]
	// xy座標の原点位置をHP(オートモード開始地点)に設定(int32_t): 緯度,経度 [1e-7*deg]
    Path_Origin.lat = prev_WP_loc.lat;
    Path_Origin.lng = prev_WP_loc.lng;
    // フライトプランの設定
    Flight_Plan = g.TPARAM_Flight_Plan;
	// フィードバックゲイン  Fx[3], Fchi[4][3]
	Fx[0] = g.TPARAM_Fx1;
	Fx[1] = g.TPARAM_Fx2;
	Fx[2] = g.TPARAM_Fx3;
	Fchi[0][0] = g.TPARAM_Fchi1_1;
	Fchi[0][1] = g.TPARAM_Fchi1_2;
	Fchi[0][2] = g.TPARAM_Fchi1_3;
	Fchi[1][0] = g.TPARAM_Fchi2_1;
	Fchi[1][1] = g.TPARAM_Fchi2_2;
	Fchi[1][2] = g.TPARAM_Fchi2_3;
	Fchi[2][0] = g.TPARAM_Fchi3_1;
	Fchi[2][1] = g.TPARAM_Fchi3_2;
	Fchi[2][2] = g.TPARAM_Fchi3_3;
	Fchi[3][0] = g.TPARAM_Fchi4_1;
	Fchi[3][1] = g.TPARAM_Fchi4_2;
	Fchi[3][2] = g.TPARAM_Fchi4_3;
	// ファジィ化範囲(const)
	v_g_min = g.TPARAM_v_g_min;  // default: 3 [m/s]
	v_g_max = g.TPARAM_v_g_max;  // default: 10 [m/s]
	kappa_max = g.TPARAM_kappa_max;
	kappa_min = g.TPARAM_kappa_min;
	u_x_max = g.TPARAM_u_x_max;
	chiF_max = g.TPARAM_chiF_max_deg*M_PI/180.f;  // default: 178/180*PI [rad]
	// 非線形項の最大・最小値
	z1_max = (v_g_max + u_x_max)*kappa_max;
	z1_min = (v_g_max + u_x_max)*kappa_min;
	z2_max = v_g_max;
	z2_min = v_g_max*sinf(chiF_max)/chiF_max;
	// 初期値
	Path_Mode = 0;  // 目標経路の設定
	s = 0;  // 経路長 [m] >= 0
	zeta = 0;  // 媒介変数 >= 0
	//P0 = location_diff(Path_Origin, prev_WP_loc);  // P0を更新: 目標経路中心からの変位(N.E <-> x,y) [m]
	//P1 = location_diff(Path_Origin, next_WP_loc);  // P1を更新: 目標経路中心からの変位(N.E <-> x,y) [m]
	P0 = Path_Origin.get_distance_NE(prev_WP_loc);  // P0を更新: 目標経路中心からの変位(N.E <-> x,y) [m]
	P1 = Path_Origin.get_distance_NE(next_WP_loc);  // P1を更新: 目標経路中心からの変位(N.E <-> x,y) [m]
	//dist_WPs = get_distance(prev_WP_loc, next_WP_loc);  // 2つのWP間の距離 [m]を更新
	dist_WPs = prev_WP_loc.get_distance(next_WP_loc);  // 2つのWP間の距離 [m]を更新
	i_now_CMD = 0;
	u_x = 0;
	t_now = AP_HAL::micros64();  // 現在の時刻 [us]
}


/* ##### Added by Kaito Yamamoto 2022.04.02. #####
 * "目標経路の生成"
 * 概要: 目標点 P の慣性座標位置や目標航路角を現在の経路長 s から生成する
*/
void Plane::TLAB_generate_2D_Path(void){
	uint16_t i_prev_CMD = i_now_CMD;
	i_now_CMD = TLAB_CMD_index;  // CMDインデックスの更新
	float zeta_prev = zeta;

	// Commented out 2021.08.26.
	/*
	if (s < 0) {
		s = 0;
	}
	*/

	float Px, Py;  // 構造体 Vector2f のx, yを置換するために使用する
	Location WP0;  // 目標経路の初期位置を設定するためのローカル変数
	// フライトプランを指定する
	switch (Flight_Plan) {
	// Mode 0: HP -(直線)-> WP1, WP0 -(直線)-> WP2 -...
	// 指定WP -- N個 (何個でも指定可能.ただし,HPはAUTOモードに切り替えた地点が割り当てられる.)
	case 0:
		Path_Mode = 0;
		// AUTOモードに切り替わり,WPがセットされたとき
		if (zeta < 0.1 && i_now_CMD != i_prev_CMD) {
			// 直線経路を更新(初回なので定義)する
			//dist_WPs = get_distance(prev_WP_loc, next_WP_loc);  // 2つのWP間の距離 [m]を更新
			dist_WPs = prev_WP_loc.get_distance(next_WP_loc);  // 2つのWP間の距離 [m]を更新
			P0 = Path_Origin.get_distance_NE(prev_WP_loc);  // P0を更新: 目標経路中心からの変位(N.E <-> y,x) [m]
			Px = P0.y;
			Py = P0.x;
			P0.x = Px;
			P0.y = Py;
			P1 = Path_Origin.get_distance_NE(next_WP_loc);  // P1を更新: 目標経路中心からの変位(N.E <-> y,x) [m]
			Px = P1.y;
			Py = P1.x;
			P1.x = Px;
			P1.y = Py;
		}
		// WP半径内に到達し,WPが新たにセットされたとき
		else if (zeta >= 0.1 && i_now_CMD != i_prev_CMD) {
			// 次に媒介変数zetaが1に達したときに,P0
			change_path_flag = true;
		}
		// WP半径内に到達したあとで,zetaが1に到達したとき
		if (change_path_flag == true && zeta >= 1) {
			// 直線経路の切り替え: s, zeta の初期化
			s = 0;
			zeta = 0;
			change_path_flag = false;
			// 直線経路の更新
			if (i_now_CMD == 2) {
				// 検証用 目標経路の初期位置 緯度・経度の設定
				WP0.lat = g.TPARAM_Path_Origin_lat;  // 目標経路の初期位置(WP0) 緯度 [1e-7*deg]
				WP0.lng = g.TPARAM_Path_Origin_lng;  // 目標経路の初期位置(WP0) 経度 [1e-7*deg]
				dist_WPs = WP0.get_distance(next_WP_loc);  // 2つのWP間の距離 [m]を更新
				P0 = Path_Origin.get_distance_NE(WP0);  // P0をMPで設定した位置にセット: 目標経路中心からの変位(N.E <-> y,x) [m]
				Px = P0.y;
				Py = P0.x;
				P0.x = Px;
				P0.y = Py;
			}
			else {
				//dist_WPs = get_distance(prev_WP_loc, next_WP_loc);  // 2つのWP間の距離 [m]を更新
				dist_WPs = prev_WP_loc.get_distance(next_WP_loc);  // 2つのWP間の距離 [m]を更新
				P0 = Path_Origin.get_distance_NE(prev_WP_loc);  // P0を更新: 目標経路中心からの変位(N.E <-> y,x) [m]
				Px = P0.y;
				Py = P0.x;
				P0.x = Px;
				P0.y = Py;
			}
			P1 = Path_Origin.get_distance_NE(next_WP_loc);  // P1を更新: 目標経路中心からの変位(N.E <-> y,x) [m]
			Px = P1.y;
			Py = P1.x;
			P1.x = Px;
			P1.y = Py;
		}
		break;
	// Mode 1: WP0 -(直線)-> WP1 -(円経路・右旋回)-> WP2
	// 指定WP -- N個 (何個でも指定可能.ただし,HPはAUTOモードに切り替えた地点が割り当てられる.)
	case 1:
		if (Path_Mode == 0) {
			// AUTOモードに切り替わり,WPがセットされたとき
			if (zeta < 0.1 && i_now_CMD != i_prev_CMD) {
				Path_Mode = 0;  // 直線経路モードにセット
				// 直線経路を更新(初回なので定義)する
				// 目標経路の初期位置 緯度・経度の設定
				WP0.lat = g.TPARAM_Path_Origin_lat;  // 目標経路の初期位置(WP0) 緯度 [1e-7*deg]
				WP0.lng = g.TPARAM_Path_Origin_lng;  // 目標経路の初期位置(WP0) 経度 [1e-7*deg]
				//dist_WPs = get_distance(WP0, next_WP_loc);  // 2つのWP間の距離 [m]を更新
				dist_WPs = WP0.get_distance(next_WP_loc);  // 2つのWP間の距離 [m]を更新
				P0 = Path_Origin.get_distance_NE(WP0);  // P0をMPで設定した位置にセット: 目標経路中心からの変位(N.E <-> y,x) [m]
				Px = P0.y;
				Py = P0.x;
				P0.x = Px;
				P0.y = Py;
				//P0 = location_diff(Path_Origin, prev_WP_loc);  // P0を更新: 目標経路中心からの変位(N.E <-> y,x) [m]
				//Px = P0.y;
				//Py = P0.x;
				//P0.x = Px;
				//P0.y = Py;
				P1 = Path_Origin.get_distance_NE(next_WP_loc);  // P1を更新: 目標経路中心からの変位(N.E <-> y,x) [m]
				Px = P1.y;  // East軸方向の変位 [m]
				Py = P1.x;  // North軸方向の変位 [m]
				P1.x = Px;
				P1.y = Py;
			}
			// WP半径内に到達し,WPが新たにセットされたとき
			else if (zeta >= 0.1 && i_now_CMD != i_prev_CMD) {
				// 次に媒介変数zetaが1に達したときに,P0
				change_path_flag = true;
			}
			// WP半径内に到達したあとで,zetaが1に到達したとき
			if (change_path_flag == true && zeta >= 1) {
				// 目標経路の切り替え: s, zeta の初期化
				s = 0;
				zeta_prev = 0;  // zeta 初期化の瞬間に dot_zetaが大きくなるのを防ぐため
				change_path_flag = false;
				// P0, P1の更新
				//dist_WPs = get_distance(prev_WP_loc, next_WP_loc);  // 2つのWP間の距離 [m]を更新 ---使わない
				dist_WPs = prev_WP_loc.get_distance(next_WP_loc);  // 2つのWP間の距離 [m]を更新 ---使わない
				P0 = Path_Origin.get_distance_NE(prev_WP_loc);  // P0を更新: 目標経路中心からの変位(N.E <-> y,x) [m]
				Px = P0.y;
				Py = P0.x;
				P0.x = Px;
				P0.y = Py;
				P1 = Path_Origin.get_distance_NE(next_WP_loc);  // P1を更新: 目標経路中心からの変位(N.E <-> y,x) [m]
				Px = P1.y;
				Py = P1.x;
				P1.x = Px;
				P1.y = Py;
				Path_Mode = 5;  // 円経路 右旋回モードにセット
			}
		}

		if (Path_Mode == 5) {
			// do nothing
			// 経路の切り替えはしない  -> 円経路を旋回し続ける
		}
		break;
	// Mode 2: HP -(直線)-> WP1 -(直線)-> WP2 -> -(リサージュ曲線 8の字 2周) -> WP3 -(直線)-> WP4 ...
	// 指定WP -- N個 (何個でも指定可能.ただし,HPはAUTOモードに切り替えた地点が割り当てられる.)
	case 2:
		if (Path_Mode == 0) {
			// AUTOモードに切り替わり,WPがセットされたとき
			if (zeta < 0.1 && i_now_CMD != i_prev_CMD) {
				// 直線経路を更新(初回なので定義)する
				// 目標経路の初期位置 緯度・経度の設定
				//WP0.lat = g.TPARAM_Path_Origin_lat;  // 目標経路の初期位置(WP0) 緯度 [1e-7*deg]
				//WP0.lng = g.TPARAM_Path_Origin_lng;  // 目標経路の初期位置(WP0) 経度 [1e-7*deg]
				//dist_WPs = get_distance(WP0, next_WP_loc);  // 2つのWP間の距離 [m]を更新
				//P0 = location_diff(Path_Origin, WP0);  // P0をMPで設定した位置にセット: 目標経路中心からの変位(N.E <-> y,x) [m]
				//Px = P0.y;
				//Py = P0.x;
				//P0.x = Px;
				//P0.y = Py;
				//dist_WPs = get_distance(prev_WP_loc, next_WP_loc);  // 2つのWP間の距離 [m]を更新
				dist_WPs = prev_WP_loc.get_distance(next_WP_loc);  // 2つのWP間の距離 [m]を更新
				P0 = Path_Origin.get_distance_NE(prev_WP_loc);  // P0を更新: 目標経路中心からの変位(N.E <-> y,x) [m]
				Px = P0.y;
				Py = P0.x;
				P0.x = Px;
				P0.y = Py;
				P1 = Path_Origin.get_distance_NE(next_WP_loc);  // P1を更新: 目標経路中心からの変位(N.E <-> y,x) [m]
				Px = P1.y;
				Py = P1.x;
				P1.x = Px;
				P1.y = Py;
			}
			// WP半径内に到達し,WPが新たにセットされたとき
			else if (zeta >= 0.1 && i_now_CMD != i_prev_CMD) {
				// 次に媒介変数zetaが1に達したときに,P0
				change_path_flag = true;
			}
			// WP半径内に到達したあとで,zetaが1に到達したとき
			if (change_path_flag == true && zeta >= 1) {
				// 目標経路の切り替え: s, zeta の初期化
				s = 0;
				zeta_prev = 0;  // zeta 初期化の瞬間に dot_zetaが大きくなるのを防ぐため
				change_path_flag = false;
				// P0, P1の更新
				//dist_WPs = get_distance(prev_WP_loc, next_WP_loc);  // 2つのWP間の距離 [m]を更新 ---使わない
				dist_WPs = prev_WP_loc.get_distance(next_WP_loc);  // 2つのWP間の距離 [m]を更新 ---使わない
				P0 = Path_Origin.get_distance_NE(prev_WP_loc);  // P0を更新: 目標経路中心からの変位(N.E <-> y,x) [m]
				Px = P0.y;
				Py = P0.x;
				P0.x = Px;
				P0.y = Py;
				P1 = Path_Origin.get_distance_NE(next_WP_loc);  // P1を更新: 目標経路中心からの変位(N.E <-> y,x) [m]
				Px = P1.y;
				Py = P1.x;
				P1.x = Px;
				P1.y = Py;
				if (i_now_CMD == 3) {
					Path_Mode = 6;  // リサージュ曲線（8の字）経路モードにセット
				}
			}
		}
		if (Path_Mode == 6) {
			// WP半径内に到達し,WPが新たにセットされたとき
			if (zeta >= 0.1 && i_now_CMD != i_prev_CMD) {
				// 次に媒介変数zetaが1に達したときに,P0
				change_path_flag = true;
			}
			// WP半径内に到達したあとで,zetaが4PIに到達したとき
			if (change_path_flag == true && zeta >= 4*M_PI) {
				// 目標経路の切り替え: s, zeta の初期化
				s = 0;
				zeta_prev = 0;  // zeta 初期化の瞬間に dot_zetaが大きくなるのを防ぐため
				change_path_flag = false;
				// P0, P1の更新
				//dist_WPs = get_distance(prev_WP_loc, next_WP_loc);  // 2つのWP間の距離 [m]を更新 ---使わない
				dist_WPs = prev_WP_loc.get_distance(next_WP_loc);  // 2つのWP間の距離 [m]を更新 ---使わない
				P0 = Path_Origin.get_distance_NE(prev_WP_loc);  // P0を更新: 目標経路中心からの変位(N.E <-> y,x) [m]
				Px = P0.y;
				Py = P0.x;
				P0.x = Px;
				P0.y = Py;
				P1 = Path_Origin.get_distance_NE(next_WP_loc);  // P1を更新: 目標経路中心からの変位(N.E <-> y,x) [m]
				Px = P1.y;
				Py = P1.x;
				P1.x = Px;
				P1.y = Py;
				Path_Mode = 0;  // 直線経路モードにセット
			}
		}
		break;
	// Mode 3: HP -(直線)-> WP1 -(直線)-> WP2 -> -(リサージュ曲線 電通大マーク) -> WP3 -(直線)-> WP4 ...
	// 指定WP -- N個 (何個でも指定可能.ただし,HPはAUTOモードに切り替えた地点が割り当てられる.)
	case 3:
		if (Path_Mode == 0) {
			// AUTOモードに切り替わり,WPがセットされたとき
			if (zeta < 0.1 && i_now_CMD != i_prev_CMD) {
				// 直線経路を更新(初回なので定義)する
				// 目標経路の初期位置 緯度・経度の設定
				//WP0.lat = g.TPARAM_Path_Origin_lat;  // 目標経路の初期位置(WP0) 緯度 [1e-7*deg]
				//WP0.lng = g.TPARAM_Path_Origin_lng;  // 目標経路の初期位置(WP0) 経度 [1e-7*deg]
				//dist_WPs = get_distance(WP0, next_WP_loc);  // 2つのWP間の距離 [m]を更新
				//P0 = location_diff(Path_Origin, WP0);  // P0をMPで設定した位置にセット: 目標経路中心からの変位(N.E <-> y,x) [m]
				//Px = P0.y;
				//Py = P0.x;
				//P0.x = Px;
				//P0.y = Py;
				dist_WPs = prev_WP_loc.get_distance(next_WP_loc);  // 2つのWP間の距離 [m]を更新
				P0 = Path_Origin.get_distance_NE(prev_WP_loc);  // P0を更新: 目標経路中心からの変位(N.E <-> y,x) [m]
				Px = P0.y;
				Py = P0.x;
				P0.x = Px;
				P0.y = Py;
				P1 = Path_Origin.get_distance_NE(next_WP_loc);  // P1を更新: 目標経路中心からの変位(N.E <-> y,x) [m]
				Px = P1.y;
				Py = P1.x;
				P1.x = Px;
				P1.y = Py;
			}
			// WP半径内に到達し,WPが新たにセットされたとき
			else if (zeta >= 0.1 && i_now_CMD != i_prev_CMD) {
				// 次に媒介変数zetaが1に達したときに,P0
				change_path_flag = true;
			}
			// WP半径内に到達したあとで,zetaが1に到達したとき
			if (change_path_flag == true && zeta >= 1) {
				// 目標経路の切り替え: s, zeta の初期化
				s = 0;
				zeta_prev = 0;  // zeta 初期化の瞬間に dot_zetaが大きくなるのを防ぐため
				change_path_flag = false;
				// P0, P1の更新
				dist_WPs = prev_WP_loc.get_distance(next_WP_loc);  // 2つのWP間の距離 [m]を更新 ---使わない
				P0 = Path_Origin.get_distance_NE(prev_WP_loc);  // P0を更新: 目標経路中心からの変位(N.E <-> y,x) [m]
				Px = P0.y;
				Py = P0.x;
				P0.x = Px;
				P0.y = Py;
				P1 = Path_Origin.get_distance_NE(next_WP_loc);  // P1を更新: 目標経路中心からの変位(N.E <-> y,x) [m]
				Px = P1.y;
				Py = P1.x;
				P1.x = Px;
				P1.y = Py;
				if (i_now_CMD == 3) {
					P1.x -= g.TPARAM_r;
					P1.y -= g.TPARAM_r;
					Path_Mode = 3;  // リサージュ曲線（電通大マーク）経路モードにセット
				}
			}
		}
		if (Path_Mode == 3) {
			// WP半径内に到達し,WPが新たにセットされたとき
			if (zeta >= 0.1 && i_now_CMD != i_prev_CMD) {
				// 次に媒介変数zetaが1に達したときに,P0
				change_path_flag = true;
			}
			// WP半径内に到達したあとで,zetaがPIに到達したとき
			if (change_path_flag == true && zeta >= M_PI) {
				// 目標経路の切り替え: s, zeta の初期化
				s = 0;
				zeta_prev = 0;  // zeta 初期化の瞬間に dot_zetaが大きくなるのを防ぐため
				change_path_flag = false;
				// P0, P1の更新
				dist_WPs = prev_WP_loc.get_distance(next_WP_loc);  // 2つのWP間の距離 [m]を更新 ---使わない
				P0 = Path_Origin.get_distance_NE(prev_WP_loc);  // P0を更新: 目標経路中心からの変位(N.E <-> y,x) [m]
				Px = P0.y;
				Py = P0.x;
				P0.x = Px;
				P0.y = Py;
				P1 = Path_Origin.get_distance_NE(next_WP_loc);  // P1を更新: 目標経路中心からの変位(N.E <-> y,x) [m]
				Px = P1.y;
				Py = P1.x;
				P1.x = Px;
				P1.y = Py;
				Path_Mode = 0;  // 直線経路モードにセット
			}
		}
		break;
	// Mode 4: WP0 -(直線)-> WP1 -(円経路・左旋回)-> WP2
	// 指定WP -- N個 (何個でも指定可能.ただし,HPはAUTOモードに切り替えた地点が割り当てられる.)
	case 4:
		if (Path_Mode == 0) {
			// AUTOモードに切り替わり,WPがセットされたとき
			if (zeta < 0.1 && i_now_CMD != i_prev_CMD) {
				Path_Mode = 0;  // 直線経路モードにセット
				// 直線経路を更新(初回なので定義)する
				// 目標経路の初期位置 緯度・経度の設定
				WP0.lat = g.TPARAM_Path_Origin_lat;  // 目標経路の初期位置(WP0) 緯度 [1e-7*deg]
				WP0.lng = g.TPARAM_Path_Origin_lng;  // 目標経路の初期位置(WP0) 経度 [1e-7*deg]
				dist_WPs = WP0.get_distance(next_WP_loc);  // 2つのWP間の距離 [m]を更新
				P0 = Path_Origin.get_distance_NE(WP0);  // P0をMPで設定した位置にセット: 目標経路中心からの変位(N.E <-> y,x) [m]
				Px = P0.y;
				Py = P0.x;
				P0.x = Px;
				P0.y = Py;
				//P0 = location_diff(Path_Origin, prev_WP_loc);  // P0を更新: 目標経路中心からの変位(N.E <-> y,x) [m]
				//Px = P0.y;
				//Py = P0.x;
				//P0.x = Px;
				//P0.y = Py;
				P1 = Path_Origin.get_distance_NE(next_WP_loc);  // P1を更新: 目標経路中心からの変位(N.E <-> y,x) [m]
				Px = P1.y;
				Py = P1.x;
				P1.x = Px;
				P1.y = Py;
			}
			// WP半径内に到達し,WPが新たにセットされたとき
			else if (zeta >= 0.1 && i_now_CMD != i_prev_CMD) {
				// 次に媒介変数zetaが1に達したときに,P0
				change_path_flag = true;
			}
			// WP半径内に到達したあとで,zetaが1に到達したとき
			if (change_path_flag == true && zeta >= 1) {
				// 目標経路の切り替え: s, zeta の初期化
				s = 0;
				zeta_prev = 0;  // zeta 初期化の瞬間に dot_zetaが大きくなるのを防ぐため
				change_path_flag = false;
				// P0, P1の更新
				dist_WPs = prev_WP_loc.get_distance(next_WP_loc);  // 2つのWP間の距離 [m]を更新 ---使わない
				P0 = Path_Origin.get_distance_NE(prev_WP_loc);  // P0を更新: 目標経路中心からの変位(N.E <-> y,x) [m]
				Px = P0.y;
				Py = P0.x;
				P0.x = Px;
				P0.y = Py;
				P1 = Path_Origin.get_distance_NE(next_WP_loc);  // P1を更新: 目標経路中心からの変位(N.E <-> y,x) [m]
				Px = P1.y;
				Py = P1.x;
				P1.x = Px;
				P1.y = Py;
				Path_Mode = 4;  // 円経路 左旋回モードにセット
			}
		}
		if (Path_Mode == 4) {
			// do nothing
			// 経路の切り替えはしない  -> 円経路を旋回し続ける
		}
		break;
	}  // switch(Flight_Plan)文の終わり

	float dPdzeta = 0;
	// 経路の種類を指定する
	switch (Path_Mode) {
	// Mode 0: 2点のWP(P0, P1)で定義される直線経路
	case 0:
		zeta = s/dist_WPs;  // 媒介変数の更新 (s の関数)
		dot_zeta = (zeta - zeta_prev)/dt_TL;  // 使わない
		x_d = (1 - zeta)*P0.x + zeta*P1.x;  // [m] x,y が実際とは逆を意味していることに注意
		y_d = (1 - zeta)*P0.y + zeta*P1.y;  // [m]
		chi_d = - atan2f((P1.y - P0.y), (P1.x - P0.x));  // [rad]: (-PI ~ PI)
		dot_chi_d = 0;
		kappa = 0;
		break;
	// Mode 1: 2点のWP(P0, P1)で定義される円経路(左旋回)
	case 1:
		zeta = s/dist_WPs*2;
		dot_zeta = (zeta - zeta_prev)/dt_TL;  // 使わない
		x_d = dist_WPs/2*cosf(zeta) + (P0.x + P1.x)/2;
		y_d = dist_WPs/2*sinf(zeta) + (P0.y + P1.y)/2;
		chi_d = atan2f(cosf(zeta), sinf(zeta));
		dot_chi_d = - dot_zeta;
		kappa = 2/dist_WPs;
		break;
	// Mode 2: 2点のWP(P0, P1)で定義される円経路(右旋回)
	case 2:
		zeta = s/dist_WPs*2;
		dot_zeta = (zeta - zeta_prev)/dt_TL;  // 使わない
		x_d = dist_WPs/2*cosf(-zeta) + (P0.x + P1.x)/2;
		y_d = dist_WPs/2*sinf(-zeta) + (P0.y + P1.y)/2;
		chi_d = atan2f(cosf(zeta), -sinf(zeta));
		dot_chi_d = dot_zeta;
		kappa = 2/dist_WPs;
		break;
	// Mode 3: 1点のWP(P1)と半径rで定義されるリサージュ曲線経路(電通大マークver.) : 左下が始点
	case 3:
		// s と dPdzeta から数値計算でzetaを求める
		while (1) {
			float dPdzeta2 = 25*powf(sinf(5*g.TPARAM_dzeta*i_zeta), 2) + 36*pow(sinf(6*g.TPARAM_dzeta*i_zeta), 2);
			dPdzeta = g.TPARAM_r*sqrt(dPdzeta2);
			s_calc += dPdzeta*g.TPARAM_dzeta;
			if (s_calc >= s) {
				zeta = i_zeta*g.TPARAM_dzeta;
				break;
			}
			i_zeta ++;
		}
		dot_zeta = (zeta - zeta_prev)/dt_TL;
		x_d = - g.TPARAM_r*cosf(5.f*zeta) + P1.x;
		y_d = g.TPARAM_r*cosf(6.f*zeta) + P1.y;
		chi_d = atan2f(6.f/5.f*sinf(6.f*zeta), sinf(5.f*zeta));
		dot_chi_d = - 30*dot_zeta*(sinf(11*zeta) - 11*sinf(zeta))/(25*cosf(10*zeta) + 36*cosf(12*zeta) - 61.f);
		kappa = (15*fabsf(11.f*sinf(zeta) - sinf(11*zeta))) / (g.TPARAM_r*powf((25*powf(sinf(5*zeta) , 2) + 36*powf(sinf(6*zeta) , 2)) , 1.5f));
		break;
	// Mode 4: 1点のWP(P1)と半径rで定義される円経路(左旋回): 初期位相 +PI
	case 4:
		zeta = s/g.TPARAM_r;
		dot_zeta = (zeta - zeta_prev)/dt_TL;
		x_d = - g.TPARAM_r*cosf(zeta) + P1.x;
		y_d = - g.TPARAM_r*sinf(zeta) + P1.y;
		chi_d = atan2f(cosf(zeta), sinf(zeta));
		dot_chi_d = - dot_zeta;
		kappa = 1.f/g.TPARAM_r;
		break;
	// Mode 5: 1点のWP(P1)と半径rで定義される円経路(右旋回): 初期位相 +PI
	case 5:
		zeta = s/g.TPARAM_r;
		dot_zeta = (zeta - zeta_prev)/dt_TL;
		x_d = - g.TPARAM_r*cosf(zeta) + P1.x;
		y_d = g.TPARAM_r*sinf(zeta) + P1.y;
		chi_d = atan2f(-cosf(zeta), sinf(zeta));
		dot_chi_d = dot_zeta;
		kappa = 1.f/g.TPARAM_r;
		break;
	// Mode 6: 1点のWP(P1)と半径rで定義されるリサージュ曲線経路(8の字ver.) : 初期ベクトルは右上に向かって
	case 6:
		// s と dPdzeta から数値計算でzetaを求める
		while (1) {
			dPdzeta = g.TPARAM_r*sqrt((25*powf(sinf(5*g.TPARAM_dzeta*i_zeta), 2) + 36*pow(sinf(6*g.TPARAM_dzeta*i_zeta), 2)));
			s_calc += dPdzeta*g.TPARAM_dzeta;
			if (s_calc >= s) {
				zeta = i_zeta*g.TPARAM_dzeta;
				break;
			}
			i_zeta ++;  // i_zeta = i_zeta + 1;
		}
		dot_zeta = (zeta - zeta_prev)/dt_TL;
		x_d = 2*g.TPARAM_r*sinf(zeta) + P1.x;
		y_d = g.TPARAM_r*sinf(2*zeta) + P1.y;
		chi_d = atan2f(- cosf(2*zeta), cosf(zeta));
		dot_chi_d = - (dot_zeta*sinf(zeta)*(2*powf(sinf(zeta), 2) - 3))/(4*powf(sinf(zeta), 4) - 5*powf(sinf(zeta), 2) + 2);
		kappa = - (fabsf(sinf(zeta))*(2*powf(sinf(zeta) , 2) - 3))/(2*g.TPARAM_r*powf((4*powf(cosf(zeta) , 4) - 3*powf(cosf(zeta) , 2) + 1) , 1.5f));
		break;
	default:
		zeta = 0;
		break;
	}  // switch(Path_Mode)文の終わり
}


/* ##### Added by Kaito Yamamoto 2022.04.02. #####
 * "PPG 2次元経路追従コントローラ"
 * 参考文献: 高橋裕徳さんの修論(2016), ミーティング資料2021/05/16 etc.
 * 概要: コントロールバーのサーボモータ角度 [cdeg.]を計算して返す
 * 制御目的: 設定した2次元経路への追従（セレ・フレネ座標系におけるレギュレータとして設計）
*/
int32_t Plane::TLAB_2D_Trace_Controller(void){
	// 変数の初期設定
	if (!yet_init) {
		init_TLAB_2D_Trace_Controller();
		yet_init = true;
		return 0;  // dt_TL = 0 による0割りを防ぐため
	}

	// 慣性座標系における現在の状態の更新(IMU + GPS + Barometer)
	uint64_t t_prev = t_now;  // 直前の時刻 [us]
	t_now = AP_HAL::micros64();  // 現在の時刻 [us]
	dt_TL = (t_now - t_prev)*1.e-6f;  // サンプリング時間間隔 [s]
	Vector2f xyI = Path_Origin.get_distance_NE(current_loc);  // 現在地点: 目標経路中心からの変位(N.E <-> y,x) [m]
	xI = xyI.y;  // 慣性x座標(緯度方向) [m]
	yI = xyI.x;  // 慣性y座標(経度方向) [m]
	psi = wrap_2PI(ahrs.yaw - M_PI/2);  // ヨー角(機首方位角) [rad] (0 ~ 2PI)
	chi = wrap_2PI(gps.ground_course()*M_PI/180.0f - M_PI/2);  // 航路角 [rad]: (0 ~ 2PI)
	v_g_TL = gps.ground_speed();  // 対地速度の大きさ [m/s]

	// 経路生成: 経路情報(目標点 P の慣性座標 etc.)を現在の経路長 s から計算する
	TLAB_generate_2D_Path();

	// 慣性座標系{I}からセレ・フレネ座標系{F}への変換
	float e_xI = x_d - xI;  // 慣性座標 x の制御偏差 [m]
	float e_yI = y_d - yI;  // 慣性座標 y の制御偏差 [m]
	xF = - cosf(chi_d)*e_xI + sinf(chi_d)*e_yI;  // [m]
	yF = - sinf(chi_d)*e_xI - cosf(chi_d)*e_yI;  // [m]
	chiF = wrap_PI(chi_d - chi);  // [rad]: (-PI ~ PI)
	float X[3] = {xF, yF, chiF};  // 状態変数ベクトル

	// 制御入力 u_x の計算
	u_x_calc = 0;  // 計算用 u_x の初期化
	for (int i = 0; i < 3; i++) {
		u_x_calc -= Fx[i]*X[i];
	}
	/*
	// u_x のファジィ化範囲上限 u_x_max で入力値をカット
	if (u_x_calc > u_x_max) {
		u_x = u_x_max;
	}
	else if (u_x_calc < - u_x_max) {
		u_x = - u_x_max;
	}
	else {
		u_x = u_x_calc;
	}
	*/
	u_x = u_x_calc;

	// メンバーシップ関数の計算
	// 非線形項
	float z1 = (v_g_TL*cosf(chiF) + u_x)*kappa;  // 係数行列の非線形項 1
	float z2 = v_g_TL*sinf(chiF)/chiF;  // 係数行列の非線形項2
	// K1, K2, M1, M2 の計算
	if (chiF < 1.2e-38 && chiF > -1.2e-38) {
		// 0割り対策
		K1 = 1;
		K2 = 0;
	}
	else {
		K1 = (z2 - z2_min)/(z2_max - z2_min);
		K2 = (z2_max - z2)/(z2_max - z2_min);
	}
	M1 = (z1 - z1_min)/(z1_max - z1_min);  // 分母は定数なので0割りは起こり得ない
	M2 = (z1_max - z1)/(z1_max - z1_min);
	// K1, K2, M1, M2 の上限・下限カット
	if (K1 > 1) {
		K1 = 1;
		K2 = 0;
	}
	else if (K1 < 0) {
		K1 = 0;
		K2 = 1;
	}
	if (M1 > 1) {
		M1 = 1;
		M2 = 0;
	}
	else if (M1 < 0) {
		M1 = 0;
		M2 = 1;
	}
	// メンバーシップ関数  h_chi[1] ~ h_chi[4] の計算
	h_chi[0] = K1*M1;
	h_chi[1] = K2*M1;
	h_chi[2] = K1*M2;
	h_chi[3] = K2*M2;

	// 制御入力 u_chi の計算
	float u_chi_calc = 0;  // 計算用
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 3; j++) {
			u_chi_calc -= h_chi[i]*Fchi[i][j]*X[j];
		}
	}
	u_chi = u_chi_calc;

	// 経路長  s の更新(数値積分)
	ds = u_x + v_g_TL*cosf(chiF);  // 目標点 P の移動速度 [m/s]
	s += ds*dt_TL;  // 経路長の更新式 [m]

	// u_chi [rad/s] をコントロールバー角度 bar_angle, サーボモーター角度 servo [cdeg] へ変換
    d_angle = static_cast<int32_t>(1/k_prop_const*v_g_TL/v_a_TL/cosf(chi - psi)*(-u_chi + dot_chi_d)*100.0f*180.0f/M_PI);  // [cdeg]
    bar_angle = d_angle + g.TPARAM_bar_neutral*100;  // コントロールバー角度 [cdeg]
    u = bar_angle/100.f*M_PI/180.f;  // [cdeg] --> [rad]
    servo = static_cast<int32_t>(asinf(constrain_float(58.0f / 29.0f * sinf(u), -1, 1))*100.0f*180.0f/M_PI);  // サーボモーター角度 [cdeg]
    return servo;
}


//added by hatae, 2022.3
uint8_t Plane::TLAB_readSwitch(uint8_t const channel)
{
    uint16_t pulsewidth = RC_Channels::get_radio_in(channel- 1);
    if (pulsewidth <= 900 || pulsewidth >= 2200) return 255;            // This is an error condition
    if (pulsewidth <= 1230) return 0;
    if (pulsewidth <= 1360) return 1;
    if (pulsewidth <= 1490) return 2;
    if (pulsewidth <= 1620) return 3;
    if (pulsewidth <= 1749) return 4;              // Software Manual
    return 5;                                                           // Hardware Manual
}
