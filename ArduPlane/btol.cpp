#include "Plane.h"
#include "config.h"
#if BTOL_ENABLED == ENABLED
#include "btol.h"
//#include "btolRegulator.h"
#include <utility>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
extern const AP_HAL::HAL& hal;


/*  //already in Plane.h
#include <AP_HAL/AP_HAL.h>

#if HAL_OS_POSIX_IO
#include <stdio.h>
#endif
*/ 
/*
*  
*
*  
*/

//Presently running at 50Hz.  See ArduPlane.cpp :    SCHED_TASK(update_btol,            50,    400),

//this is not the proper way to pass information but lets go with it for now....

#define RC_CHANNEL_INPUT_MAX_VALUE 2000
#define RC_CHANNEL_INPUT_MIN_VALUE 1000
#define RC_CHANNEL_INPUT_CENTER_VALUE 1500
#define RC_CHANNEL_INPUT_HALF_RANGE 500
#define RC_CHANNEL_NUMBER_FOR_ROLL_STICK 0
#define RC_CHANNEL_NUMBER_FOR_PITCH_STICK 1
#define RC_CHANNEL_NUMBER_FOR_YAW_STICK 3
#define RC_CHANNEL_NUMBER_FOR_THROTTLE_STICK 2
#define RC_CHANNEL_NUMBER_FOR_LEFT_SLIDER_STICK 6
#define RC_CHANNEL_NUMBER_FOR_RIGHT_SLIDER_STICK CH_8  //7
#define RC_CHANNEL_NUMBER_FOR_ARM_SWITCH CH_6  //2000us = forward, 1000us = aft.
#define RC_CHANNEL_NUMBER_FOR_MODE_SWITCH CH_5  //2000us = forward, 1000us = aft.

#define TILT1_SERVO_MIN_ANGLE 0.0f //radians
#define TILT1_SERVO_MIN_ANGLE_PWM 2060  //PWM uS
#define TILT1_SERVO_MAX_ANGLE 1.74533f  //radians
#define TILT1_SERVO_MAX_ANGLE_PWM 925 //PWM uS

#define TILT2_SERVO_MIN_ANGLE 0.0f //radians
#define TILT2_SERVO_MIN_ANGLE_PWM 925 //PWM uS
#define TILT2_SERVO_MAX_ANGLE 1.74533f //radians
#define TILT2_SERVO_MAX_ANGLE_PWM 2070 //PWM uS
#define THROTTLE_DISARMED_VALUE 1000 //PWM uS
#define MTV_MAX_COMMANDABLE_TILT_ANGLE_IN_RADIANS 1.5708f
#define MTV_MIN_COMMANDABLE_TILT_ANGLE_IN_RADIANS 0.0f
#define MTV_MAX_COMMANDABLE_TILT_ACCELERATION_IN_MSS 20.0f
#define MTV_MIN_COMMANDABLE_TILT_ACCELERATION_IN_MSS 0.0f

#define MOTOR_LIFT_DEFAULT_MAX_THRUST_N 16.671305f//7.0f//8.0
#define MOTOR_PROPULSION_DEFAULT_MAX_THRUST_N 16.671305f//3.0f

#define MOTOR_CONTROL_MIN_VALUE 1000
#define MOTOR_CONTROL_MAX_VALUE 2000
#define MOTOR_CONTROL_RANGE (MOTOR_CONTROL_MAX_VALUE-MOTOR_CONTROL_MIN_VALUE)

#define TOP_OF_TRANSITION_DEFAULT_DYNAMIC_PRESSURE 200.0f //N/m^2 or Pa
#define DEFAULT_VERTICAL_ACCELERATION_THRESHOLD_TO_CONSIDER_AIRCRAFT_IN_HOVER -8.0f //m/s/s
#define DEFAULT_AIRCRAFT_MASS_IN_KG 0.920f
#define DEFAULT_THRUST_MAX_MOTORS_12 8.0f //Newtons
#define DEFAULT_THRUST_MAX_MOTOR_3 3.5f //Newtons
#define DEFAULT_AIRCRAFT_CENTER_OF_MASS_METERS -0.053f //Meters
#define DEFAULT_MOTOR3_THRUST_TO_TORQUE_COEF 0.025f //NM per N thrust.
#define DEFAULT_ELEVON_COEF_OF_LIFT_PER_DEFLECTION 6.3f //(2*PI?)
#define DEFAULT_ELEVON_MINIMUM_DYNAMIC_PRESSURE 50.0f
#define DEFAULT_LOWPASS_FILTER_CUTOFF_FREQUENCY_PITCH 0.0f
#define DEFAULT_LOWPASS_FILTER_CUTOFF_FREQUENCY_ROLL 0.0f
#define DEFAULT_LOWPASS_FILTER_CUTOFF_FREQUENCY_YAW 0.0f

#define DEFAULT_AERO_DAMPING_ROLL_VS_TRUE_AIRSPEED_COEF 0.0f
#define DEFAULT_AERO_DAMPING_PITCH_VS_TRUE_AIRSPEED_COEF 0.0f
#define DEFAULT_AERO_DAMPING_YAW_VS_TRUE_AIRSPEED_COEF 0.0f

#define DEFAULT_AERO_DAMPING_ROLL_HOVER 0.0f
#define DEFAULT_AERO_DAMPING_PITCH_HOVER 0.0f
#define DEFAULT_AERO_DAMPING_YAW_HOVER 0.0f

#define DEFAULT_P_TERM_HOVER_PITCH 0.0f
#define DEFAULT_P_TERM_HOVER_FORWARD_FLIGHT_PITCH 0.0f
#define DEFAULT_I_TERM_HOVER_PITCH 0.0f
#define DEFAULT_I_TERM_HOVER_FORWARD_FLIGHT_PITCH 0.0f
#define DEFAULT_D_TERM_HOVER_PITCH 0.0f
#define DEFAULT_D_TERM_HOVER_FORWARD_FLIGHT_PITCH 0.0f

#define DEFAULT_I_TERM_MAX_HOVER_PITCH 0.0f
#define DEFAULT_I_TERM_MAX_FORWARD_FLIGHT_PITCH 0.0f

#define DEFAULT_P_TERM_HOVER_ROLL 0.0f
#define DEFAULT_P_TERM_HOVER_FORWARD_FLIGHT_ROLL 0.0f
#define DEFAULT_I_TERM_HOVER_ROLL 0.0f
#define DEFAULT_I_TERM_HOVER_FORWARD_FLIGHT_ROLL 0.0f
#define DEFAULT_D_TERM_HOVER_ROLL 0.0f
#define DEFAULT_D_TERM_HOVER_FORWARD_FLIGHT_ROLL 0.0f

#define DEFAULT_I_TERM_MAX_HOVER_ROLL 0.0f
#define DEFAULT_I_TERM_MAX_FORWARD_FLIGHT_ROLL 0.0f

#define DEFAULT_P_TERM_HOVER_YAW 0.0f
#define DEFAULT_P_TERM_HOVER_FORWARD_FLIGHT_YAW 0.0f
#define DEFAULT_I_TERM_HOVER_YAW 0.0f
#define DEFAULT_I_TERM_HOVER_FORWARD_FLIGHT_YAW 0.0f
#define DEFAULT_D_TERM_HOVER_YAW 0.0f
#define DEFAULT_D_TERM_HOVER_FORWARD_FLIGHT_YAW 0.0f

#define DEFAULT_I_TERM_MAX_HOVER_YAW 0.0f
#define DEFAULT_I_TERM_MAX_FORWARD_FLIGHT_YAW 0.0f

#define DEFAULT_COMMAND_GAIN_PASSTHROUGH_ACCELERATION_ROLL 10.0f
#define DEFAULT_COMMAND_GAIN_PASSTHROUGH_ACCELERATION_PITCH 10.0f
#define DEFAULT_COMMAND_GAIN_PASSTHROUGH_ACCELERATION_YAW 10.0f

#define DEFAULT_LOWPASS_FILTER_CUTOFF_FREQUENCY_ELEVON_ANGLE 0.0f
#define DEFAULT_LOWPASS_FILTER_CUTOFF_FREQUENCY_TILT_ANGLE 0.0f

const AP_Param::GroupInfo BTOL_Controller::var_info[] = {
	    // parameters from parent vehicle
    //AP_SUBGROUPINFO(_pid_rate_roll, "B_ROLL_", 0, BTOL_Controller, AC_PID),
    //AP_SUBGROUPINFO(_pid_rate_pitch, "B_PTCH_", 1, BTOL_Controller, AC_PID),
    //AP_SUBGROUPINFO(_pid_rate_yaw, "B_YAW_", 2, BTOL_Controller, AC_PID),

    AP_GROUPINFO("IzzYawKgMM",      0, BTOL_Controller, aircraftMomentOfInertiaYawInKgMM,       0.02f),
    AP_GROUPINFO("IyyPtcKgMM",      1, BTOL_Controller, aircraftMomentOfInertiaPitchInKgMM,       0.02f),
    AP_GROUPINFO("IxxRolKgMM",      2, BTOL_Controller, aircraftMomentOfInertiaRollInKgMM,       0.02f),

	AP_GROUPINFO("RRATECMDG",      3, BTOL_Controller, rollRateCommandGain,       0.5f),
	AP_GROUPINFO("PRATECMDG",        4, BTOL_Controller, pitchRateCommandGain,        0.5f),
	AP_GROUPINFO("YRATECMDG",        5, BTOL_Controller, yawRateCommandGain,        0.5f),

	AP_GROUPINFO("RATTICMDG",        6, BTOL_Controller, rollAttitudeCommandGain,        0.5f),

    AP_GROUPINFO("R_AttiToRat",      7, BTOL_Controller, rollAttitudeErrorToRollRateGain,       1.0f), //attitude error to rate gain.
    AP_GROUPINFO("P_AttiToRat",      8, BTOL_Controller, pitchAttitudeErrorToPitchRateGain,       1.0f),
    AP_GROUPINFO("Q_Override",      9, BTOL_Controller, overrideDynamicPressureEstimateToThisValueIfPosititive,       -1.0f),
    AP_GROUPINFO("PATTICMDG",        10, BTOL_Controller, pitchAttitudeCommandGain,        0.5f),
    AP_GROUPINFO("MLFT_MXTHR",        11, BTOL_Controller, motorLiftMaxThrust,        MOTOR_LIFT_DEFAULT_MAX_THRUST_N),
    AP_GROUPINFO("MPRO_MXTHR",        12, BTOL_Controller, motorPropulsionMaxThrust,        MOTOR_PROPULSION_DEFAULT_MAX_THRUST_N),
    AP_GROUPINFO("TOP_TRAN_Q",        13, BTOL_Controller, topOfTransitionDynamicPressure,        TOP_OF_TRANSITION_DEFAULT_DYNAMIC_PRESSURE),
    AP_GROUPINFO("HOV_AC_THR",        14, BTOL_Controller, verticalAccelerationThresholdToConsiderAircraftInHover,        DEFAULT_VERTICAL_ACCELERATION_THRESHOLD_TO_CONSIDER_AIRCRAFT_IN_HOVER),
    AP_GROUPINFO("MASS_KG",        15, BTOL_Controller, aircraftMassInKg,        DEFAULT_AIRCRAFT_MASS_IN_KG),
    AP_GROUPINFO("CG_METERS",        16, BTOL_Controller, centerOfMassLocationX,        DEFAULT_AIRCRAFT_CENTER_OF_MASS_METERS),
    AP_GROUPINFO("AzCMDmss",        17, BTOL_Controller, linearAccelerationZCommandGain,        30),
    AP_GROUPINFO("Elvn_CL",        18, BTOL_Controller, elevonCoefLiftPerDeflection,        DEFAULT_ELEVON_COEF_OF_LIFT_PER_DEFLECTION),
    AP_GROUPINFO("Elvn_min_q",        19, BTOL_Controller, elevonControlMinimumDynamicPressure,        DEFAULT_ELEVON_MINIMUM_DYNAMIC_PRESSURE),
    AP_GROUPINFO("LowPassF_P",        20, BTOL_Controller, lowpassFilterCuttofFrequencyPitch,        DEFAULT_LOWPASS_FILTER_CUTOFF_FREQUENCY_PITCH),
    AP_GROUPINFO("LowPassF_R",        21, BTOL_Controller, lowpassFilterCuttofFrequencyRoll,        DEFAULT_LOWPASS_FILTER_CUTOFF_FREQUENCY_ROLL),
    AP_GROUPINFO("LowPassF_Y",        22, BTOL_Controller, lowpassFilterCuttofFrequencyYaw,        DEFAULT_LOWPASS_FILTER_CUTOFF_FREQUENCY_YAW),

    AP_GROUPINFO("R_AeDamCa",        23, BTOL_Controller, aeroDampingVsTrueAirspeedCoefRoll,        DEFAULT_AERO_DAMPING_ROLL_VS_TRUE_AIRSPEED_COEF),
    AP_GROUPINFO("P_AeDamCa",        24, BTOL_Controller, aeroDampingVsTrueAirspeedCoefPitch,        DEFAULT_AERO_DAMPING_PITCH_VS_TRUE_AIRSPEED_COEF),
    AP_GROUPINFO("Y_AeDamCa",        25, BTOL_Controller, aeroDampingVsTrueAirspeedCoefYaw,        DEFAULT_AERO_DAMPING_YAW_VS_TRUE_AIRSPEED_COEF),

   AP_GROUPINFO("P_P_Hov",         26, BTOL_Controller, PitchRegulatorPtermHover,               DEFAULT_P_TERM_HOVER_PITCH),
    AP_GROUPINFO("P_P_FF",         27, BTOL_Controller, PitchRegulatorPtermForwardFlight,        DEFAULT_P_TERM_HOVER_FORWARD_FLIGHT_PITCH),
    AP_GROUPINFO("P_I_Hov",        28, BTOL_Controller, PitchRegulatorItermHover,               DEFAULT_I_TERM_HOVER_PITCH),
    AP_GROUPINFO("P_I_FF",        29, BTOL_Controller, PitchRegulatorItermForwardFlight,        DEFAULT_I_TERM_HOVER_FORWARD_FLIGHT_PITCH),
    AP_GROUPINFO("P_D_Hov",        30, BTOL_Controller, PitchRegulatorDtermHover,               DEFAULT_D_TERM_HOVER_PITCH),
    AP_GROUPINFO("P_D_FF",        31, BTOL_Controller, PitchRegulatorDtermForwardFlight,        DEFAULT_D_TERM_HOVER_FORWARD_FLIGHT_PITCH),

    AP_GROUPINFO("R_P_Hov",         32, BTOL_Controller, RollRegulatorPtermHover,               DEFAULT_P_TERM_HOVER_ROLL),
    AP_GROUPINFO("R_P_FF",         33, BTOL_Controller, RollRegulatorPtermForwardFlight,        DEFAULT_P_TERM_HOVER_FORWARD_FLIGHT_ROLL),
    AP_GROUPINFO("R_I_Hov",        34, BTOL_Controller, RollRegulatorItermHover,               DEFAULT_I_TERM_HOVER_ROLL),
    AP_GROUPINFO("R_I_FF",        35, BTOL_Controller, RollRegulatorItermForwardFlight,        DEFAULT_I_TERM_HOVER_FORWARD_FLIGHT_ROLL),
    AP_GROUPINFO("R_D_Hov",        36, BTOL_Controller, RollRegulatorDtermHover,               DEFAULT_D_TERM_HOVER_ROLL),
    AP_GROUPINFO("R_D_FF",        37, BTOL_Controller, RollRegulatorDtermForwardFlight,        DEFAULT_D_TERM_HOVER_FORWARD_FLIGHT_ROLL),

    AP_GROUPINFO("Y_P_Hov",         38, BTOL_Controller, YawRegulatorPtermHover,               DEFAULT_P_TERM_HOVER_YAW),
    AP_GROUPINFO("Y_P_FF",         39, BTOL_Controller, YawRegulatorPtermForwardFlight,        DEFAULT_P_TERM_HOVER_FORWARD_FLIGHT_YAW),
    AP_GROUPINFO("Y_I_Hov",        40, BTOL_Controller, YawRegulatorItermHover,               DEFAULT_I_TERM_HOVER_YAW),
    AP_GROUPINFO("Y_I_FF",        41, BTOL_Controller, YawRegulatorItermForwardFlight,        DEFAULT_I_TERM_HOVER_FORWARD_FLIGHT_YAW),
    AP_GROUPINFO("Y_D_Hov",        42, BTOL_Controller, YawRegulatorDtermHover,               DEFAULT_D_TERM_HOVER_YAW),
    AP_GROUPINFO("Y_D_FF",        43, BTOL_Controller, YawRegulatorDtermForwardFlight,        DEFAULT_D_TERM_HOVER_FORWARD_FLIGHT_YAW),

    AP_GROUPINFO("R_CMDG_PT",        44, BTOL_Controller, passthroughAngularAccelerationCommandGainRoll,        DEFAULT_COMMAND_GAIN_PASSTHROUGH_ACCELERATION_ROLL),
    AP_GROUPINFO("P_CMDG_PT",        45, BTOL_Controller, passthroughAngularAccelerationCommandGainPitch,        DEFAULT_COMMAND_GAIN_PASSTHROUGH_ACCELERATION_PITCH),
    AP_GROUPINFO("Y_CMDG_PT",        46, BTOL_Controller, passthroughAngularAccelerationCommandGainYaw,        DEFAULT_COMMAND_GAIN_PASSTHROUGH_ACCELERATION_YAW),

    AP_GROUPINFO("R_AeDamH",        47, BTOL_Controller, aeroDampingBaselineHoverRoll,        DEFAULT_AERO_DAMPING_ROLL_HOVER),
    AP_GROUPINFO("P_AeDamH",        48, BTOL_Controller, aeroDampingBaselineHoverPitch,        DEFAULT_AERO_DAMPING_PITCH_HOVER),
    AP_GROUPINFO("Y_AeDamH",        49, BTOL_Controller, aeroDampingBaselineHoverYaw,        DEFAULT_AERO_DAMPING_YAW_HOVER),

    AP_GROUPINFO("P_I_Max_H",        50, BTOL_Controller, PitchRegulatorItermMaxHover,        DEFAULT_I_TERM_MAX_HOVER_PITCH),
    AP_GROUPINFO("P_I_Max_FF",        51, BTOL_Controller, PitchRegulatorItermMaxForwardFlight,        DEFAULT_I_TERM_MAX_FORWARD_FLIGHT_PITCH),

    AP_GROUPINFO("R_I_Max_H",        52, BTOL_Controller, RollRegulatorItermMaxHover,        DEFAULT_I_TERM_MAX_HOVER_ROLL),
    AP_GROUPINFO("R_I_Max_FF",        53, BTOL_Controller, RollRegulatorItermMaxForwardFlight,        DEFAULT_I_TERM_MAX_FORWARD_FLIGHT_ROLL),
    
    AP_GROUPINFO("Y_I_Max_H",        54, BTOL_Controller, YawRegulatorItermMaxHover,        DEFAULT_I_TERM_MAX_HOVER_YAW),
    AP_GROUPINFO("Y_I_Max_FF",        55, BTOL_Controller, YawRegulatorItermMaxForwardFlight,        DEFAULT_I_TERM_MAX_FORWARD_FLIGHT_YAW),

    AP_GROUPINFO("EA_FltCOF",        56, BTOL_Controller, lowpassFilterCuttofFrequencyElevonAngle,        DEFAULT_LOWPASS_FILTER_CUTOFF_FREQUENCY_ELEVON_ANGLE),
    AP_GROUPINFO("TA_FltCOF",        57, BTOL_Controller, lowpassFilterCuttofFrequencyTiltAngle,        DEFAULT_LOWPASS_FILTER_CUTOFF_FREQUENCY_TILT_ANGLE),
    AP_GROUPINFO("VCompCoef",        58, BTOL_Controller, BatteryVoltageCompensationCoeficent,        1.0),
    AP_GROUPINFO("MixTopQ",        59, BTOL_Controller, EffectorMixingDynamicPressureTop,        200),
    AP_GROUPINFO("MixBotQ",        60, BTOL_Controller, EffectorMixingDynamicPressureBottom,        50),
    AP_GROUPINFO("EleOvrflRAT",        61, BTOL_Controller, ElevonResidualOverflowRatio,        1.0),
    AP_GROUPINFO("AttiStbTopQ",        62, BTOL_Controller, topOfAttitudeFeedbackDynamicPressure,        100),
    AP_GROUPINFO("H_RTrimTopQ",        63, BTOL_Controller, automaticHoverRollTrimDynamicPressureMax,        20),

    //I think there may be an issue with more than 64 parameters ie: if there are 63, that's the max.  No 64!  Yep.  It's an issue!
    //Make sure that the number is progressed!
	AP_GROUPEND
};

void Plane::update_btol() {  //50Hz
    //take pilot input
    //get the rc input and map them to a range of -1.0 to +1.0;
    //TODO: Add in a something which zeros the values if the incoming raw signal is zero.
	static uint32_t _last_t_ms = 0;
    uint32_t tnow_ms = AP_HAL::millis(); //this should be micros if we want to see 400Hz
	uint32_t dt_ms = tnow_ms - _last_t_ms;
	if (_last_t_ms == 0 || dt_ms > 1000) {
		dt_ms = 0;  //presently this is being logged as 20, which means we are running at 50Hz
	}
	_last_t_ms = tnow_ms;

   float rcCommandInputPitchStickAft = 0;
   float rcCommandInputRollStickRight = 0;
   float rcCommandInputYawStickRight = 0;
   float rcCommandInputThrottleStickForward = 0;
   float rcCommandInputLeftSliderStickForward = 0;
   float rcCommandInputRightSliderStickForward = 0;
   //float rcCommandInputArmSwitch = 0;
    
    //need protections which set these to neutral values if the signal isn't present. TODO
    rcCommandInputPitchStickAft = constrain_float( ((float)hal.rcin->read(RC_CHANNEL_NUMBER_FOR_PITCH_STICK) - RC_CHANNEL_INPUT_CENTER_VALUE)/RC_CHANNEL_INPUT_HALF_RANGE, -1.0, 1.0);  //this should be a function, private
    rcCommandInputRollStickRight = constrain_float( ((float)hal.rcin->read(RC_CHANNEL_NUMBER_FOR_ROLL_STICK) - RC_CHANNEL_INPUT_CENTER_VALUE)/RC_CHANNEL_INPUT_HALF_RANGE, -1.0, 1.0);  //I don't know if any of these are in the correct orientation.
    rcCommandInputYawStickRight = constrain_float( ((float)hal.rcin->read(RC_CHANNEL_NUMBER_FOR_YAW_STICK) - RC_CHANNEL_INPUT_CENTER_VALUE)/RC_CHANNEL_INPUT_HALF_RANGE, -1.0, 1.0); 
    rcCommandInputThrottleStickForward = constrain_float( ((float)hal.rcin->read(RC_CHANNEL_NUMBER_FOR_THROTTLE_STICK) - RC_CHANNEL_INPUT_CENTER_VALUE)/RC_CHANNEL_INPUT_HALF_RANGE, -1.0, 1.0); 
    rcCommandInputLeftSliderStickForward = constrain_float( ((float)hal.rcin->read(RC_CHANNEL_NUMBER_FOR_LEFT_SLIDER_STICK) - RC_CHANNEL_INPUT_CENTER_VALUE)/RC_CHANNEL_INPUT_HALF_RANGE, -1.0, 1.0); 
    rcCommandInputRightSliderStickForward = constrain_float( ((float)hal.rcin->read(RC_CHANNEL_NUMBER_FOR_RIGHT_SLIDER_STICK) - RC_CHANNEL_INPUT_CENTER_VALUE)/RC_CHANNEL_INPUT_HALF_RANGE, -1.0, 1.0); 

    AP::logger().Write("B_RC", "TimeUS,dt_ms,P,R,Y,T,LS,RS",
                   "SS------", // units: seconds, rad/sec
                   "F0000000", // mult: 1e-6, 1e-2
                   "QIffffff", // format: uint64_t, float
                   AP_HAL::micros64(),
                   dt_ms,
                   (double)rcCommandInputPitchStickAft,
                   (double)rcCommandInputRollStickRight,
                   (double)rcCommandInputYawStickRight,
                   (double)rcCommandInputThrottleStickForward,
                   (double)rcCommandInputLeftSliderStickForward,
                   (double)rcCommandInputRightSliderStickForward
                   );

    float targetHoverPitchAttitude = 0.0f;
    #define HOVER_TARGET_PITCH_NOMINAL_ATTITUDE 0.0872665f //5 deg
    #define HOVER_TARGET_PITCH_NOMINAL_ATTITUDE_COMMANDABLE_DELTA 0.0872665f //5 deg
    targetHoverPitchAttitude = HOVER_TARGET_PITCH_NOMINAL_ATTITUDE + rcCommandInputRightSliderStickForward * HOVER_TARGET_PITCH_NOMINAL_ATTITUDE_COMMANDABLE_DELTA;
    g2.btolController.setCommandedHoverPitchAttitudeBaseline(targetHoverPitchAttitude);//rad 

    #define MAX_BODY_AXIS_POWERED_LIFT_ACCELERATION_COMMAND_MS 20.0
    #define MAX_BODY_AXIS_POWERED_ACCELERATION_COMMAND_MS 15.0
    
    //float desiredUpAccelerationComponent = ((rcCommandInputThrottleStickForward + 1.0f) / 2.0f) * MAX_BODY_AXIS_POWERED_LIFT_ACCELERATION_COMMAND_MS; //this is the wrong direciton, of course!
    //AP_GROUPINFO("AzCMDmss",        17, BTOL_Controller, linearAccelerationZCommandGain,        30),
    float desiredUpAccelerationComponent = ((rcCommandInputThrottleStickForward + 1.0f) / 2.0f) * g2.btolController.getLinearAccelerationZCommandGain(); //this is the wrong direciton, of course!

    float desiredForwardAccelerationComponent = (rcCommandInputLeftSliderStickForward) * MAX_BODY_AXIS_POWERED_ACCELERATION_COMMAND_MS;
    //need protections
    
    g2.btolController.setDesiredAccelerationBodyZ(desiredUpAccelerationComponent * -1.0);  //the -1.0 converts this into the +Z = down frame.  this (of course) needs work.
    g2.btolController.setDesiredAccelerationBodyX(desiredForwardAccelerationComponent);  //this (of course) needs work.

    float mtv_direct_command_tilt_angle = 0.0;
    mtv_direct_command_tilt_angle = MTV_MIN_COMMANDABLE_TILT_ANGLE_IN_RADIANS + ((MTV_MAX_COMMANDABLE_TILT_ANGLE_IN_RADIANS-MTV_MIN_COMMANDABLE_TILT_ANGLE_IN_RADIANS) * ((rcCommandInputLeftSliderStickForward + 1.0f) / 2.0f));
    float mtv_direct_command_tilt_acceleration = 0.0;
    mtv_direct_command_tilt_acceleration = MTV_MIN_COMMANDABLE_TILT_ACCELERATION_IN_MSS + ((MTV_MAX_COMMANDABLE_TILT_ACCELERATION_IN_MSS-MTV_MIN_COMMANDABLE_TILT_ACCELERATION_IN_MSS) * ((rcCommandInputThrottleStickForward + 1.0f) / 2.0f));
   
    g2.btolController.setDesiredTiltAngle(mtv_direct_command_tilt_angle);
    g2.btolController.setDesiredAccelerationAlongTiltAngle(mtv_direct_command_tilt_acceleration);

    //calculate pitch atttiude
    g2.btolController.setDesiredPitchAttitude(rcCommandInputPitchStickAft * g2.btolController.getPitchAttitudeCommandGain());
    g2.btolController.setDesiredRollAttitude(rcCommandInputRollStickRight * g2.btolController.getRollAttitudeCommandGain());
    //Plane::btolController.setDesiredYawRate(0.0f);//heading rate...
    
    g2.btolController.setCommandedPitchRate(rcCommandInputPitchStickAft * g2.btolController.getPitchRateCommandGain());
    g2.btolController.setCommandedRollRate(rcCommandInputRollStickRight * g2.btolController.getRollRateCommandGain());
    g2.btolController.setCommandedYawRate(rcCommandInputYawStickRight * g2.btolController.getYawRateCommandGain()); 

    g2.btolController.setDesiredPassthroughAngularAccelerationPitch(rcCommandInputPitchStickAft * g2.btolController.getPitchPassthroughAccelerationCommandGain());
    g2.btolController.setDesiredPassthroughAngularAccelerationRoll(rcCommandInputRollStickRight * g2.btolController.getRollPassthroughAccelerationCommandGain());
    g2.btolController.setDesiredPassthroughAngularAccelerationYaw(rcCommandInputYawStickRight * g2.btolController.getYawPassthroughAccelerationCommandGain()); //TODO: these are temporary gain placeholders.


    //Todo: this should be a state machine.  This is a bit hacky, setting it every cycle.
    if(hal.rcin->read(RC_CHANNEL_NUMBER_FOR_ARM_SWITCH) > 1600 && g2.btolController.getArmedState() != 1)
    {
        g2.btolController.setArmedState(1);
        gcs().send_text(MAV_SEVERITY_CRITICAL, "BTOL ARMED");

    }
    else if(hal.rcin->read(RC_CHANNEL_NUMBER_FOR_ARM_SWITCH) < 1400 && g2.btolController.getArmedState() != 0)
    {
        g2.btolController.setArmedState(0);
        gcs().send_text(MAV_SEVERITY_CRITICAL, "BTOL DISARMED");
    }

    int16_t modeSwitchValue = hal.rcin->read(RC_CHANNEL_NUMBER_FOR_MODE_SWITCH);

    if(modeSwitchValue > 1700 && g2.btolController.getRegulatorModeState() != CONTROLLER_STATE_REGULATOR_MODE_ATTITUDE)
    {
        g2.btolController.setRegulatorModeState(CONTROLLER_STATE_REGULATOR_MODE_ATTITUDE);
        gcs().send_text(MAV_SEVERITY_CRITICAL, "MODE ATTITUDE");
    }
    else if(modeSwitchValue > 1300 && modeSwitchValue < 1700 && g2.btolController.getRegulatorModeState() != CONTROLLER_STATE_REGULATOR_MODE_RATE)
    {
        g2.btolController.setRegulatorModeState(CONTROLLER_STATE_REGULATOR_MODE_RATE);
        gcs().send_text(MAV_SEVERITY_CRITICAL, "MODE RATE");
    }
    else if (modeSwitchValue < 1300 && g2.btolController.getRegulatorModeState() != CONTROLLER_STATE_REGULATOR_MODE_PASSTHROUGH)
    {
        g2.btolController.setRegulatorModeState(CONTROLLER_STATE_REGULATOR_MODE_PASSTHROUGH);
        gcs().send_text(MAV_SEVERITY_CRITICAL, "MODE PASS THROUGH");
        hal.console->printf("Mode Number = %i\n",g2.btolController.getRegulatorModeState());
        
        //https://ardupilot.org/dev/docs/code-overview-adding-a-new-log-message.html
       // AP::logger().Write("BSYS", "TimeUS,Mode", "QI",
       //                                 AP_HAL::micros64(),
        //                                g2.btolController.getRegulatorModeState());
        // Reset the PID filters
       // g2.btolController.get_rate_roll_pid().reset_filter();
       // g2.btolController.get_rate_pitch_pid().reset_filter();
       // g2.btolController.get_rate_yaw_pid().reset_filter();

       // g2.btolController.get_rate_roll_pid().reset_I();
       // g2.btolController.get_rate_pitch_pid().reset_I();
       // g2.btolController.get_rate_yaw_pid().reset_I();
    }


//https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Logger/LogStructure.h#L42
//https://ardupilot.org/dev/docs/code-overview-adding-a-new-log-message.html
    AP::logger().Write("BCMD", "TimeUS,dt_ms,PitchR,RollR,YawR,Tilt,Accel,modeSw,mode,arm",
                   "SSEEE-----", // units: seconds, rad/sec
                   "F000000000", // mult: 1e-6, 1e-2
                   "QIfffffhhh", // format: uint64_t, float
                   AP_HAL::micros64(),
                   dt_ms,
                   (double)rcCommandInputPitchStickAft * g2.btolController.getPitchRateCommandGain(),
                   (double)rcCommandInputRollStickRight * g2.btolController.getRollRateCommandGain(),
                   (double)rcCommandInputYawStickRight * g2.btolController.getYawRateCommandGain(),
                   (double)mtv_direct_command_tilt_angle,
                   (double)mtv_direct_command_tilt_acceleration,
                   modeSwitchValue,
                   g2.btolController.getRegulatorModeState(),
                   g2.btolController.getArmedState()
                   );
    //calculate roll atttiude

    //calculate heading rate

    //calculate desired attiutdes and rates

    //set control parameter targets

   //gcs().send_text(MAV_SEVERITY_INFO, "UPD BTOL");// %5.3f", (double)3.142f);  //Checked, this is getting called.
  //DOESN'T SEEM TO WORK hal.console->printf("RC: %i\n", plane.channel_rudder->get_control_in_zero_dz());
   //Doesn't work printf(" test2\n");
    //doesn't work ::printf(" test\n");
   //hal.console->printf("test4");  //didn't work.
}

void Plane::initialize_btol(){
    hal.rcout->set_default_rate(50);
    //hal.rcout->set_freq(0xF, 400);  //for the first four channels?  0xF = 1111  Testing!  Works!  
    hal.rcout->set_freq(0xFF, 400);  //for the first 8 channels 0xFF = 11111111

  //  hal.rcout->set_output_mode(CH6, MODE_PWM_DSHOT_300)

    //hal.rcout->set_freq(CH6, 400);

    hal.rcout->enable_ch(CH_1);
    hal.rcout->enable_ch(CH_2);
    hal.rcout->enable_ch(CH_3);
    hal.rcout->enable_ch(CH_4);
    hal.rcout->enable_ch(CH_5);
    hal.rcout->enable_ch(CH_6);
    hal.rcout->enable_ch(CH_7);
    hal.rcout->enable_ch(CH_8);

        /*
      output modes. Allows for support of PWM, oneshot and dshot 
     */
    /*enum output_mode {
        MODE_PWM_NONE,
        MODE_PWM_NORMAL,
        MODE_PWM_ONESHOT,
        MODE_PWM_ONESHOT125,
        MODE_PWM_BRUSHED,
        MODE_PWM_DSHOT150,
        MODE_PWM_DSHOT300,
        MODE_PWM_DSHOT600,
        MODE_PWM_DSHOT1200,
        MODE_NEOPIXEL,      // same as MODE_PWM_DSHOT at 800kHz but it's an LED
    };
    virtual void    set_output_mode(uint16_t mask, enum output_mode mode) {}*/

     //   hal.rcout->set_default_rate(50);
   // hal.rcout->set_output_mode(CH_1, 
   hal.console->printf("initialize_btol\n");

   
}

 //TODO: Not actually running at 400!  Running at 50Hz....Need to fix!  Presently running at 400Hz.  See ArduPlane.cpp :   SCHED_TASK(btol_stabilize,         400,   200),  //blake added.
void Plane::btol_stabilize() {   

    //Plane.ahrs.roll_sensor 
    //ahrs.get_roll();  //    // integer Euler angles (Degrees * 100)   //int32_t roll_sensor; 
	static uint32_t _last_t_ms = 0;
    uint32_t tnow_ms = AP_HAL::millis(); //this should be micros if we want to see 400Hz
	uint32_t dt_ms = tnow_ms - _last_t_ms;
	if (_last_t_ms == 0 || dt_ms > 1000) {
		dt_ms = 0;  //presently this is being logged as 20, which means we are running at 50Hz
	}
	_last_t_ms = tnow_ms;

    static uint32_t _last_t_us = 0; //just added equal to zero.

    uint32_t tnow_us = AP_HAL::micros(); //this should be micros if we want to see 400Hz
	uint32_t dt_us = tnow_us - _last_t_us;  //should be 2500 at 400Hz.
	if (_last_t_us == 0 || dt_us > 100000) {
		dt_us = 0;
	}
	_last_t_us = tnow_us;

    static int16_t deltaAmount = 0;
    #define SERVO_CONTROL_VALUE_1_HALF_WIDTH 500
    #define SERVO_CONTROL_CENTER_VALUE 1500
    #define DELTA_VALUE_PER_TIMESTEP 1  //this doesn't seem to be responding at 400Hz which would be center to full range in 1.25 sec.
    static int16_t deltaStep = DELTA_VALUE_PER_TIMESTEP;

    deltaAmount = deltaAmount + deltaStep;
    if(deltaAmount > SERVO_CONTROL_VALUE_1_HALF_WIDTH)
    {
        deltaStep = -DELTA_VALUE_PER_TIMESTEP;
        //servoControlValue1 = -SERVO_CONTROL_VALUE_1_HALF_WIDTH;
    }else if(deltaAmount < -SERVO_CONTROL_VALUE_1_HALF_WIDTH){
        deltaStep = DELTA_VALUE_PER_TIMESTEP;
    }
    //int16_t servoControlValue5 = SERVO_CONTROL_CENTER_VALUE;// + constrain_int16(int16_t(rcCommandInputPitchStickAft*500), -500, 500);  //works (float)

    EffectorList effectorCommands;
    float deltaTimeSecondsForPID = ((float)dt_us)/1000000.0f;
    effectorCommands = g2.btolController.calculateEffectorOutputs(deltaTimeSecondsForPID); //PID_400HZ_DT); //this delta time is wrong, of course!  Should be dynamicly populated.

    //int16_t servoControlValueElevon1 = g2.btolController.calculateServoValueFromAngle(effectorCommands.elevon1Angle, AIRCRAFT_PROPERTIES_ELEVON1_SERVO_MIN_ANGLE, AIRCRAFT_PROPERTIES_ELEVON1_SERVO_MAX_ANGLE, AIRCRAFT_PROPERTIES_ELEVON1_SERVO_MIN_ANGLE_PWM, AIRCRAFT_PROPERTIES_ELEVON1_SERVO_MAX_ANGLE_PWM);
    //int16_t servoControlValueElevon2 = g2.btolController.calculateServoValueFromAngle(effectorCommands.elevon2Angle, AIRCRAFT_PROPERTIES_ELEVON2_SERVO_MIN_ANGLE, AIRCRAFT_PROPERTIES_ELEVON2_SERVO_MAX_ANGLE, AIRCRAFT_PROPERTIES_ELEVON2_SERVO_MIN_ANGLE_PWM, AIRCRAFT_PROPERTIES_ELEVON2_SERVO_MAX_ANGLE_PWM);
    //int16_t servoControlValueTilt1 = g2.btolController.calculateServoValueFromAngle(effectorCommands.tilt1Angle, TILT1_SERVO_MIN_ANGLE, TILT1_SERVO_MAX_ANGLE, TILT1_SERVO_MIN_ANGLE_PWM, TILT1_SERVO_MAX_ANGLE_PWM);
    //int16_t servoControlValueTilt2 = g2.btolController.calculateServoValueFromAngle(effectorCommands.tilt2Angle, TILT2_SERVO_MIN_ANGLE, TILT2_SERVO_MAX_ANGLE, TILT2_SERVO_MIN_ANGLE_PWM, TILT2_SERVO_MAX_ANGLE_PWM);

    //starting out expecting values from 0.0 to +1.0
    //need to convert motor thrust to ESC commands using some scalar...ie: from newtons to scalar 0.0 to 1.0

    //calculate motor thrust vs voltage here...
    //lets start by assuming a linear relationship between voltage and max thrust.
    #define THREE_CELL_BATTERY_VOLTAGE_FULL 12.6f
    //TODO: Consider what happens when we get a bad battery voltage.  Constrain battery voltage input also.  TODO.
    float batteryVoltageRatio = (battery.voltage() / THREE_CELL_BATTERY_VOLTAGE_FULL) * g2.btolController.getBatteryVoltageCompensationCoeficent();
    if(batteryVoltageRatio > 1.0f) batteryVoltageRatio = 1.0f;
    if(batteryVoltageRatio < 0.5f) batteryVoltageRatio = 0.5f; //constrain value
    batteryVoltageRatio = 1.0f; //TODO...put this in here for dev reasions.

    int16_t servoControlValueMotor1 = MOTOR_CONTROL_MIN_VALUE + constrain_int16(int16_t((effectorCommands.motor1Thrust / g2.btolController.getLiftMotorMaxThrust()*batteryVoltageRatio) * MOTOR_CONTROL_RANGE), 0, MOTOR_CONTROL_RANGE); //this needs to be scaled and offset correctly TODO
    int16_t servoControlValueMotor2 = MOTOR_CONTROL_MIN_VALUE + constrain_int16(int16_t((effectorCommands.motor2Thrust / g2.btolController.getLiftMotorMaxThrust()*batteryVoltageRatio)* MOTOR_CONTROL_RANGE), 0, MOTOR_CONTROL_RANGE);
    int16_t servoControlValueMotor3 = MOTOR_CONTROL_MIN_VALUE + constrain_int16(int16_t((effectorCommands.motor3Thrust / g2.btolController.getLiftMotorMaxThrust()*batteryVoltageRatio) * MOTOR_CONTROL_RANGE), 0, MOTOR_CONTROL_RANGE);  //isn't working.  Is stuck at 2000.
    int16_t servoControlValueMotor4 = MOTOR_CONTROL_MIN_VALUE + constrain_int16(int16_t((effectorCommands.motor4Thrust / g2.btolController.getLiftMotorMaxThrust()*batteryVoltageRatio) * MOTOR_CONTROL_RANGE), 0, MOTOR_CONTROL_RANGE); //this needs to be scaled and offset correctly TODO
    int16_t servoControlValueMotor5 = MOTOR_CONTROL_MIN_VALUE + constrain_int16(int16_t((effectorCommands.motor5Thrust / g2.btolController.getLiftMotorMaxThrust()*batteryVoltageRatio)* MOTOR_CONTROL_RANGE), 0, MOTOR_CONTROL_RANGE);
    int16_t servoControlValueMotor6 = MOTOR_CONTROL_MIN_VALUE + constrain_int16(int16_t((effectorCommands.motor6Thrust / g2.btolController.getLiftMotorMaxThrust()*batteryVoltageRatio) * MOTOR_CONTROL_RANGE), 0, MOTOR_CONTROL_RANGE);  //isn't working.  Is stuck at 2000.
    int16_t servoControlValueMotor7 = MOTOR_CONTROL_MIN_VALUE + constrain_int16(int16_t((effectorCommands.motor7Thrust / g2.btolController.getLiftMotorMaxThrust()*batteryVoltageRatio) * MOTOR_CONTROL_RANGE), 0, MOTOR_CONTROL_RANGE); //this needs to be scaled and offset correctly TODO
    int16_t servoControlValueMotor8 = MOTOR_CONTROL_MIN_VALUE + constrain_int16(int16_t((effectorCommands.motor8Thrust / g2.btolController.getLiftMotorMaxThrust()*batteryVoltageRatio)* MOTOR_CONTROL_RANGE), 0, MOTOR_CONTROL_RANGE);
    int16_t servoControlValueMotor9 = MOTOR_CONTROL_MIN_VALUE + constrain_int16(int16_t((effectorCommands.motor9Thrust / g2.btolController.getPropulsionMotorMaxThrust()*batteryVoltageRatio) * MOTOR_CONTROL_RANGE), 0, MOTOR_CONTROL_RANGE);  //isn't working.  Is stuck at 2000.

    //int16_t servoControlValueMotor1 = MOTOR_CONTROL_MIN_VALUE + constrain_int16(int16_t((effectorCommands.motor1Thrust / g2.btolController.getMotor12MaxThrust()) * MOTOR_CONTROL_RANGE), 0, MOTOR_CONTROL_RANGE); //this needs to be scaled and offset correctly TODO
    //int16_t servoControlValueMotor2 = MOTOR_CONTROL_MIN_VALUE + constrain_int16(int16_t((effectorCommands.motor2Thrust / g2.btolController.getMotor12MaxThrust())* MOTOR_CONTROL_RANGE), 0, MOTOR_CONTROL_RANGE);
    //int16_t servoControlValueMotor3 = MOTOR_CONTROL_MIN_VALUE + constrain_int16(int16_t((effectorCommands.motor3Thrust / g2.btolController.getMotor3MaxThrust()) * MOTOR_CONTROL_RANGE), 0, MOTOR_CONTROL_RANGE);  //isn't working.  Is stuck at 2000.

    if(g2.btolController.getArmedState() != 1)
    {
        servoControlValueMotor1 = THROTTLE_DISARMED_VALUE;
        servoControlValueMotor2 = THROTTLE_DISARMED_VALUE;
        servoControlValueMotor3 = THROTTLE_DISARMED_VALUE;
        servoControlValueMotor4 = THROTTLE_DISARMED_VALUE;
        servoControlValueMotor5 = THROTTLE_DISARMED_VALUE;
        servoControlValueMotor6 = THROTTLE_DISARMED_VALUE;
        servoControlValueMotor7 = THROTTLE_DISARMED_VALUE;
        servoControlValueMotor8 = THROTTLE_DISARMED_VALUE;
        servoControlValueMotor9 = THROTTLE_DISARMED_VALUE;
    }

//Need to put more protections!  better than above.
//consider swapping the order so the motors can be in the first 4 if needed to get 400Hz update rate.
    hal.rcout->cork();  //  SRV_Channels::cork();
    hal.rcout->write(CH_1, servoControlValueMotor1);
    hal.rcout->write(CH_2, servoControlValueMotor2);  //not working...perhaps base zero??
    hal.rcout->write(CH_3, servoControlValueMotor3);
    hal.rcout->write(CH_4, servoControlValueMotor4);
    hal.rcout->write(CH_5, servoControlValueMotor5);
    hal.rcout->write(CH_6, servoControlValueMotor6);
    hal.rcout->write(CH_7, servoControlValueMotor7);
    hal.rcout->write(CH_8, servoControlValueMotor8);

     /*   hal.rcout->write(CH_1, servoControlValueElevon1);
    hal.rcout->write(CH_2, servoControlValueElevon2);  //not working...perhaps base zero??
    hal.rcout->write(CH_3, servoControlValueTilt1);
    hal.rcout->write(CH_4, servoControlValueTilt2);
    hal.rcout->write(CH_5, servoControlValue5);
    hal.rcout->write(CH_6, servoControlValueMotor1);
    hal.rcout->write(CH_7, servoControlValueMotor2);
    hal.rcout->write(CH_8, servoControlValueMotor3);*/
    hal.rcout->push();  //will need to use: SRV_Channels::push(); or parts of it when we use BL Heli or D-shot!

    static uint32_t functionTime_us = 0; //this way we include the time it takes to log.

//https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Logger/LogStructure.h#L42
//https://ardupilot.org/dev/docs/code-overview-adding-a-new-log-message.html

    uint64_t timeForLog = AP_HAL::micros64();
    AP::logger().Write("BSTB", "TimeUS,dt_ms,dt_us,ft_us,dt_PID,c1,c2,c3,c4,c5,c6,c7,c8,c9,bVr",
                   "SSSSS----------", // units: seconds, rad/sec
                   "F00000000000000", // mult: 1e-6, 1e-2
                   "QIIIfhhhhhhhhhf", // format: uint64_t, float
                   timeForLog,
                   dt_ms,
                   dt_us,
                   functionTime_us,
                   (double) deltaTimeSecondsForPID,
                   servoControlValueMotor1,
                   servoControlValueMotor2,
                   servoControlValueMotor3,
                   servoControlValueMotor4,
                   servoControlValueMotor5,
                   servoControlValueMotor6,
                   servoControlValueMotor7,
                   servoControlValueMotor8,
                   servoControlValueMotor9,
                   (double)batteryVoltageRatio
                   );

AP::logger().Write("BEFF", "TimeUS,m1,m2,m3,m4,m5,m6,m7,m8,m9mass,VbatR",
                   "S-----------", // units: seconds, rad/sec
                   "F00000000000", // mult: 1e-6, 1e-2
                   "Qfffffffffff", // format: uint64_t, float
                   timeForLog,
                   (double)effectorCommands.motor1Thrust,
                   (double)effectorCommands.motor2Thrust,
                   (double)effectorCommands.motor3Thrust,
                   (double)effectorCommands.motor4Thrust,
                   (double)effectorCommands.motor5Thrust,
                   (double)effectorCommands.motor6Thrust,
                   (double)effectorCommands.motor7Thrust,
                   (double)effectorCommands.motor8Thrust,
                   (double)effectorCommands.motor9Thrust,
                   (double)g2.btolController.getAircraftMass(),
                   (double)batteryVoltageRatio
                    );
//battery.read();

//This is causing issues with the logger, not recording the correct time.  All appearing in one time at 000000
AP::logger().Write("BBAT", "TimeUS,v,v_rest,%",
                   "S---", // units: seconds, rad/sec
                   "F000", // mult: 1e-6, 1e-2
                   "Qfff", // format: uint64_t, float
                   timeForLog,
                   (double)battery.voltage(),
                   (double)battery.voltage_resting_estimate(),
                   (double)battery.capacity_remaining_pct()
                    );
    functionTime_us = AP_HAL::micros() - tnow_us; //this way we include the time it takes to log.
}

float BTOL_Controller::getControlSurfaceForce(float deflectionAngleInRadians, float areaInM2, float dynamicPressureInPa)
{
    float controlSurfaceForce = 0.0;

    //TODO: Improve this!  Using lift equation...or pressure equation?  ....needs improvement!
    float controlSurfaceCl = 0.0;
    controlSurfaceCl = deflectionAngleInRadians * 2.0f; //standin!..need to make this approximation better!  TODO!!
    controlSurfaceForce = dynamicPressureInPa * controlSurfaceCl * areaInM2;

    return controlSurfaceForce;
}

float BTOL_Controller::getEstimatedDynamicPressure(void)
{
    //first, lets look at the vertical component of thrust and infer dynamic pressure.
    float tiltThrustCommandedVerticalAccelerationZ = -1.0f * command.targetTiltAcceleration * sinf(command.targetTiltAngle);
    
    float transitionRatio = getInferredTransitionRatio(tiltThrustCommandedVerticalAccelerationZ, verticalAccelerationThresholdToConsiderAircraftInHover.get());
    float estimatedDynamicPressure = getInferredDynamicPressureFromTransitionRatio(transitionRatio, topOfTransitionDynamicPressure);

    //TODO: filter this!!!

    //If the override value is negative, then do the 
    if(!is_negative(getOverrideDynamicPressureEstimateToThisValueIfPosititive()))
    {
        estimatedDynamicPressure = getOverrideDynamicPressureEstimateToThisValueIfPosititive();
    }
    return estimatedDynamicPressure;
}

float BTOL_Controller::getInferredTransitionRatio(float verticalComponentOfAcceleration, float verticalAccelerationThresholdForHover) //test
{
    //float accelerationDueToGravity = 9.81;
    //using body axis, because that's what the pilot has control over.
    //"UP" direction is negative in the body frame coordinate system.
    float hoverAccelerationDueToGravity = -9.0;//-9.81;
    if(verticalAccelerationThresholdForHover < -2.0f && verticalAccelerationThresholdForHover > -12.0f) //chec for Div 0 and polarity
    {
        hoverAccelerationDueToGravity = verticalAccelerationThresholdForHover;
    }

    float ratioOfThrustToWeight = verticalComponentOfAcceleration / hoverAccelerationDueToGravity;
    constrain_float(ratioOfThrustToWeight, 0.0f, 1.0f);
    //do we want to put a buffer here?  ie: some margin on the hover end?  ie: thrust to 
    //acceleraiton consider us in hover? so we can conver the normal operating case where the thrust is less than the weight but we are in hover?
    float wingLiftToPropThrustRatio = 1.0f - ratioOfThrustToWeight;

    return wingLiftToPropThrustRatio;
}

//should add some hysteresis on the low end...
float BTOL_Controller::getInferredDynamicPressureFromTransitionRatio(float inferredTransitionRatio, float dynamicPressureAtTopOfTransition)
{
    float bottomOfTransitionDynamicPressure = 0;
    float inferredDynamicPressure = 0.0;

    //verticalComponentOfAcceleration = constrain_float(verticalComponentOfAcceleration, 0.0, accelerationDueToGravity);

   // float ratioOfThrustToWeight = verticalComponentOfAcceleration / accelerationDueToGravity;
  //  constrain_float(ratioOfThrustToWeight, 0.0f, 1.0f);
  //  float wingLiftRatio = 1.0f - ratioOfThrustToWeight;

    inferredDynamicPressure = bottomOfTransitionDynamicPressure + inferredTransitionRatio * (dynamicPressureAtTopOfTransition - bottomOfTransitionDynamicPressure);
    //Cap high end of inferred dynamic pressure.
    if(inferredDynamicPressure > dynamicPressureAtTopOfTransition)
    {
        inferredDynamicPressure = dynamicPressureAtTopOfTransition;
    }
    //Cap low end of inferred dynamic pressure.
    if(inferredDynamicPressure < bottomOfTransitionDynamicPressure)
    {
        inferredDynamicPressure = bottomOfTransitionDynamicPressure;
    }

    return inferredDynamicPressure;
}


int16_t BTOL_Controller::getRegulatorModeState(void)
{
    return state.regulatorMode;
}
int16_t BTOL_Controller::setRegulatorModeState(int16_t desiredState)
{
    state.regulatorMode = desiredState;
    return state.regulatorMode;
}

int16_t BTOL_Controller::getArmedState(void)
{
    return state.armedState;
}
int16_t BTOL_Controller::setArmedState(int16_t desiredState)
{
    //Todo: check for validity of input.
    state.armedState = desiredState;
    return state.armedState;
}
void BTOL_Controller::setDesiredPitchAttitude(float pitchAttitudeTarget)
{
    //constrain
    command.targetPitchAttitude = pitchAttitudeTarget;
}
void BTOL_Controller::setDesiredRollAttitude(float rollAttitudeTarget)
{
    //constrain
    command.targetRollAttitude = rollAttitudeTarget;
}
//void BTOL_Controller::setDesiredYawRate(float yawRateTarget)
//{
    //constrain
  //  command.targetYawRate = yawRateTarget;
//}


void BTOL_Controller::setDesiredTiltAngle(float tiltAngle)
{
    command.targetTiltAngle = tiltAngle;
}
void BTOL_Controller::setDesiredAccelerationAlongTiltAngle(float tiltAcceleration)
{
    command.targetTiltAcceleration = tiltAcceleration;
}

void BTOL_Controller::setDesiredAccelerationBodyX(float aX)
{
    command.targetAccelerationX = aX;
}
void BTOL_Controller::setDesiredAccelerationBodyZ(float aZ)
{
    command.targetAccelerationZ = aZ;
}

void BTOL_Controller::setDesiredPassthroughAngularAccelerationRoll(float waX)
{
    command.passthroughAngularAccelerationRoll = waX;
}
void BTOL_Controller::setDesiredPassthroughAngularAccelerationPitch(float waY)
{
    command.passthroughAngularAccelerationPitch = waY;
}
void BTOL_Controller::setDesiredPassthroughAngularAccelerationYaw(float waZ)
{
    command.passthroughAngularAccelerationYaw = waZ;
}

void BTOL_Controller::setCommandedRollRate(float rollRate)
{
    command.targetRollRate = rollRate;
}
void BTOL_Controller::setCommandedPitchRate(float pitchRate)
{
    command.targetPitchRate = pitchRate;
}
void BTOL_Controller::setCommandedYawRate(float yawRate)
{
    command.targetYawRate = yawRate;
}

void BTOL_Controller::setCommandedHoverPitchAttitudeBaseline(float nominalHoverPitchAttitude)
{
    command.targetHoverNominalPitchAttitude = nominalHoverPitchAttitude;
}

// get_filt_alpha - calculate a filter alpha
float BTOL_Controller::getFilterAlpha(float filt_hz, float dt)
{
    if (is_zero(filt_hz)) {
        return 1.0f;
    }

    // calculate alpha
    float rc = 1 / (M_2PI * filt_hz);
    return dt / (dt + rc);
}

int16_t BTOL_Controller::calculateServoValueFromAngle(float desiredAngle, float minimumAngle, float maximumAngle, int16_t minimumPWM, int16_t maximumPWM)
{
    int16_t servoPWMValue = 1500;
    desiredAngle = constrain_float(desiredAngle, minimumAngle, maximumAngle); //we obviously can't command past the min or max, so lets not try!

    float angleRatio = (desiredAngle - minimumAngle) / (maximumAngle - minimumAngle); //This needs to be checked!  TODO!  handle negatives, reversed, etc.
    
    int16_t pwmOutputRange = maximumPWM - minimumPWM; //this could easily be negative if the servo is reversed....it should work out fine, but might want to check.

    int16_t pwmAngleContribution = (int16_t) ((float) pwmOutputRange * angleRatio);

    servoPWMValue = minimumPWM + pwmAngleContribution;

    constrain_int16(servoPWMValue, minimumPWM, maximumPWM); //added protection.  I've added this line but haven't tested yet 2019-12-13 TODO.

    return servoPWMValue;
}

 float BTOL_Controller::calculateMotorThrustBasedOnTiltAngle(float attainedTiltAngle, float desiredForceForward, float desiredForceUp, float satisfactionAngleLow, float satisfactionAngleHigh)
 {
     //protect satisfaction angles.  TODO: this needs to be way better...ie: protect agains overlap.  protect against too high, too low.
    if(satisfactionAngleLow < 0.03f) satisfactionAngleLow = 0.03f;
    if(satisfactionAngleLow > 0.78f) satisfactionAngleLow = 0.78f;

    if(satisfactionAngleHigh > 1.55f) satisfactionAngleHigh = 1.55f;
    if(satisfactionAngleHigh < 0.79f) satisfactionAngleHigh = 0.79f;


    float motorForceDemand;
//todo: make this into a function!
        if(attainedTiltAngle < satisfactionAngleLow)
        {
            //Lower Range
            motorForceDemand = desiredForceForward / cosf(attainedTiltAngle); //don;t div/0!  TODO!
        }else if (attainedTiltAngle < satisfactionAngleHigh){
            //Middle Range
            //linear blend?  what type of blend?
            float satisfactionRatio = 0.0;
            float satisfactionValueX = desiredForceForward / cosf(attainedTiltAngle); //don;t div/0!  TODO!
            float satisfactionValueY = desiredForceUp / sinf(attainedTiltAngle); //don;t div/0!  TODO!
            float middleRange = satisfactionAngleHigh - satisfactionAngleLow;

            satisfactionRatio = (attainedTiltAngle - satisfactionAngleLow) / middleRange; //TODO: div/0 protection!
            motorForceDemand = satisfactionRatio * satisfactionValueX + (1.0f - satisfactionRatio) * satisfactionValueY;

        }else{
            //Upper Range
            motorForceDemand = desiredForceUp / sinf(attainedTiltAngle); //don't div/0 TODO!
        }

    //This should also output the residual!
    return motorForceDemand;
 }

float BTOL_Controller::getAugmentedStabilityRatioWithinAngleRange(float angleRad, float innerAngleRadPositive, float outerAngleRadPositive)
{
    //TODO: check to make sure inner and outer angles are valid and not equal.
    //TODO: check for the inner angle = zero case.
    //TODO: guard against bad inputs.
    #define MIN_ANGLE_SPREAD_TOLERENCE_FOR_STABILITY_ZONE 0.1 //6 degrees.
    if(innerAngleRadPositive > (outerAngleRadPositive - MIN_ANGLE_SPREAD_TOLERENCE_FOR_STABILITY_ZONE)) return 0; //don't do calculations if the angle spread is too small!

    //provie a restoring response if the value is within the range, positive if in a negative area, and negative if in a positive area.  
    //This will allow us to talor the stability zones based on the phase of flight.
    float reactiveAngleRangeMagnitude = outerAngleRadPositive - innerAngleRadPositive;
    float angleMagnitude = fabs(angleRad);
    float angleRatio = 0.0;
    if(angleMagnitude <= innerAngleRadPositive)
    {
        //the angle is between the +- inner angles, so no stabilization.
        //return 0;
    }else{
        //angle is greater than innerAngle
        angleRatio = (angleMagnitude - innerAngleRadPositive)/(reactiveAngleRangeMagnitude);
        
        if(angleRad > 0.0f)
        {
            angleRatio = -angleRatio; //provied the restoring ratio.
        }
    }

    //between the +-inner angle? (might be zero!)
    //between the +-inner and outer angles
    //beyond the outer angles. (cap?)

    return angleRatio;
}

//TODO: Look up table function,

float BTOL_Controller::getRangeRatio(float value, float min, float max)  //TODO: Allow max to be less than min...this would allow us to invert the output.
{
    float ratio = 0.0;
    if(max < min)
    {
        //throw error.
        return 0.0f;
    }

    if(value<min)
    {
        ratio = 0.0f;
    }else if(value>max){
        ratio = 1.0f;
    }else{
        ratio = (value-min)/(max-min);
    }

    return ratio;
}

float BTOL_Controller::yawRateRegulator(float targetRate, float measuredRate, float dynamicPressure, float trueAirspeed, float deltaTime)
{
    //Sanitize inputs.  TODO.

    float torqueDemand = 0.0f;
    //get the coeficents (some of them scale with dynamic pressure or true arispeed)
    float dynamicPressureRatio = getRangeRatio(dynamicPressure, 0.0f, getTopOfTransitionDynamicPressure());  //this should be scaled with dynamic pressure, not capped...but we'll start here.
    float proportionalCoef = YawRegulatorPtermHover + dynamicPressureRatio * (YawRegulatorPtermForwardFlight - YawRegulatorPtermHover);
    float integralCoef = YawRegulatorItermHover + dynamicPressureRatio * (YawRegulatorItermForwardFlight - YawRegulatorItermHover);
    float derivitiveCoef = YawRegulatorDtermHover + dynamicPressureRatio * (YawRegulatorDtermForwardFlight - YawRegulatorDtermHover);
    float integratorMax = YawRegulatorItermMaxHover + dynamicPressureRatio * (YawRegulatorItermMaxForwardFlight - YawRegulatorItermMaxHover); //This is what's breaking it.

    //look up the aeroRateDampingCoeficent 
    float aeroRateDampingCoeficent = 0.0f; //this will be a negative term.
    aeroRateDampingCoeficent = aeroDampingBaselineHoverYaw + aeroDampingVsTrueAirspeedCoefYaw * trueAirspeed;

    //float momentOfInertia = aircraftProperties.momentOfInertiaYaw;
    float momentOfInertia = getAircraftMomentOfInertiaInYaw();

    //get the torque demand.
    torqueDemand = _regulatorYaw.getTorqueDemand(targetRate,measuredRate,deltaTime,proportionalCoef,integralCoef,derivitiveCoef,integratorMax,aeroRateDampingCoeficent,momentOfInertia);
    return torqueDemand;
}

float BTOL_Controller::rollRateRegulator(float targetRate, float measuredRate, float dynamicPressure, float trueAirspeed, float deltaTime)
{
    //Sanitize inputs.  TODO.

    float torqueDemand = 0.0f;
    //get the coeficents (some of them scale with dynamic pressure or true arispeed)
    float dynamicPressureRatio = getRangeRatio(dynamicPressure, 0.0f, getTopOfTransitionDynamicPressure());  //this should be scaled with dynamic pressure, not capped...but we'll start here.
    float proportionalCoef = RollRegulatorPtermHover + dynamicPressureRatio * (RollRegulatorPtermForwardFlight - RollRegulatorPtermHover);
    float integralCoef = RollRegulatorItermHover + dynamicPressureRatio * (RollRegulatorItermForwardFlight - RollRegulatorItermHover);
    float derivitiveCoef = RollRegulatorDtermHover + dynamicPressureRatio * (RollRegulatorDtermForwardFlight - RollRegulatorDtermHover);
    float integratorMax = RollRegulatorItermMaxHover + dynamicPressureRatio * (RollRegulatorItermMaxForwardFlight - RollRegulatorItermMaxHover); //This is what's breaking it.

    //look up the aeroRateDampingCoeficent 
    float aeroRateDampingCoeficent = 0.0f; //this will be a negative term.
    aeroRateDampingCoeficent = aeroDampingBaselineHoverRoll + aeroDampingVsTrueAirspeedCoefRoll * trueAirspeed;

    //float momentOfInertia = aircraftProperties.momentOfInertiaRoll;
    float momentOfInertia = getAircraftMomentOfInertiaInRoll();

    //get the torque demand.
    torqueDemand = _regulatorRoll.getTorqueDemand(targetRate,measuredRate,deltaTime,proportionalCoef,integralCoef,derivitiveCoef,integratorMax,aeroRateDampingCoeficent,momentOfInertia);
    return torqueDemand;
}

float BTOL_Controller::pitchRateRegulator(float targetRate, float measuredRate, float dynamicPressure, float trueAirspeed, float deltaTime)
{
    //Sanitize inputs.  TODO.

    float torqueDemand = 0.0f;
    //get the coeficents (some of them scale with dynamic pressure or true arispeed)
    float dynamicPressureRatio = getRangeRatio(dynamicPressure, 0.0f, getTopOfTransitionDynamicPressure());  //this should be scaled with dynamic pressure, not capped...but we'll start here.
    float proportionalCoef = PitchRegulatorPtermHover + dynamicPressureRatio * (PitchRegulatorPtermForwardFlight - PitchRegulatorPtermHover);
    float integralCoef = PitchRegulatorItermHover + dynamicPressureRatio * (PitchRegulatorItermForwardFlight - PitchRegulatorItermHover);
    float derivitiveCoef = PitchRegulatorDtermHover + dynamicPressureRatio * (PitchRegulatorDtermForwardFlight - PitchRegulatorDtermHover);
    float integratorMax = PitchRegulatorItermMaxHover + dynamicPressureRatio * (PitchRegulatorItermMaxForwardFlight - PitchRegulatorItermMaxHover); //This is what's breaking it.

    //look up the aeroRateDampingCoeficent 
    float aeroRateDampingCoeficent = 0.0f; //this will be a negative term.
    aeroRateDampingCoeficent = aeroDampingBaselineHoverPitch + aeroDampingVsTrueAirspeedCoefPitch * trueAirspeed;

    //float momentOfInertia = aircraftProperties.momentOfInertiaPitch;
    float momentOfInertia = getAircraftMomentOfInertiaInPitch();

    //get the torque demand.
    torqueDemand = _regulatorPitch.getTorqueDemand(targetRate,measuredRate,deltaTime,proportionalCoef,integralCoef,derivitiveCoef,integratorMax,aeroRateDampingCoeficent,momentOfInertia);
    return torqueDemand;
}

void BTOL_Controller::updateSensorData(void)
{

    //const AP_InertialSensor &_ins = AP::ins();
    
    //Vector3f initAccVec = _ins.get_accel();


    
    //AHRS defined in the BTOL Class.
    //const  AP_InertialNav_NavEKF &_inertial_nav = AP::  inertial_nav.get_velocity().z;
    
    //_ahrs.get_roll();

    //consider using delta-velocity ?  Does 
    //From throw function.
     //       const float velocity = inertial_nav.get_velocity().length();
       // const float velocity_z = inertial_nav.get_velocity().z;
       // const float accel = copter.ins.get_accel().length();
       // const float ef_accel_z = ahrs.get_accel_ef().z;


    //accels
    //gyros
    //attitude
    //heaing
    //baro
    //validity
        //      by using get_delta_velocity() instead of get_accel() the
        //      accel value is sampled over the right time delta for
         //     each sensor, which prevents an aliasing effect.  Does this take into acount gravity?  I don't think so!
    


    //log the values
}
void BTOL_Controller::updateAdhrsEstimate(void)
{
    const AP_InertialSensor &_ins = AP::ins();
    
    Vector3f rawInitAccVec;
    rawInitAccVec = _ins.get_accel(); //I think this is wrong.  Needs an update function?  Should likely subscribe in the normal way...same as done with baro!
    //how are these acceleratons defined?  
    estimate.attitudePitch = _ahrs.get_pitch();
    estimate.attitudeRoll = _ahrs.get_roll();
    estimate.bodyProperAccelerationX = rawInitAccVec.x;
    estimate.bodyProperAccelerationY = rawInitAccVec.y;
    estimate.bodyProperAccelerationZ = rawInitAccVec.z; //will read -9.81 due to the proper acceleration due to the force of the ground pushing it.
    estimate.heading = _ahrs.get_yaw();
    estimate.ratePitch = _ahrs.get_gyro().y;
    estimate.rateRoll = _ahrs.get_gyro().x;
    estimate.rateYaw = _ahrs.get_gyro().z;
    estimate.dynamicPressure = 0.0f;
    rawInitAccVec = rawInitAccVec * 1.0f;  //to make compile warning go away.  (unused)

    AP::logger().Write("BEST", "TimeUS,q,Rx,Ry,Rz,P,R,Y,Ax,Ay,Az,Yre,AOA,AOS,Vgx,Vgy,init",
                "S----------------", // units: seconds, any
                "F0000000000000000", // mult: 1e-6, 1e-2
                "Qffffffffffffffff", // format: uint64_t, float
                AP_HAL::micros64(),
                (double)estimate.dynamicPressure,
                (double)estimate.rateRoll,
                (double)estimate.ratePitch,
                (double)estimate.rateYaw ,
                (double)estimate.attitudePitch,
                (double)estimate.attitudeRoll,
                (double)estimate.heading,
                (double)estimate.bodyProperAccelerationX,
                (double)estimate.bodyProperAccelerationY,
                (double)estimate.bodyProperAccelerationZ,
                (double)_ahrs.get_yaw_rate_earth(),
                (double)_ahrs.getAOA(),
                (double)_ahrs.getSSA(),
                //(double)_ahrs.get_airspeed()
                (double)_ahrs.groundspeed_vector().x,
                (double)_ahrs.groundspeed_vector().y,
                (double)_ahrs.initialised()
                );

//_ahrs.get_gyro().z



//Attiude
//Rates
//Accels
//Airspeed
//dynamic pressure
//AOA
//AOS

}


EffectorList BTOL_Controller::calculateEffectorOutputs(float dt)
{
    updateSensorData();
    updateAdhrsEstimate();

    float dynamicPressure = getEstimatedDynamicPressure();
    //Add transition ratio.  Consider mutiplying q by 1/100 or acceleration by 10 and radians by 100
    AP::logger().Write("BTOL", "TimeUS,q,cmdTilt,cmdAccel",
                "S-ro", // units: seconds, rad/sec
                "F000", // mult: 1e-6, 1e-2
                "Qfff", // format: uint64_t, float
                AP_HAL::micros64(),
                (double)dynamicPressure,
                (double)command.targetTiltAngle,
                (double)command.targetTiltAcceleration
                );

    //Get the output moment from the regulator, depending on the control mode.
    float desiredMomentX = 0.0f;
    float desiredMomentY = 0.0f;
    float desiredMomentZ = 0.0f;

    //Get the target rates from the pilot commands or stabilization system, depending on the control mode.
    float targetYawRate = 0.0f;
    float targetPitchRate = 0.0f;
    float targetRollRate = 0.0f;

    if(state.regulatorMode == CONTROLLER_STATE_REGULATOR_MODE_ATTITUDE)
    {
      //  Attitude Command ...will be useful later...need to figure out how to get both!

       /* float dynamicPressureRatio = getRangeRatio(dynamicPressure, 0.0f, getTopOfTransitionDynamicPressure());
        #define FORWARD_FLIGHT_ROLL_INNER_ANGLE 0.698132f //40 degrees
        #define FORWARD_FLIGHT_ROLL_OUTER_ANGLE 1.5708f   //90 degrees
        #define HOVER_FLIGHT_ROLL_ANGLE_OUTER 1.0f //57 degrees
        float innerStabilizationAngleRoll = dynamicPressureRatio * FORWARD_FLIGHT_ROLL_INNER_ANGLE;
        float outerStabilizationAngleRoll = dynamicPressureRatio * (FORWARD_FLIGHT_ROLL_OUTER_ANGLE - HOVER_FLIGHT_ROLL_ANGLE_OUTER) + HOVER_FLIGHT_ROLL_ANGLE_OUTER;

        #define FORWARD_FLIGHT_PITCH_INNER_ANGLE 0.698132f //40 degrees
        #define FORWARD_FLIGHT_PITCH_OUTER_ANGLE 1.5708f //90 degrees
        #define HOVER_FLIGHT_PITCH_ANGLE_OUTER 1.0f //57 degrees
        float innerStabilizationAnglePitch = dynamicPressureRatio * FORWARD_FLIGHT_PITCH_INNER_ANGLE;
        float outerStabilizationAnglePitch = dynamicPressureRatio * (FORWARD_FLIGHT_PITCH_OUTER_ANGLE - HOVER_FLIGHT_PITCH_ANGLE_OUTER) + HOVER_FLIGHT_PITCH_ANGLE_OUTER;

        targetRollRate = command.targetRollRate + getAugmentedStabilityRatioWithinAngleRange(estimate.attitudeRoll, innerStabilizationAngleRoll, outerStabilizationAngleRoll) * getRollRateCommandGain();
        targetPitchRate = command.targetPitchRate + getAugmentedStabilityRatioWithinAngleRange(estimate.attitudePitch, innerStabilizationAnglePitch, outerStabilizationAnglePitch) * getPitchRateCommandGain();
        targetYawRate = command.targetYawRate; //can add auto coordination here.... 
      */
        //need regulator gains.
        //Blend in attitude control or fight the roll rate target?  
        //float rollAttitudeFeedbackRatio = 1.0f - getRangeRatio(dynamicPressure, 0.0f, getTopOfAttitudeFeedbackDynamicPressure());  //this should be scaled with dynamic pressure, not capped...but we'll start here.
        //float hoverRollAttitudeSetpoint = 0.0f;

        /*//add automatic hover roll trim here:
        float rollAttitudeToCompensateForLateralAcceleration = 0.0f;

        //calculate the automatic roll trim.
        rollAttitudeToCompensateForLateralAcceleration = -1.0f * asinf(estimate.bodyProperAccelerationY / 9.81);  //we could use the zaccleration here, but we know what it will be!, ALthough, in hover, that's a much better way to estimate thrust!
        //scale output for so that it only occurs at low dynamic pressures?
        float automaticHoverRollTrimStrengthRatio = 1.0f - getRangeRatio(dynamicPressure, 0.0f, automaticHoverRollTrimDynamicPressureMax);
        rollAttitudeToCompensateForLateralAcceleration =  rollAttitudeToCompensateForLateralAcceleration * automaticHoverRollTrimStrengthRatio;
        //cap output
        float automaticHoverRollTrimMaximumAngle = 10 * DEG_TO_RAD;
        if(rollAttitudeToCompensateForLateralAcceleration > automaticHoverRollTrimMaximumAngle) rollAttitudeToCompensateForLateralAcceleration = automaticHoverRollTrimMaximumAngle;
        if(rollAttitudeToCompensateForLateralAcceleration < -automaticHoverRollTrimMaximumAngle) rollAttitudeToCompensateForLateralAcceleration = -automaticHoverRollTrimMaximumAngle;
        hoverRollAttitudeSetpoint = hoverRollAttitudeSetpoint + rollAttitudeToCompensateForLateralAcceleration;
        */

      /*AP::logger().Write("BHRT", "TimeUS,est_q,ratio,Ay,RollAtti,RollSetpoint",
        "S-----", // units: seconds, any
        "F00000", // mult: 1e-6, 1e-2
        "Qfffff", // format: uint64_t, float
        AP_HAL::micros64(),
        (double)dynamicPressure,
        (double)automaticHoverRollTrimStrengthRatio,
        (double)estimate.bodyProperAccelerationY,
        (double)rollAttitudeToCompensateForLateralAcceleration,
        (double)hoverRollAttitudeSetpoint
        );
        */
        //rollAttitudeFeedbackRatio = 1.0f;
        float rollAttitudeFeedbackRatio = 1.0f;// - getRangeRatio(dynamicPressure, 0.0f, getTopOfAttitudeFeedbackDynamicPressure());  //this should be scaled with dynamic pressure, not capped...but we'll start here.
        float hoverRollAttitudeSetpoint = 0.0f;

        #define MAX_ROLL_ATTITUDE_HOVER 1.0f //57 degrees
        targetRollRate = command.targetRollRate + rollAttitudeFeedbackRatio * ((hoverRollAttitudeSetpoint - estimate.attitudeRoll)/MAX_ROLL_ATTITUDE_HOVER) * getRollRateCommandGain();
        
        //float pitchAttitudeFeedbackRatio = 1.0f - getRangeRatio(dynamicPressure, 0.0f, getTopOfAttitudeFeedbackDynamicPressure());  //this should be scaled with dynamic pressure, not capped...but we'll start here.
        float pitchAttitudeFeedbackRatio = 1.0f;
        float hoverPitchAttitudeSetpoint = 0.0f;
        hoverPitchAttitudeSetpoint = command.targetHoverNominalPitchAttitude; //so we can set the deck angle with a slider, or other input, etc....
        #define MAX_PITCH_ATTITUDE_HOVER 1.0f //57 degrees
        targetPitchRate = command.targetPitchRate + pitchAttitudeFeedbackRatio * ((hoverPitchAttitudeSetpoint - estimate.attitudePitch)/MAX_PITCH_ATTITUDE_HOVER) * getPitchRateCommandGain();
        
        targetYawRate = command.targetYawRate; //can add auto coordination here.... 
    }

    if(state.regulatorMode == CONTROLLER_STATE_REGULATOR_MODE_RATE)
    {
        targetPitchRate = command.targetPitchRate;
        targetRollRate = command.targetRollRate;
        targetYawRate = command.targetYawRate;
    }

        //overwrite desire moments if in passthrough test mode.
    if(state.regulatorMode == CONTROLLER_STATE_REGULATOR_MODE_PASSTHROUGH)
    {
        desiredMomentX = command.passthroughAngularAccelerationRoll * aircraftProperties.momentOfInertiaRoll; //passthrough to start to help build effector blender.
        desiredMomentY = command.passthroughAngularAccelerationPitch * aircraftProperties.momentOfInertiaPitch;
        desiredMomentZ = command.passthroughAngularAccelerationYaw * aircraftProperties.momentOfInertiaYaw;
        //acceleration * moment of inertia
    }else{
        //handle non-passthrough regulation.
        desiredMomentZ = yawRateRegulator(targetYawRate, estimate.rateYaw, dynamicPressure, sqrtf(dynamicPressure), dt);
        desiredMomentY = pitchRateRegulator(targetPitchRate, estimate.ratePitch, dynamicPressure, sqrtf(dynamicPressure), dt);
        desiredMomentX = rollRateRegulator(targetRollRate, estimate.rateRoll, dynamicPressure, sqrtf(dynamicPressure), dt);

        //also can get earth frame acceleration vectors:
        //_ahrs.get_accel_ef();

        //https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_AHRS/AP_AHRS_DCM.cpp

        /*AP_InertialSensor &_ins = AP::ins();

        // Get body frame accel vector
        Vector3f initAccVec = _ins.get_accel();
        uint8_t counter = 0;

        _ins.get_accel();
        _ins.get_gyro();


        // the first vector may be invalid as the filter starts up
        while ((initAccVec.length() < 9.0f || initAccVec.length() > 11) && counter++ < 20) {
            _ins.wait_for_sample();
            _ins.update();
            initAccVec = _ins.get_accel();
        }
        */

//Log things such as vertical rate, attitude, ext.

AP::logger().Write("BARO", "TimeUS,est_q,bVS,bALT",
                "S---", // units: seconds, any
                "F000", // mult: 1e-6, 1e-2
                "Qfff", // format: uint64_t, float
                AP_HAL::micros64(),
                (double)dynamicPressure,
                //(double)_ahrs.get_airspeed(),
                (double)_baro.get_climb_rate(),
                (double)_baro.get_altitude()
                );

    AP::logger().Write("BREP", "TimeUS,q,t,es,E,cP,cI,cD,P,I,D,rA,rT,cRD,ffT,rtT,vI,mI",
                "S----------------", // units: seconds, any
                "F0000000000000000", // mult: 1e-6, 1e-2
                "Qffffffffffffffff", // format: uint64_t, float
                AP_HAL::micros64(),
                (double)dynamicPressure,
                (double)_regulatorPitch._lastRegulatorTarget,
                (double)_regulatorPitch._lastRegulatorEstimate,
                (double)_regulatorPitch._lastRegulatorError,
                (double)_regulatorPitch._lastRegulatorPcoef,
                (double)_regulatorPitch._lastRegulatorIcoef,
                (double)_regulatorPitch._lastRegulatorDcoef,
                (double)_regulatorPitch._lastRegulatorP,
                (double)_regulatorPitch._lastRegulatorI,
                (double)_regulatorPitch._lastRegulatorD,
                (double)_regulatorPitch._lastRegulatorAccelerationContribution,
                (double)_regulatorPitch._lastRegulatorTorqueContribution,
                (double)_regulatorPitch._lastRegulatorRateDampingCoef,
                (double)_regulatorPitch._lastRegulatorFFTorqueDemand,
                (double)_regulatorPitch._lastOutputTotalTorqueDemand,
                (double)_regulatorPitch._lastRegulatorIntegralValue,
                (double)_regulatorPitch._lastRegulatorIntegralValueMax
                );

    AP::logger().Write("BRER", "TimeUS,q,t,es,E,cP,cI,cD,P,I,D,rA,rT,cRD,ffT,rtT,vI,mI",
                "S----------------", // units: seconds, any
                "F0000000000000000", // mult: 1e-6, 1e-2
                "Qffffffffffffffff", // format: uint64_t, float
                AP_HAL::micros64(),
                (double)dynamicPressure,
                (double)_regulatorRoll._lastRegulatorTarget,
                (double)_regulatorRoll._lastRegulatorEstimate,
                (double)_regulatorRoll._lastRegulatorError,
                (double)_regulatorRoll._lastRegulatorPcoef,
                (double)_regulatorRoll._lastRegulatorIcoef,
                (double)_regulatorRoll._lastRegulatorDcoef,
                (double)_regulatorRoll._lastRegulatorP,
                (double)_regulatorRoll._lastRegulatorI,
                (double)_regulatorRoll._lastRegulatorD,
                (double)_regulatorRoll._lastRegulatorAccelerationContribution,
                (double)_regulatorRoll._lastRegulatorTorqueContribution,
                (double)_regulatorRoll._lastRegulatorRateDampingCoef,
                (double)_regulatorRoll._lastRegulatorFFTorqueDemand,
                (double)_regulatorRoll._lastOutputTotalTorqueDemand,
                (double)_regulatorRoll._lastRegulatorIntegralValue,
                (double)_regulatorRoll._lastRegulatorIntegralValueMax
                );

    AP::logger().Write("BREY", "TimeUS,q,t,es,E,cP,cI,cD,P,I,D,rA,rT,cRD,ffT,rtT,vI,mI",
                "S----------------", // units: seconds, any
                "F0000000000000000", // mult: 1e-6, 1e-2
                "Qffffffffffffffff", // format: uint64_t, float
                AP_HAL::micros64(),
                (double)dynamicPressure,
                (double)_regulatorYaw._lastRegulatorTarget,
                (double)_regulatorYaw._lastRegulatorEstimate,
                (double)_regulatorYaw._lastRegulatorError,
                (double)_regulatorYaw._lastRegulatorPcoef,
                (double)_regulatorYaw._lastRegulatorIcoef,
                (double)_regulatorYaw._lastRegulatorDcoef,
                (double)_regulatorYaw._lastRegulatorP,
                (double)_regulatorYaw._lastRegulatorI,
                (double)_regulatorYaw._lastRegulatorD,
                (double)_regulatorYaw._lastRegulatorAccelerationContribution,
                (double)_regulatorYaw._lastRegulatorTorqueContribution,
                (double)_regulatorYaw._lastRegulatorRateDampingCoef,
                (double)_regulatorYaw._lastRegulatorFFTorqueDemand,
                (double)_regulatorYaw._lastOutputTotalTorqueDemand,
                (double)_regulatorYaw._lastRegulatorIntegralValue,
                (double)_regulatorYaw._lastRegulatorIntegralValueMax
                );
        
    }

    //lowpass filter the desired moments.
    static float filteredDesiredMomentX = 0.0;
    filteredDesiredMomentX = filteredDesiredMomentX + getFilterAlpha(lowpassFilterCuttofFrequencyRoll, dt)  * (desiredMomentX - filteredDesiredMomentX);
    static float filteredDesiredMomentY = 0.0;
    filteredDesiredMomentY = filteredDesiredMomentY + getFilterAlpha(lowpassFilterCuttofFrequencyPitch, dt)  * (desiredMomentY - filteredDesiredMomentY);
    static float filteredDesiredMomentZ = 0.0;
    filteredDesiredMomentZ = filteredDesiredMomentZ + getFilterAlpha(lowpassFilterCuttofFrequencyYaw, dt)  * (desiredMomentZ - filteredDesiredMomentZ);


    AP::logger().Write("BFIL", "TimeUS,dt,mX,mfX,cfX,mY,mfY,cfY,mZ,mfZ,cfZ",
                "S----------", // units: seconds, rad/sec
                "F0000000000", // mult: 1e-6, 1e-2
                "Qffffffffff", // format: uint64_t, float
                AP_HAL::micros64(),
                (double)dt,
                (double)desiredMomentX,
                (double)filteredDesiredMomentX,
                (double)lowpassFilterCuttofFrequencyRoll,
                (double)desiredMomentY,
                (double)filteredDesiredMomentY,
                (double)lowpassFilterCuttofFrequencyPitch,
                (double)desiredMomentZ,
                (double)filteredDesiredMomentZ,
                (double)lowpassFilterCuttofFrequencyYaw
                );


    //overwrite non-filtered moments with the filtered moments.
    desiredMomentX = filteredDesiredMomentX;
    desiredMomentY = filteredDesiredMomentY;
    desiredMomentZ = filteredDesiredMomentZ;


    //Log the PID values.
    //not sure what to put in the identifier enum  just going to try using the default ones...because the defaults should't be running, because I disabled them.
   // AP::logger().Write_PID(LOG_PIDR_MSG, get_rate_roll_pid().get_pid_info());
   // AP::logger().Write_PID(LOG_PIDP_MSG, get_rate_pitch_pid().get_pid_info());
   // AP::logger().Write_PID(LOG_PIDY_MSG, get_rate_yaw_pid().get_pid_info());



        

    //add the torque from motor3 (assuming motors 1 and 2 cancel out!)
    float estimatedNetMotorTorqueToCancelOut = 0.0;
    estimatedNetMotorTorqueToCancelOut = effectors.motor3Thrust * motor3ThrustToTorqueCoef;
    desiredMomentZ = desiredMomentZ - estimatedNetMotorTorqueToCancelOut;

        
    //calculate desired forces
    float desiredAccelerationZ = 0.0f;
          desiredAccelerationZ = command.targetAccelerationZ;
    float requiredForceZ = desiredAccelerationZ * getAircraftMass();//aircraftProperties.totalMass; //Newtons

    //calculate effector outputs

    //calculate effector mixing
        float pitchMomentForMotors = 0.0f;
        float rollMomentForMotors = 0.0f;
        float yawMomentForMotors = 0.0f;

        pitchMomentForMotors = desiredMomentY;
        rollMomentForMotors = desiredMomentX;
        yawMomentForMotors = desiredMomentZ;

        desiredMomentX = rollMomentForMotors; //so we don't need to change the code below.
        desiredMomentY = pitchMomentForMotors;
        desiredMomentZ = yawMomentForMotors;

        float liftMotorThrustDemands[8] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};

        float zForceToLiftMotorThrustMatrix[8] = {-0.125f,-0.125f,-0.125f,-0.125f,-0.125f,-0.125f,-0.125f,-0.125f};
        float xMomentToLiftMotorThrustMatrix[8] = {0.434848f, 0.434848f, 0.176475f, 0.176475f, -0.176475f, -0.176475f, -0.434848f, -0.434848f};
        float yMomentToLiftMotorThrustMatrix[8] = {-1.77168f, -0.590514f, 0.590514f, 1.77168f, 1.77168f, 0.590514f, -0.590514f, -1.77168f};
        float zMomentToLiftMotorThrustMatrix[8] = {-0.0931148f, 0.156885f, -0.11206f, 0.13794f, -0.13794f, 0.11206f, -0.156885f, 0.0931148f};

        for(int i = 0; i < 8; i++)
        {
            liftMotorThrustDemands[i] = liftMotorThrustDemands[i]
             + zForceToLiftMotorThrustMatrix[i]*requiredForceZ
             + xMomentToLiftMotorThrustMatrix[i]*desiredMomentX
             + yMomentToLiftMotorThrustMatrix[i]*desiredMomentY
             + zMomentToLiftMotorThrustMatrix[i]*desiredMomentZ
            ;

        }

        effectors.motor1Thrust = liftMotorThrustDemands[0];
        effectors.motor2Thrust = liftMotorThrustDemands[1];
        effectors.motor3Thrust = liftMotorThrustDemands[2];
        effectors.motor4Thrust = liftMotorThrustDemands[3];
        effectors.motor5Thrust = liftMotorThrustDemands[4];
        effectors.motor6Thrust = liftMotorThrustDemands[5];
        effectors.motor7Thrust = liftMotorThrustDemands[6];
        effectors.motor8Thrust = liftMotorThrustDemands[7];
        effectors.motor9Thrust = 0.0f;
        

    return effectors;
}

#endif //BTOL_ENABLED
