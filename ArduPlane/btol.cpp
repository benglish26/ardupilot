#include "Plane.h"
#include "config.h"
#if BTOL_ENABLED == ENABLED
#include "btol.h"
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

#define MOTOR_12_DEFAULT_MAX_THRUST_N 8.0f//7.0f//8.0
#define MOTOR_3_DEFAULT_MAX_THRUST_N 3.5f//3.0f
#define TOP_OF_TRANSITION_DEFAULT_DYNAMIC_PRESSURE 200.0f //N/m^2 or Pa
#define DEFAULT_VERTICAL_ACCELERATION_THRESHOLD_TO_CONSIDER_AIRCRAFT_IN_HOVER -8.0f //m/s/s
#define DEFAULT_AIRCRAFT_MASS_IN_KG 0.950f

#define MOTOR_CONTROL_MIN_VALUE 1000
#define MOTOR_CONTROL_MAX_VALUE 2000
#define MOTOR_CONTROL_RANGE (MOTOR_CONTROL_MAX_VALUE-MOTOR_CONTROL_MIN_VALUE)
#define DEFAULT_THRUST_MAX_MOTORS_12 8.0f //Newtons
#define DEFAULT_THRUST_MAX_MOTOR_3 3.5f //Newtons
#define DEFAULT_AIRCRAFT_CENTER_OF_MASS_METERS -0.053f //Meters
#define DEFAULT_MOTOR3_THRUST_TO_TORQUE_COEF 0.025 //NM per N thrust.

const AP_Param::GroupInfo BTOL_Controller::var_info[] = {
	    // parameters from parent vehicle


    AP_SUBGROUPINFO(_pid_rate_roll, "B_ROLL_", 0, BTOL_Controller, AC_PID),
    AP_SUBGROUPINFO(_pid_rate_pitch, "B_PTCH_", 1, BTOL_Controller, AC_PID),
    AP_SUBGROUPINFO(_pid_rate_yaw, "B_YAW_", 2, BTOL_Controller, AC_PID),
      
    // @Param: TCONST
	// @DisplayName: Roll Time Constant
	// @Description: Time constant in seconds from demanded to achieved roll angle. Most models respond well to 0.5. May be reduced for faster responses, but setting lower than a model can achieve will not help.
	// @Range: 0.4 1.0
	// @Units: s
	// @Increment: 0.1
	// @User: Advanced
	AP_GROUPINFO("RRATECMDG",      3, BTOL_Controller, rollRateCommandGain,       0.5f),

	// @Param: P
	// @DisplayName: Proportional Gain
	// @Description: Proportional gain from roll angle demands to ailerons. Higher values allow more servo response but can cause oscillations. Automatically set and adjusted by AUTOTUNE mode.
	// @Range: 0.1 4.0
	// @Increment: 0.1
	// @User: User
	AP_GROUPINFO("PRATECMDG",        4, BTOL_Controller, pitchRateCommandGain,        0.5f),

	// @Param: D
	// @DisplayName: Damping Gain
	// @Description: Damping gain from roll acceleration to ailerons. Higher values reduce rolling in turbulence, but can cause oscillations. Automatically set and adjusted by AUTOTUNE mode.
	// @Range: 0 0.2
	// @Increment: 0.01
	// @User: User
	AP_GROUPINFO("YRATECMDG",        5, BTOL_Controller, yawRateCommandGain,        0.5f),

	// @Param: I
	// @DisplayName: Integrator Gain
	// @Description: Integrator gain from long-term roll angle offsets to ailerons. Higher values "trim" out offsets faster but can cause oscillations. Automatically set and adjusted by AUTOTUNE mode.
	// @Range: 0 1.0
	// @Increment: 0.05
	// @User: User
	AP_GROUPINFO("RATTICMDG",        6, BTOL_Controller, rollAttitudeCommandGain,        0.5f),
    AP_GROUPINFO("ROLL_ATR",      7, BTOL_Controller, rollAttitudeErrorToRollRateGain,       1.0f), //attitude error to rate gain.
    AP_GROUPINFO("PTCH_ATR",      8, BTOL_Controller, pitchAttitudeErrorToPitchRateGain,       1.0f),
    AP_GROUPINFO("MTV_TLT_DIR",      9, BTOL_Controller, manualTiltCommandMappingPolarity,       1.0f),
    AP_GROUPINFO("PATTICMDG",        10, BTOL_Controller, pitchAttitudeCommandGain,        0.5f),
    AP_GROUPINFO("M12_MXTRST",        11, BTOL_Controller, motor12MaxThrust,        MOTOR_12_DEFAULT_MAX_THRUST_N),
    AP_GROUPINFO("M3_MXTHRST",        12, BTOL_Controller, motor3MaxThrust,        MOTOR_3_DEFAULT_MAX_THRUST_N),
    AP_GROUPINFO("TOP_TRAN_Q",        13, BTOL_Controller, topOfTransitionDynamicPressure,        TOP_OF_TRANSITION_DEFAULT_DYNAMIC_PRESSURE),
    AP_GROUPINFO("HOV_AC_THR",        14, BTOL_Controller, verticalAccelerationThresholdToConsiderAircraftInHover,        DEFAULT_VERTICAL_ACCELERATION_THRESHOLD_TO_CONSIDER_AIRCRAFT_IN_HOVER),
    AP_GROUPINFO("MASS_KG",        15, BTOL_Controller, aircraftMassInKg,        DEFAULT_AIRCRAFT_MASS_IN_KG),
    AP_GROUPINFO("CG_METERS",        16, BTOL_Controller, centerOfMassLocationX,        DEFAULT_AIRCRAFT_CENTER_OF_MASS_METERS),
    AP_GROUPINFO("M3_TRQ_RATO",        17, BTOL_Controller, motor3ThrustToTorqueCoef,        DEFAULT_MOTOR3_THRUST_TO_TORQUE_COEF),

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
   //float rcCommandInputArmSwitch = 0;
    
    //need protections which set these to neutral values if the signal isn't present. TODO
    rcCommandInputPitchStickAft = constrain_float( ((float)hal.rcin->read(RC_CHANNEL_NUMBER_FOR_PITCH_STICK) - RC_CHANNEL_INPUT_CENTER_VALUE)/RC_CHANNEL_INPUT_HALF_RANGE, -1.0, 1.0);  //this should be a function, private
    rcCommandInputRollStickRight = constrain_float( ((float)hal.rcin->read(RC_CHANNEL_NUMBER_FOR_ROLL_STICK) - RC_CHANNEL_INPUT_CENTER_VALUE)/RC_CHANNEL_INPUT_HALF_RANGE, -1.0, 1.0);  //I don't know if any of these are in the correct orientation.
    rcCommandInputYawStickRight = constrain_float( ((float)hal.rcin->read(RC_CHANNEL_NUMBER_FOR_YAW_STICK) - RC_CHANNEL_INPUT_CENTER_VALUE)/RC_CHANNEL_INPUT_HALF_RANGE, -1.0, 1.0); 
    rcCommandInputThrottleStickForward = constrain_float( ((float)hal.rcin->read(RC_CHANNEL_NUMBER_FOR_THROTTLE_STICK) - RC_CHANNEL_INPUT_CENTER_VALUE)/RC_CHANNEL_INPUT_HALF_RANGE, -1.0, 1.0); 
    rcCommandInputLeftSliderStickForward = constrain_float( ((float)hal.rcin->read(RC_CHANNEL_NUMBER_FOR_LEFT_SLIDER_STICK) - RC_CHANNEL_INPUT_CENTER_VALUE)/RC_CHANNEL_INPUT_HALF_RANGE, -1.0, 1.0); 

    #define MAX_BODY_AXIS_POWERED_LIFT_ACCELERATION_COMMAND_MS 20.0
    #define MAX_BODY_AXIS_POWERED_ACCELERATION_COMMAND_MS 15.0
    float desiredUpAccelerationComponent = ((rcCommandInputThrottleStickForward + 1.0f) / 2.0f) * MAX_BODY_AXIS_POWERED_LIFT_ACCELERATION_COMMAND_MS; //this is the wrong direciton, of course!
    float desiredForwardAccelerationComponent = (rcCommandInputLeftSliderStickForward) * MAX_BODY_AXIS_POWERED_ACCELERATION_COMMAND_MS;
    //need protections
    
    //no longer declared in Plane.h....trying to mimic the soaring controller.

    g2.btolController.setDesiredAccelerationBodyZ(desiredUpAccelerationComponent * -1.0);  //the -1.0 converts this into the +Z = down frame.  this (of course) needs work.
    g2.btolController.setDesiredAccelerationBodyX(desiredForwardAccelerationComponent);  //this (of course) needs work.

    float mtv_direct_command_tilt_angle = 0.0;
    mtv_direct_command_tilt_angle = MTV_MIN_COMMANDABLE_TILT_ANGLE_IN_RADIANS + ((MTV_MAX_COMMANDABLE_TILT_ANGLE_IN_RADIANS-MTV_MIN_COMMANDABLE_TILT_ANGLE_IN_RADIANS) * ((g2.btolController.getTiltCommandMappingPolarity() * rcCommandInputLeftSliderStickForward + 1.0f) / 2.0f));
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

    g2.btolController.setDesiredPassthroughAngularAccelerationPitch(rcCommandInputPitchStickAft * 1.0f);
    g2.btolController.setDesiredPassthroughAngularAccelerationRoll(rcCommandInputRollStickRight * 1.0f);
    g2.btolController.setDesiredPassthroughAngularAccelerationYaw(rcCommandInputYawStickRight * 1.0f); //TODO: these are temporary gain placeholders.


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
        g2.btolController.get_rate_roll_pid().reset_filter();
        g2.btolController.get_rate_pitch_pid().reset_filter();
        g2.btolController.get_rate_yaw_pid().reset_filter();

        g2.btolController.get_rate_roll_pid().reset_I();
        g2.btolController.get_rate_pitch_pid().reset_I();
        g2.btolController.get_rate_yaw_pid().reset_I();
    }


//https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Logger/LogStructure.h#L42
//https://ardupilot.org/dev/docs/code-overview-adding-a-new-log-message.html
    AP::logger().Write("BCMD", "TimeUS,dt_ms,PitchRate,RollRate,YawRate,modeSw,mode,arm",
                   "SSEEE---", // units: seconds, rad/sec
                   "F0000000", // mult: 1e-6, 1e-2
                   "QIfffhhh", // format: uint64_t, float
                   AP_HAL::micros64(),
                   dt_ms,
                   (double)rcCommandInputPitchStickAft * g2.btolController.getPitchRateCommandGain(),
                   (double)rcCommandInputRollStickRight * g2.btolController.getRollRateCommandGain(),
                   (double)rcCommandInputYawStickRight * g2.btolController.getYawRateCommandGain(),
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
    hal.rcout->set_freq(0xF, 400);  //for the first four channels?  0xF = 1111  Testing!  Works!  
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



    /*uint32_t now = AP_HAL::millis();
    if (now - last_stabilize_ms > 2000) {
        // if we haven't run the rate controllers for 2 seconds then
        // reset the integrators
       // rollController.reset_I();
       // pitchController.reset_I();
       // yawController.reset_I();

        // and reset steering controls
       // steer_state.locked_course = false;
       // steer_state.locked_course_err = 0;
    }
        last_stabilize_ms = now;
*/


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

    int16_t servoControlValue5 = SERVO_CONTROL_CENTER_VALUE;// + constrain_int16(int16_t(rcCommandInputPitchStickAft*500), -500, 500);  //works (float)

    EffectorList effectorCommands;
    float deltaTimeSecondsForPID = ((float)dt_us)/1000000.0f;
    effectorCommands = g2.btolController.calculateEffectorPositions(deltaTimeSecondsForPID); //PID_400HZ_DT); //this delta time is wrong, of course!  Should be dynamicly populated.

    int16_t servoControlValueElevon1 = g2.btolController.calculateServoValueFromAngle(effectorCommands.elevon1Angle, AIRCRAFT_PROPERTIES_ELEVON1_SERVO_MIN_ANGLE, AIRCRAFT_PROPERTIES_ELEVON1_SERVO_MAX_ANGLE, AIRCRAFT_PROPERTIES_ELEVON1_SERVO_MIN_ANGLE_PWM, AIRCRAFT_PROPERTIES_ELEVON1_SERVO_MAX_ANGLE_PWM);
    int16_t servoControlValueElevon2 = g2.btolController.calculateServoValueFromAngle(effectorCommands.elevon2Angle, AIRCRAFT_PROPERTIES_ELEVON2_SERVO_MIN_ANGLE, AIRCRAFT_PROPERTIES_ELEVON2_SERVO_MAX_ANGLE, AIRCRAFT_PROPERTIES_ELEVON2_SERVO_MIN_ANGLE_PWM, AIRCRAFT_PROPERTIES_ELEVON2_SERVO_MAX_ANGLE_PWM);
    int16_t servoControlValueTilt1 = g2.btolController.calculateServoValueFromAngle(effectorCommands.tilt1Angle, TILT1_SERVO_MIN_ANGLE, TILT1_SERVO_MAX_ANGLE, TILT1_SERVO_MIN_ANGLE_PWM, TILT1_SERVO_MAX_ANGLE_PWM);
    int16_t servoControlValueTilt2 = g2.btolController.calculateServoValueFromAngle(effectorCommands.tilt2Angle, TILT2_SERVO_MIN_ANGLE, TILT2_SERVO_MAX_ANGLE, TILT2_SERVO_MIN_ANGLE_PWM, TILT2_SERVO_MAX_ANGLE_PWM);

    //starting out expecting values from 0.0 to +1.0
    //need to convert motor thrust to ESC commands using some scalar...ie: from newtons to scalar 0.0 to 1.0
    int16_t servoControlValueMotor1 = MOTOR_CONTROL_MIN_VALUE + constrain_int16(int16_t((effectorCommands.motor1Thrust / g2.btolController.getMotor12MaxThrust()) * MOTOR_CONTROL_RANGE), 0, MOTOR_CONTROL_RANGE); //this needs to be scaled and offset correctly TODO
    int16_t servoControlValueMotor2 = MOTOR_CONTROL_MIN_VALUE + constrain_int16(int16_t((effectorCommands.motor2Thrust / g2.btolController.getMotor12MaxThrust())* MOTOR_CONTROL_RANGE), 0, MOTOR_CONTROL_RANGE);
    int16_t servoControlValueMotor3 = MOTOR_CONTROL_MIN_VALUE + constrain_int16(int16_t((effectorCommands.motor3Thrust / g2.btolController.getMotor3MaxThrust()) * MOTOR_CONTROL_RANGE), 0, MOTOR_CONTROL_RANGE);  //isn't working.  Is stuck at 2000.

    if(g2.btolController.getArmedState() != 1)
    {
        servoControlValueMotor1 = THROTTLE_DISARMED_VALUE;
        servoControlValueMotor2 = THROTTLE_DISARMED_VALUE;
        servoControlValueMotor3 = THROTTLE_DISARMED_VALUE;
    }

//Need to put more protections!  better than above.
//consider swapping the order so the motors can be in the first 4 if needed to get 400Hz update rate.
    hal.rcout->cork();  //  SRV_Channels::cork();
    hal.rcout->write(CH_1, servoControlValueMotor1);
    hal.rcout->write(CH_2, servoControlValueMotor2);  //not working...perhaps base zero??
    hal.rcout->write(CH_3, servoControlValueMotor3);
    hal.rcout->write(CH_4, servoControlValue5);
    hal.rcout->write(CH_5, servoControlValueElevon1);
    hal.rcout->write(CH_6, servoControlValueElevon2);
    hal.rcout->write(CH_7, servoControlValueTilt1);
    hal.rcout->write(CH_8, servoControlValueTilt2);


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
    AP::logger().Write("BSTB", "t_us,dt_ms,dt_us,ft_us,dt_PID,c1,c2,c3,c4,c5,c6,c7,c8",
                   "SSSSS--------", // units: seconds, rad/sec
                   "F000000000000", // mult: 1e-6, 1e-2
                   "QIIIfhhhhhhhh", // format: uint64_t, float
                   AP_HAL::micros64(),
                   dt_ms,
                   dt_us,
                   functionTime_us,
                   (double) deltaTimeSecondsForPID,
                   servoControlValueElevon1,
                   servoControlValueElevon2,
                   servoControlValueTilt1,
                   servoControlValueTilt2,
                   servoControlValue5,
                   servoControlValueMotor1,
                   servoControlValueMotor2,
                   servoControlValueMotor3
                   );

AP::logger().Write("BEFF", "t_us,m1,m2,m3,e1,e2,t1,t2,mass",
                   "S--------", // units: seconds, rad/sec
                   "F00000000", // mult: 1e-6, 1e-2
                   "Qffffffff", // format: uint64_t, float
                   AP_HAL::micros64(),
                   (double)effectorCommands.motor1Thrust,
                   (double)effectorCommands.motor2Thrust,
                   (double)effectorCommands.motor3Thrust,
                   (double)effectorCommands.elevon1Angle,
                   (double)effectorCommands.elevon2Angle,
                   (double)effectorCommands.tilt1Angle,
                   (double)effectorCommands.tilt2Angle,
                   (double)g2.btolController.getAircraftMass()
                    );
//battery.read();

AP::logger().Write("BBAT", "t_us,v,v_rest,%",
                   "S---", // units: seconds, rad/sec
                   "F000", // mult: 1e-6, 1e-2
                   "Qfff", // format: uint64_t, float
                   AP_HAL::micros64(),
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

float BTOL_Controller::getRangeRatio(float value, float min, float max)
{
    float ratio = 0.0;
    if(max < min)
    {
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

EffectorList BTOL_Controller::calculateEffectorPositions(float dt)
{

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

    //now do some regulator stuff!
    float desiredMomentX = 0.0f;
    float desiredMomentY = 0.0f;
    float desiredMomentZ = 0.0f;

    if(state.regulatorMode == CONTROLLER_STATE_REGULATOR_MODE_ATTITUDE)
    {
      //  Attitude Command ...will be useful later...need to figure out how to get both!
      //  float attitudeErrorRoll = command.targetRollAttitude - _ahrs.get_roll();
      //  float attitudeErrorPitch = command.targetPitchAttitude - _ahrs.get_pitch();

      //  float targetRollRate = attitudeErrorRoll * rollAttitudeErrorToRollRateGain.get(); //this could be an issue, also.  TODO
      //  float targetPitchRate = attitudeErrorPitch * pitchAttitudeErrorToPitchRateGain.get(); //not sure if this is the right way to do this!


       // float BTOL_Controller::getRangeRatio(float value, float min, float max);

        float dynamicPressureRatio = getRangeRatio(dynamicPressure, 0.0f, getTopOfTransitionDynamicPressure());
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

        float targetRollRate = command.targetRollRate + getAugmentedStabilityRatioWithinAngleRange(_ahrs.get_roll(), innerStabilizationAngleRoll, outerStabilizationAngleRoll) * getRollRateCommandGain();
        float targetPitchRate = command.targetPitchRate + getAugmentedStabilityRatioWithinAngleRange(_ahrs.get_pitch(), innerStabilizationAnglePitch, outerStabilizationAnglePitch) * getPitchRateCommandGain();

        //could try summing the demanded rates and the attitude stability instead of this explicit values?
    //    float targetRollRate = command.targetRollRate + getAugmentedStabilityRatioWithinAngleRange(_ahrs.get_roll(), 0.0f, 1.0f) * getRollRateCommandGain();
    //    float targetPitchRate = command.targetPitchRate + getAugmentedStabilityRatioWithinAngleRange(_ahrs.get_pitch(), 0.0f, 1.0f) * getPitchRateCommandGain();


        //get_rate_roll_pid()
        desiredMomentX = get_rate_roll_pid().update_all(targetRollRate,_ahrs.get_gyro().x, false);
        desiredMomentY = get_rate_pitch_pid().update_all(targetPitchRate,_ahrs.get_gyro().y, false);

        //float lateralAcceleration = _ahrs.get_accel().y; //lateral Acceleration.

        //_ahrs.get_accel(). adsaf

        // Get body frame accel vector
        //Vector3f initAccVec;
        //uint8_t counter = 0;
        //initAccVec = _ins.get_accel();
        //??????  https://ardupilot.org/plane/docs/roll-pitch-controller-tuning.html#tuning-the-yaw-damper  look here!
        //might not have enought side area to matter...!


        //perhaps this should be pilot input + rate stability?  as we need to be able to remove rate stability in forward flight and apply some turn coordination, if needed.
        desiredMomentZ = get_rate_yaw_pid().update_all(command.targetYawRate,_ahrs.get_gyro().z, false);
    }

    if(state.regulatorMode == CONTROLLER_STATE_REGULATOR_MODE_RATE)
    {
        desiredMomentX = get_rate_roll_pid().update_all(command.targetRollRate,_ahrs.get_gyro().x, false);
        desiredMomentY = get_rate_pitch_pid().update_all(command.targetPitchRate,_ahrs.get_gyro().y, false);
        desiredMomentZ = get_rate_yaw_pid().update_all(command.targetYawRate,_ahrs.get_gyro().z, false);

    }

    //Log the PID values.
    //not sure what to put in the identifier enum  just going to try using the default ones...because the defaults should't be running, because I disabled them.
    AP::logger().Write_PID(LOG_PIDR_MSG, get_rate_roll_pid().get_pid_info());
    AP::logger().Write_PID(LOG_PIDP_MSG, get_rate_pitch_pid().get_pid_info());
    AP::logger().Write_PID(LOG_PIDY_MSG, get_rate_yaw_pid().get_pid_info());

    //calculate desired moments
        //calculate reponse to the rate errors.
        //overwrite values if in passthrough test mode.
        if(state.regulatorMode == CONTROLLER_STATE_REGULATOR_MODE_PASSTHROUGH)
        {
            desiredMomentX = command.passthroughAngularAccelerationRoll * aircraftProperties.momentOfInertiaRoll; //passthrough to start to help build effector blender.
            desiredMomentY = command.passthroughAngularAccelerationPitch * aircraftProperties.momentOfInertiaPitch;
            desiredMomentZ = command.passthroughAngularAccelerationYaw * aircraftProperties.momentOfInertiaYaw;
        }
        

    //add the torque from motor3 (assuming motors 1 and 2 cancel out!)
    float estimatedNetMotorTorqueToCancelOut = 0.0;
    estimatedNetMotorTorqueToCancelOut = effectors.motor3Thrust * motor3ThrustToTorqueCoef;
    desiredMomentZ = desiredMomentZ - estimatedNetMotorTorqueToCancelOut;

        
    //calculate desired forces
    float desiredAccelerationZ = 0.0f;
    float desiredAccelerationX = 0.0f;
        if(state.commandMode == 0) //manual tilt angle
        {
            //calculate vector components based on tilt and thrust
            desiredAccelerationZ = -1.0 * sinf(command.targetTiltAngle) * command.targetTiltAcceleration; //vertical component in body frame. (+Z = down)
            desiredAccelerationX =  cosf(command.targetTiltAngle) * command.targetTiltAcceleration; //longitudinal component in body frame. (+X = forward)
            
        }else{
            //take target accelerations already vectorized and use those.
            desiredAccelerationZ = command.targetAccelerationZ;  //Right now commanded directly by pilot
            desiredAccelerationX = command.targetAccelerationX;  //Right now commanded directly by pilot
        }

        float requiredForceZ = desiredAccelerationZ * getAircraftMass();//aircraftProperties.totalMass; //Newtons
        float requiredForceX = desiredAccelerationX * getAircraftMass();//aircraftProperties.totalMass; //Newtons

    //calculate effector outputs

        //control surfaces: Elevons.
        //Trailing edge up is positive.
        float elevon1Angle = 0.0f; //there is likely a better metric...ratio, or effort, or contribution...
        float elevon2Angle = 0.0f;
        float pitchMomentToElevonSurfaceDeflectionGain = 1.0f;
        float rollMomentToElevonSurfaceDeflectionGainPos = 1.0f;

        #define ELEVON_COEF_OF_LIFT_PER_DEFLECTION 4.0f //(2*PI?)

        //TODO: have control surface movement become more limited as we get into hover?  Ie: shape the up and down limits?
        float distanceXFromCGElevonsAbsv = fabs(aircraftProperties.elevon1LocationX - getCenterOfMassLocationX()); //aircraftProperties.centerOfMassLocationX);//will be positive //should I make this an abs
        float distanceYFromCGElevonsAbsv = fabs(aircraftProperties.elevon1LocationY - aircraftProperties.centerOfMassLocationY);//will be positive //should I make these ABS?  and understand that they are symetric?

        //this is assuming symetrical elevons, and symetrical deflections.  TODO: protect against negative values.
        //float elevonMaxForceMagnitude = getControlSurfaceForce(AIRCRAFT_PROPERTIES_ELEVON1_SERVO_MAX_ANGLE, AIRCRAFT_PROPERTIES_ELEVON_AREA_M2, dynamicPressure);

        //Calculate how much force the surface will generate per deflection.  Assuming linear (or close-enough)
        float elevonDeflectionToForceGain = dynamicPressure * AIRCRAFT_PROPERTIES_ELEVON_AREA_M2 * (1.0f) * ELEVON_COEF_OF_LIFT_PER_DEFLECTION;
        //protect agains div/0! //TODO: //Make better.
        float protectedElevonDeflectionToForceGain = elevonDeflectionToForceGain;
        if (protectedElevonDeflectionToForceGain < AIRCRAFT_PROPERTIES_ELEVON_AREA_M2 * 50 * ELEVON_COEF_OF_LIFT_PER_DEFLECTION)
        {
            protectedElevonDeflectionToForceGain = AIRCRAFT_PROPERTIES_ELEVON_AREA_M2 * 50 * ELEVON_COEF_OF_LIFT_PER_DEFLECTION;
        }

        //Calculate the moment-to-deflection gain so we can calculate the deflection (later).  We use the force and distance.
        pitchMomentToElevonSurfaceDeflectionGain = 1.0f / (protectedElevonDeflectionToForceGain * distanceXFromCGElevonsAbsv); //the (+) is to convert from trailing edge up = positive to force -down = positive. with the positve arm...
        rollMomentToElevonSurfaceDeflectionGainPos = 1.0f / (protectedElevonDeflectionToForceGain * distanceYFromCGElevonsAbsv); //the (+) is to convert from trailing edge up = positive to force -down = positive. with the positve arm...to positive roll.

        //Calculate desired angle for Elevon # 1 (Left).  The 0.5f is because we (implicitly) have two elevons to split the moments.
        elevon1Angle = 0.5f*pitchMomentToElevonSurfaceDeflectionGain * desiredMomentY - 0.5f*rollMomentToElevonSurfaceDeflectionGainPos * desiredMomentX;
        //Limit elevon deflection to hardware limits.
        if(elevon1Angle > AIRCRAFT_PROPERTIES_ELEVON1_SERVO_MAX_ANGLE)
        {
            elevon1Angle = AIRCRAFT_PROPERTIES_ELEVON1_SERVO_MAX_ANGLE;
        }else if(elevon1Angle < AIRCRAFT_PROPERTIES_ELEVON1_SERVO_MIN_ANGLE)
        {
            elevon1Angle = AIRCRAFT_PROPERTIES_ELEVON1_SERVO_MIN_ANGLE;
        }
        //Calculate desired angle for Elevon # 2 (Right) The 0.5f is because we (implicitly) have two elevons to split the moments.
        elevon2Angle = 0.5f*pitchMomentToElevonSurfaceDeflectionGain * desiredMomentY + 0.5f*rollMomentToElevonSurfaceDeflectionGainPos * desiredMomentX;
        //Limit elevon deflection to hardware limits.
        if(elevon2Angle > AIRCRAFT_PROPERTIES_ELEVON2_SERVO_MAX_ANGLE)
        {
            elevon2Angle = AIRCRAFT_PROPERTIES_ELEVON2_SERVO_MAX_ANGLE;
        }else if(elevon2Angle < AIRCRAFT_PROPERTIES_ELEVON2_SERVO_MIN_ANGLE)
        {
            elevon2Angle = AIRCRAFT_PROPERTIES_ELEVON2_SERVO_MIN_ANGLE;
        }

        //SOLVED: I think there is an issue here as pitch is resulting in a motor roll command.
        //Estimate how much moment the elevons are applying to the airframe based on the deflection, deflection to force gain (dynamic pressure), and arm)
        float estimatedAttainedElevonMomentY = elevon1Angle * elevonDeflectionToForceGain * 1.0f * distanceXFromCGElevonsAbsv  +   elevon2Angle * elevonDeflectionToForceGain * 1.0f * distanceXFromCGElevonsAbsv;
        float estimatedAttainedElevonMomentX = elevon1Angle * elevonDeflectionToForceGain * -1.0f * distanceYFromCGElevonsAbsv +   elevon2Angle * elevonDeflectionToForceGain * 1.0f * distanceYFromCGElevonsAbsv;

        float residualElevonMomentX = desiredMomentX - estimatedAttainedElevonMomentX;
        float residualElevonMomentY = desiredMomentY - estimatedAttainedElevonMomentY;

        effectors.elevon1Angle = elevon1Angle;
        effectors.elevon2Angle = elevon2Angle;

        AP::logger().Write("BELE", "TimeUS,q,rMom,pMom,elvDeflFrceGn,e1Cmd,e2Cmd,resX,resY,mM",
            "S---------", // units: seconds, rad/sec
            "FB00A00000", // mult: 1e-6, 1e-2
            "Qfffffffff", // format: uint64_t, float
            AP_HAL::micros64(),
            (double)dynamicPressure,
            (double)desiredMomentX,
            (double)desiredMomentY,
            (double)elevonDeflectionToForceGain,
            (double)elevon1Angle,
            (double)elevon2Angle,
            (double)residualElevonMomentX,
            (double)residualElevonMomentY,
            (double)estimatedNetMotorTorqueToCancelOut
            );

        //TEST:  //NOT TESTED YET!...tested some....
        //Have the motors do what the controls surfaces cannot.
        desiredMomentX = residualElevonMomentX; 
        desiredMomentY = residualElevonMomentY; 

        //Now calculate motors and tilts!
        //Calculate forward/aft motor thurst(force) distribution from the total body Z force desired and the desired pitching moment.
        float totalForceUp = 0.0f; //backwards
        float totalMomentForward = 0.0f; //backwards, but this is how I did the math, so lets start with this, as the arms work out this way if the x axis is used
        float forceUpMotors1and2 = 0.0f;
        float forceUpMotor3 = 0.0f;

        totalForceUp = requiredForceZ * -1.0f; //-1.0 is converting from the +Z = down frame //could get rid of this, and fix when we calculate the tilt angle?
        totalMomentForward = desiredMomentY;  //there is no -1.0 here because the moment arms are defined in such a way that the moment produced is negative!

        //TODO: need a divide by zero check here.  Make sure that D12 and D3 are not equal!
        float distanceXFromCGMotors12 = aircraftProperties.motor1LocationX - getCenterOfMassLocationX();// aircraftProperties.centerOfMassLocationX;
        float distanceXFromCGMotor3 = aircraftProperties.motor3LocationX - getCenterOfMassLocationX(); //aircraftProperties.centerOfMassLocationX;

        forceUpMotors1and2 = (totalMomentForward - distanceXFromCGMotor3 * totalForceUp) / (distanceXFromCGMotors12 - distanceXFromCGMotor3); //this appears to be working...pitch seems to be backwards, but total force up is working, which means required force Z and pitch moment need to be reversed upstream of here.. could also be an transmitter reversing issue.
        forceUpMotor3 = (distanceXFromCGMotors12 * totalForceUp - totalMomentForward) / (distanceXFromCGMotors12 - distanceXFromCGMotor3); //this appears to be working...don't know about direction/magnitude...

        //NOW CALCULATE DIFFERENT FORCES FOR THE ROLL CONTROLLER
        float forceUpMotor1 = 0.0;
        float forceUpMotor2 = 0.0;
        float deltaForceUpStation12ForRoll = 0.0; //re-aranged so it would say "deltaForce" =)

        //Note this will change if we switch to the +Z = down coordinate system. (will need to remove the *-1.0)
        //this will be very simplified because of the symetric design of the airplane.  Can make more encompasing later!
        float distanceYFromCGMotors12ForRoll = aircraftProperties.motor1LocationY - aircraftProperties.centerOfMassLocationY;//(this is a negative value)  CG = 0.0.
        deltaForceUpStation12ForRoll = desiredMomentX / (2.0f * distanceYFromCGMotors12ForRoll);
        #define DELTA_MOTOR_FORCE_UP_FOR_ROLL_POLARITY 1.0 //change this if we move to a controls frame down.


        forceUpMotor1 = forceUpMotors1and2 / 2.0f + DELTA_MOTOR_FORCE_UP_FOR_ROLL_POLARITY * -1.0f * deltaForceUpStation12ForRoll;
        forceUpMotor2 = forceUpMotors1and2 / 2.0f + DELTA_MOTOR_FORCE_UP_FOR_ROLL_POLARITY *  1.0f  * deltaForceUpStation12ForRoll;

        //NOW CALCULATE DIFFERENT FORCES FOR THE YAW CONTROLLER
        float forceForwardMotors12 = 0.0;
        float forceForwardMotor1 = 0.0;
        float forceForwardMotor2 = 0.0;
        float deltaForceForwardStation12ForYaw = 0.0;

        //max forward thrust should be defined on the capabilities of the motors, and need for yaw overhead.
        forceForwardMotors12 = constrain_float(requiredForceX, -20, 20); //temp min and max thrust values.  //Will need to change this for forward flight vs hover.

        #define DELTA_MOTOR_FORCE_FORWARD_FOR_YAW_POLARITY 1.0 //change this if we move to a controls frame down.
        float distanceYFromCGMotors12ForYaw = aircraftProperties.motor1LocationY - aircraftProperties.centerOfMassLocationY;//(this is a negative value)  CG = 0.0.
        deltaForceForwardStation12ForYaw = desiredMomentZ / (2.0f * distanceYFromCGMotors12ForYaw);

        forceForwardMotor1 = forceForwardMotors12 / 2.0f + DELTA_MOTOR_FORCE_FORWARD_FOR_YAW_POLARITY * -1.0f * deltaForceForwardStation12ForYaw;
        forceForwardMotor2 = forceForwardMotors12 / 2.0f + DELTA_MOTOR_FORCE_FORWARD_FOR_YAW_POLARITY *  1.0f  * deltaForceForwardStation12ForYaw;

        //Calculate tilt angle from the forces...

        //for the tilt angle calulation, enforce a minimum upward force.
        float idealTilt1Angle = 0.0f; //0 is forward, 90degrees (this is radians) is up.
        float tiltCalculationMotor1ForceUp = forceUpMotor1;
        float tiltCalculationMotor1ForceForward = forceForwardMotor1;
        /*#define TILT_CALCULATION_MOTOR_UP_FORCE_MIN_VALUE 0.01f  //this should only happen when there are small numbers to deal with, ie when the total trust magniude is low.....this to make the tilts want to point up.  There is a better way.  TODO:
        if(tiltCalculationMotor1ForceUp < TILT_CALCULATION_MOTOR_UP_FORCE_MIN_VALUE)
        {
            tiltCalculationMotor1ForceUp = TILT_CALCULATION_MOTOR_UP_FORCE_MIN_VALUE;
        }*/

        idealTilt1Angle = atan2f(tiltCalculationMotor1ForceUp, tiltCalculationMotor1ForceForward);


        float tilt1Angle = 0.0f;
        tilt1Angle = constrain_float(idealTilt1Angle, TILT1_SERVO_MIN_ANGLE, TILT2_SERVO_MAX_ANGLE);

        float motor1ForceDemand = 0.0;
        #define TILT_SATISFACTION_ANGLE_LOW 0.174533f
        #define TILT_SATISFACTION_ANGLE_HIGH 1.309f
        #define TILT_ANGLE_CONSTRAINT_FROM_COLLECTIVE_TILT_ANGLE_GAIN_RAD_PER_MSS 0.1f //*10m/s/s = 1.0 rad which is about 57deg
        float maxConstrainedIndividualTiltAngleDelta = TILT_ANGLE_CONSTRAINT_FROM_COLLECTIVE_TILT_ANGLE_GAIN_RAD_PER_MSS * command.targetTiltAcceleration + 0.1f; //allow some movement at no throttle.
        float maxConstrainedIndividualTiltAngleMax = command.targetTiltAngle + maxConstrainedIndividualTiltAngleDelta;
        float maxConstrainedIndividualTiltAngleMin = command.targetTiltAngle - maxConstrainedIndividualTiltAngleDelta;
        tilt1Angle = constrain_float(tilt1Angle, maxConstrainedIndividualTiltAngleMin, maxConstrainedIndividualTiltAngleMax);
        //#define TILT_ANGLE_MAX_DELTA_FROM_COLLECTIVE_TILT_ANGLE...this doesn't work because of the contribution from the tail motor...
        //lets make it a large value which is enough to keep the motors from hitting the ground at low thrust values.

        //TODO: individual tilts shouldn't be making full decisions.  look at collective values.
        motor1ForceDemand = calculateMotorThrustBasedOnTiltAngle(tilt1Angle, tiltCalculationMotor1ForceForward, tiltCalculationMotor1ForceUp, TILT_SATISFACTION_ANGLE_LOW, TILT_SATISFACTION_ANGLE_HIGH);

        //the tilts move fast, but not that fast...
        //move tilt 1 angle to this value at the correct rate, Add tilt rate limits, so we can slew correctly and don’t put motor values in that aren’t aligned with tilt angle.

        float idealTilt2Angle = 0.0f;
        float tiltCalculationMotor2ForceUp = forceUpMotor2;
        float tiltCalculationMotor2ForceForward = forceForwardMotor2;
        //#define TILT_CALCULATION_MOTOR_UP_FORCE_MIN_VALUE 0.2  //this should only happen when there are small numbers to deal with, ie when the total trust magniude is low...
     //   if(tiltCalculationMotor2ForceUp < TILT_CALCULATION_MOTOR_UP_FORCE_MIN_VALUE)
     //   {
      //      tiltCalculationMotor2ForceUp = TILT_CALCULATION_MOTOR_UP_FORCE_MIN_VALUE;
       // }

        idealTilt2Angle = atan2f(tiltCalculationMotor2ForceUp, tiltCalculationMotor2ForceForward);

        float tilt2Angle = 0.0f;
        tilt2Angle = constrain_float(idealTilt2Angle, TILT1_SERVO_MIN_ANGLE, TILT2_SERVO_MAX_ANGLE);
        tilt2Angle = constrain_float(tilt2Angle, maxConstrainedIndividualTiltAngleMin, maxConstrainedIndividualTiltAngleMax);

        float motor2ForceDemand = calculateMotorThrustBasedOnTiltAngle(tilt2Angle, tiltCalculationMotor2ForceForward, tiltCalculationMotor2ForceUp, TILT_SATISFACTION_ANGLE_LOW, TILT_SATISFACTION_ANGLE_HIGH);
        
        float motor3ForceDemand = forceUpMotor3; //make sure can't be negative...or update the firmware so it can be!

        effectors.tilt1Angle = tilt1Angle;
        effectors.tilt2Angle = tilt2Angle;

        effectors.motor1Thrust = motor1ForceDemand;//forceUpMotor1; //will need to be replaced the total force, not just the up force!
        effectors.motor2Thrust = motor2ForceDemand; //forceUpMotor2; //will need to be replaced the total force, not just the up force!
        effectors.motor3Thrust = motor3ForceDemand; //will need to be replaced the total force, not just the up force!
    

    return effectors;
}

#endif //BTOL_ENABLED
