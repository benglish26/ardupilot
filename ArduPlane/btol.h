#pragma once

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Common/AP_Common.h>
#include <AP_Vehicle/AP_Vehicle.h>
//#include "AP_AutoTune.h"
#include <AP_Logger/AP_Logger.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AC_PID/AC_PID.h>
#include <AC_PID/AC_P.h>

// default rate controller PID gains
/*#define AC_ATC_HELI_RATE_RP_P                       0.024f
#define AC_ATC_HELI_RATE_RP_I                       0.6f
#define AC_ATC_HELI_RATE_RP_D                       0.001f
#define AC_ATC_HELI_RATE_RP_IMAX                    1.0f
#define AC_ATC_HELI_RATE_RP_FF                      0.060f
#define AC_ATC_HELI_RATE_RP_FILT_HZ                 20.0f
#define AC_ATC_HELI_RATE_YAW_P                      0.18f
#define AC_ATC_HELI_RATE_YAW_I                      0.12f
#define AC_ATC_HELI_RATE_YAW_D                      0.003f
#define AC_ATC_HELI_RATE_YAW_IMAX                   1.0f
#define AC_ATC_HELI_RATE_YAW_FF                     0.024f
#define AC_ATC_HELI_RATE_YAW_FILT_HZ                20.0f*/

#define AC_PID_ROLL_RATE_P                          0.2f
#define AC_PID_ROLL_RATE_I                          0.0f
#define AC_PID_ROLL_RATE_D                          0.02f
#define AC_PID_ROLL_RATE_IMAX                       0.0f
#define AC_PID_ROLL_RATE_FF                         0.060f
#define AC_PID_ROLL_RATE_FILTER_T                   0.0f  //target filter
#define AC_PID_ROLL_RATE_FILTER_E                   0.0f //error filter
#define AC_PID_ROLL_RATE_FILTER_D                   0.0f //derivitive filter
//https://github.com/KalebKE/AccelerationExplorer/wiki/Low-Pass-Filter-Optimize-Alpha
//see: AC_PID.cpp: float AC_PID::get_filt_alpha(float filt_hz) const  0.0f is no filtering.

#define AC_PID_PITCH_RATE_P                          0.2f
#define AC_PID_PITCH_RATE_I                          0.0f
#define AC_PID_PITCH_RATE_D                          0.02f
#define AC_PID_PITCH_RATE_IMAX                       0.0f
#define AC_PID_PITCH_RATE_FF                         0.060f
#define AC_PID_PITCH_RATE_FILTER_T                   0.0f  //target filter
#define AC_PID_PITCH_RATE_FILTER_E                   0.0f //error filter
#define AC_PID_PITCH_RATE_FILTER_D                   0.0f //derivitive filter

#define AC_PID_YAW_RATE_P                          0.2f
#define AC_PID_YAW_RATE_I                          0.0f
#define AC_PID_YAW_RATE_D                          0.02f
#define AC_PID_YAW_RATE_IMAX                       0.0f
#define AC_PID_YAW_RATE_FF                         0.060f
#define AC_PID_YAW_RATE_FILTER_T                   0.0f  //target filter
#define AC_PID_YAW_RATE_FILTER_E                   0.0f //error filter
#define AC_PID_YAW_RATE_FILTER_D                   0.0f //derivitive filter

#define PID_400HZ_DT                            0.0025f

/*#define AC_ATC_HELI_RATE_YAW_P                      0.18f
#define AC_ATC_HELI_RATE_YAW_I                      0.12f
#define AC_ATC_HELI_RATE_YAW_D                      0.003f
#define AC_ATC_HELI_RATE_YAW_IMAX                   1.0f
#define AC_ATC_HELI_RATE_YAW_FF                     0.024f
#define AC_ATC_HELI_RATE_YAW_FILT_HZ                20.0f
#define AC_ATTITUDE_HELI_RATE_RP_FF_FILTER          10.0f
#define AC_ATTITUDE_HELI_RATE_Y_VFF_FILTER          10.0f*/



#define CONTROLLER_STATE_REGULATOR_MODE_PASSTHROUGH 1
#define CONTROLLER_STATE_REGULATOR_MODE_RATE 2
#define CONTROLLER_STATE_REGULATOR_MODE_ATTITUDE 3

#define AIRCRAFT_PROPERTIES_ELEVON1_SERVO_MIN_ANGLE -0.506f //Radians.
#define AIRCRAFT_PROPERTIES_ELEVON1_SERVO_MIN_ANGLE_PWM 1000  //PWM uS
#define AIRCRAFT_PROPERTIES_ELEVON1_SERVO_MAX_ANGLE 0.506f  //radians
#define AIRCRAFT_PROPERTIES_ELEVON1_SERVO_MAX_ANGLE_PWM 1950 //PWM uS

#define AIRCRAFT_PROPERTIES_ELEVON2_SERVO_MIN_ANGLE -0.506f //Radians.
#define AIRCRAFT_PROPERTIES_ELEVON2_SERVO_MIN_ANGLE_PWM 2000  //PWM uS
#define AIRCRAFT_PROPERTIES_ELEVON2_SERVO_MAX_ANGLE 0.506f  //radians
#define AIRCRAFT_PROPERTIES_ELEVON2_SERVO_MAX_ANGLE_PWM 1050 //PWM uS

#define AIRCRAFT_PROPERTIES_ELEVON_AREA_M2 0.006847f


struct EffectorList
{ 
    float motor1Thrust; //Newtons
    float motor2Thrust; //Newtons
    float motor3Thrust; //Newtons
    float tilt1Angle; //Radians
    float tilt2Angle; //Radians
    float elevon1Angle; //Radians
    float elevon2Angle; //Radians

    //TODO: add max/min and rates.
};  

struct ControllerState
{
    int regulatorMode; //1 = passthrough, 2 = rate command, 3 = attitude command
    int commandMode; //0 = Manual vectored thrust.
    int16_t armedState; //0 = disarmed, 1 = armed.
    //int usePolarTiltInput;
    
};

struct CommandInput
{
    float targetPitchAttitude;
    float targetRollAttitude;
    float targetPitchRate;
    float targetRollRate;
    float targetYawRate;
    float targetHeading;

    float targetAccelerationZ;
    float targetAccelerationX;
    float targetTiltAngle;
    float targetTiltAcceleration;
    float passthroughAngularAccelerationRoll;
    float passthroughAngularAccelerationPitch;
    float passthroughAngularAccelerationYaw;

};

struct AircraftProperties
{ 
    float totalMass; //kg
    float momentOfInertiaPitch;
    float momentOfInertiaRoll;
    float momentOfInertiaYaw;
    float motor1LocationX;
    float motor1LocationY;
    float motor1LocationZ;

    float motor2LocationX;
    float motor2LocationY;
    float motor2LocationZ;

    float motor3LocationX;
    float motor3LocationY;
    float motor3LocationZ;

    float centerOfMassLocationX;
    float centerOfMassLocationY;
    float centerOfMassLocationZ;

    float elevon1LocationX;
    float elevon1LocationY;

    float elevon2LocationX;
    float elevon2LocationY;



    //TODO: add max/min and rates.
};  


class BTOL_Controller {
public:
    //https://www.geeksforgeeks.org/when-do-we-use-initializer-list-in-c/
    BTOL_Controller(AP_AHRS &ahrs, const AP_Vehicle::FixedWing &parms): 
    _ahrs(ahrs), 
    aparm(parms),
    _pid_rate_roll(AC_PID_ROLL_RATE_P, AC_PID_ROLL_RATE_I, AC_PID_ROLL_RATE_D, AC_PID_ROLL_RATE_IMAX, AC_PID_ROLL_RATE_FF, AC_PID_ROLL_RATE_FILTER_T, AC_PID_ROLL_RATE_FILTER_E, AC_PID_ROLL_RATE_FILTER_D, PID_400HZ_DT),
    _pid_rate_pitch(AC_PID_PITCH_RATE_P, AC_PID_PITCH_RATE_I, AC_PID_PITCH_RATE_D, AC_PID_PITCH_RATE_IMAX, AC_PID_PITCH_RATE_FF, AC_PID_PITCH_RATE_FILTER_T, AC_PID_PITCH_RATE_FILTER_E, AC_PID_PITCH_RATE_FILTER_D, PID_400HZ_DT),
    _pid_rate_yaw(AC_PID_YAW_RATE_P, AC_PID_YAW_RATE_I, AC_PID_YAW_RATE_D, AC_PID_YAW_RATE_IMAX, AC_PID_YAW_RATE_FF, AC_PID_YAW_RATE_FILTER_T, AC_PID_YAW_RATE_FILTER_E, AC_PID_YAW_RATE_FILTER_D, PID_400HZ_DT)
    {
        AP_Param::setup_object_defaults(this, var_info);
        command.targetPitchAttitude = 0.0f;
        command.targetRollAttitude = 0.0f;
        command.targetPitchRate = 0.0f;
        command.targetRollRate = 0.0f;
        command.targetYawRate = 0.0f;
        command.targetHeading = 0.0f;
        pitchRateError = 0.0f;
        rollRateError = 0.0f;
        yawRateError = 0.0f;

        command.targetAccelerationZ = 0.0f;
        command.targetAccelerationX = 0.0f;
        command.passthroughAngularAccelerationRoll = 0.0f;
        command.passthroughAngularAccelerationPitch = 0.0f;
        command.passthroughAngularAccelerationYaw = 0.0f;

        aircraftProperties.totalMass = getAircraftMass();//1.0;  //approximaton.  Make tuning parameter and figure out actual value.  Don't use this!
        aircraftProperties.momentOfInertiaPitch = 0.01492262208f;  //gross approximaton.  Make tuning parameter figure out actual value.
        aircraftProperties.momentOfInertiaRoll = 0.01788486311f;  //gross approximaton.  Make tuning parameter figure out actual value.
        aircraftProperties.momentOfInertiaYaw = 0.02870341413f;  //gross approximaton.  Make tuning parameter figure out actual value.
        
        aircraftProperties.motor1LocationX = 0.0f;
        aircraftProperties.motor1LocationY = -0.200f;
        aircraftProperties.motor1LocationZ = 0.0f;

        aircraftProperties.motor2LocationX = 0.0f;
        aircraftProperties.motor2LocationY= 0.200f;
        aircraftProperties.motor2LocationZ = 0.0f;

        aircraftProperties.motor3LocationX = -0.305f;
        aircraftProperties.motor3LocationY = 0.0f;
        aircraftProperties.motor3LocationZ = 0.0f;


        aircraftProperties.centerOfMassLocationX = getCenterOfMassLocationX();//-0.053f;  //arewe using this or the aircraft properties?
        aircraftProperties.centerOfMassLocationY = 0.0f;
        aircraftProperties.centerOfMassLocationZ = 0.0f;

        aircraftProperties.elevon1LocationX = -0.14512f;
        aircraftProperties.elevon1LocationY = -0.1802f;

        aircraftProperties.elevon2LocationX = -0.14512f;
        aircraftProperties.elevon2LocationY =  0.1802f;


        effectors.elevon1Angle = 0.0f;
        effectors.elevon2Angle = 0.0f;
        effectors.tilt1Angle = 0.0f;
        effectors.tilt2Angle = 0.0f;
        effectors.motor1Thrust = 0.0f;
        effectors.motor2Thrust = 0.0f;
        effectors.motor3Thrust = 0.0f;

        state.regulatorMode = CONTROLLER_STATE_REGULATOR_MODE_RATE; //0 = none, 1 = passthrough, 2 = regulator on.
        state.commandMode = 0; //0 = Manual vectored thrust.
        state.armedState = 0; //0 = disarmed, 1 = armed.
    }


    /* Do not allow copies */
    BTOL_Controller(const BTOL_Controller &other) = delete;
    BTOL_Controller &operator=(const BTOL_Controller&) = delete;

    int16_t getRegulatorModeState(void);
    int16_t setRegulatorModeState(int16_t desiredState);
    int16_t getArmedState(void);
    int16_t setArmedState(int16_t desiredState);
    void setDesiredPitchAttitude(float pitchAttitudeTarget);
    void setDesiredRollAttitude(float rollAttitudeTarget);
    //void setDesiredYawRate(float yawRateTarget);
    void setDesiredAccelerationBodyX(float aX);
    void setDesiredAccelerationBodyZ(float aZ);
    void setDesiredTiltAngle(float tiltAngle);
    void setDesiredAccelerationAlongTiltAngle(float tiltAcceleration);
    float getRangeRatio(float value, float min, float max);

    void setDesiredPassthroughAngularAccelerationRoll(float waX); //rad/s/s
    void setDesiredPassthroughAngularAccelerationPitch(float waY); //rad/s/s
    void setDesiredPassthroughAngularAccelerationYaw(float waZ); //rad/s/s
    void setCommandedRollRate(float rollRate); //Rad/sec
    void setCommandedPitchRate(float pitchRate); //Rad/sec
    void setCommandedYawRate(float yawRate); //Rad/sec

    float getEstimatedDynamicPressure(void);
    float getInferredDynamicPressureFromTransitionRatio(float inferredTransitionRatio, float dynamicPressureAtTopOfTransition);
    float getInferredTransitionRatio(float verticalComponentOfAcceleration,  float verticalAccelerationThresholdForHover); //1.0 = transitioned, 0.0 = hover
    float getControlSurfaceForce(float deflectionAngleInRadians, float areaInM2, float dynamicPressureInPa);
    float getAugmentedStabilityRatioWithinAngleRange(float angleRad, float innerAngleRadPositive, float outerAngleRadPositive);

    int16_t calculateServoValueFromAngle(float desiredAngle, float minimumAngle, float maximumAngle, int16_t minimumPWM, int16_t maximumPWM);
    float calculateMotorThrustBasedOnTiltAngle(float attainedTiltAngle, float desiredForceForward, float desiredForceUp, float satisfactionAngleLow, float satisfactionAngleHigh); //use this function carefully to avoid div/0!

    EffectorList calculateEffectorPositions(float dt);

	static const struct AP_Param::GroupInfo var_info[];



    // tuning accessors
    //void kP(const float v) { testValue1.set(v); }
    //void kI(const float v) { testValue2.set(v); }
    //void kD(const float v) { testValue3.set(v); }
    //void kFF(const float v) { testValue4.set(v); }

   // AP_Float &kP(void) { return testValue1; }
    //AP_Float &kI(void) { return testValue2; }
    //AP_Float &kD(void) { return testValue3; }
   // AP_Float &kFF(void) { return testValue4; }



    AP_Float &getTiltCommandMappingPolarity(void) { return manualTiltCommandMappingPolarity; } 

    AP_Float &getRollRateCommandGain(void) { return rollRateCommandGain; } 
    AP_Float &getPitchRateCommandGain(void) { return pitchRateCommandGain; } 
    AP_Float &getYawRateCommandGain(void) { return yawRateCommandGain; } 
    AP_Float &getRollAttitudeCommandGain(void) { return rollAttitudeCommandGain; } 
    AP_Float &getPitchAttitudeCommandGain(void) { return pitchAttitudeCommandGain; } 

    AP_Float &getMotor12MaxThrust(void) { return motor12MaxThrust; } 
    AP_Float &getMotor3MaxThrust(void) { return motor3MaxThrust; } 

    AP_Float &getTopOfTransitionDynamicPressure(void) { return topOfTransitionDynamicPressure; } 
    AP_Float &getVerticalAccelerationThresholdToConsiderAircraftInHover(void) { return verticalAccelerationThresholdToConsiderAircraftInHover; } 
    AP_Float &getCenterOfMassLocationX(void) { return centerOfMassLocationX; } 
    AP_Float &getAircraftMass(void) { return aircraftMassInKg; } 

    // pid accessors
    AC_PID &get_rate_roll_pid() { return _pid_rate_roll; } //don't know what this does or why we need it.
    AC_PID &get_rate_pitch_pid() { return _pid_rate_pitch; }
    AC_PID &get_rate_yaw_pid() { return _pid_rate_yaw; }

private:
    AP_AHRS &_ahrs;
    const AP_Vehicle::FixedWing &aparm;
    //AP_AutoTune::ATGains gains;
    AP_Float rollRateCommandGain;
    AP_Float pitchRateCommandGain;
    AP_Float yawRateCommandGain;
    AP_Float rollAttitudeCommandGain;
    AP_Float pitchAttitudeCommandGain;
    AP_Float motor12MaxThrust;
    AP_Float motor3MaxThrust;

    AP_Float rollAttitudeErrorToRollRateGain;
    AP_Float pitchAttitudeErrorToPitchRateGain;
    AP_Float manualTiltCommandMappingPolarity;
    AP_Float topOfTransitionDynamicPressure;
    AP_Float verticalAccelerationThresholdToConsiderAircraftInHover;
    AP_Float aircraftMassInKg;
    AP_Float forwardMotorMaxThrust;
    AP_Float aftMotorMaxThrust;
    AP_Float centerOfMassLocationX; //in Meters
    AP_Float motor3ThrustToTorqueCoef; //N to NM
    AP_Float elevonCoefLiftPerDeflection;
    AP_Float elevonControlMinimumDynamicPressure; //Pa


    float pitchRateError;
    float rollRateError;
    float yawRateError;




    //From sub attitude control code...
    AC_PID   _pid_rate_roll;
    AC_PID   _pid_rate_pitch;
    AC_PID   _pid_rate_yaw;
    //AP_AutoTune autotune;
	uint32_t _last_t;
	float _last_out;

    EffectorList effectors;
    AircraftProperties aircraftProperties;
    ControllerState state;
    CommandInput command;

    //AP_Logger::PID_Info _pid_info;

	//int32_t _get_rate_out(float desired_rate, float scaler, bool disable_integrator);

};
