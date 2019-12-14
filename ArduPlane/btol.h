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

#define AC_ATC_HELI_RATE_RP_P                       0.024f
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
#define AC_ATC_HELI_RATE_YAW_FILT_HZ                20.0f
#define AC_ATTITUDE_HELI_RATE_RP_FF_FILTER          10.0f
#define AC_ATTITUDE_HELI_RATE_Y_VFF_FILTER          10.0f
#define TEMP_DT                                     0.0025f

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


    //TODO: add max/min and rates.
};  


class BTOL_Controller {
public:
    //https://www.geeksforgeeks.org/when-do-we-use-initializer-list-in-c/
    BTOL_Controller(AP_AHRS &ahrs, const AP_Vehicle::FixedWing &parms): 
    _ahrs(ahrs), 
    aparm(parms),
    _pid_rate_roll(AC_ATC_HELI_RATE_RP_P, AC_ATC_HELI_RATE_RP_I, AC_ATC_HELI_RATE_RP_D, AC_ATC_HELI_RATE_RP_FF, AC_ATC_HELI_RATE_RP_IMAX, AC_ATTITUDE_HELI_RATE_RP_FF_FILTER, AC_ATC_HELI_RATE_RP_FILT_HZ, 0.0f, TEMP_DT),
    _pid_rate_pitch(AC_ATC_HELI_RATE_RP_P, AC_ATC_HELI_RATE_RP_I, AC_ATC_HELI_RATE_RP_D, AC_ATC_HELI_RATE_RP_FF, AC_ATC_HELI_RATE_RP_IMAX, AC_ATTITUDE_HELI_RATE_RP_FF_FILTER, AC_ATC_HELI_RATE_RP_FILT_HZ, 0.0f, TEMP_DT),
    _pid_rate_yaw(AC_ATC_HELI_RATE_YAW_P, AC_ATC_HELI_RATE_YAW_I, AC_ATC_HELI_RATE_YAW_D, AC_ATC_HELI_RATE_YAW_FF, AC_ATC_HELI_RATE_YAW_IMAX, AC_ATTITUDE_HELI_RATE_Y_VFF_FILTER, AC_ATC_HELI_RATE_YAW_FILT_HZ, 0.0f, TEMP_DT)
    {
        AP_Param::setup_object_defaults(this, var_info);
        targetPitchAttitude = 0.0f;
        targetRollAttitude = 0.0f;
        targetPitchRate = 0.0f;
        targetRollRate = 0.0f;
        targetYawRate = 0.0f;
        targetHeading = 0.0f;
        pitchRateError = 0.0f;
        rollRateError = 0.0f;
        yawRateError = 0.0f;

        targetAccelerationZ = 0.0f;
        targetAccelerationX = 0.0f;
        passthroughAngularAccelerationRoll = 0.0f;
        passthroughAngularAccelerationPitch = 0.0f;
        passthroughAngularAccelerationYaw = 0.0f;

        aircraftProperties.totalMass = 1.0;  //approximaton.  Make tuning parameter figure out actual value.
        aircraftProperties.momentOfInertiaPitch = 1.0;  //gross approximaton.  Make tuning parameter figure out actual value.
        aircraftProperties.momentOfInertiaRoll = 1.0;  //gross approximaton.  Make tuning parameter figure out actual value.
        aircraftProperties.momentOfInertiaYaw = 1.0;  //gross approximaton.  Make tuning parameter figure out actual value.
        
        aircraftProperties.motor1LocationX = 0.0f;
        aircraftProperties.motor1LocationY = -0.200f;
        aircraftProperties.motor1LocationZ = 0.0f;

        aircraftProperties.motor2LocationX = 0.0f;
        aircraftProperties.motor2LocationY= 0.200f;
        aircraftProperties.motor2LocationZ = 0.0f;

        aircraftProperties.motor3LocationX = -0.305f;
        aircraftProperties.motor3LocationY = 0.0f;
        aircraftProperties.motor3LocationZ = 0.0f;


        aircraftProperties.centerOfMassLocationX = -0.053f;
        aircraftProperties.centerOfMassLocationY = 0.0f;
        aircraftProperties.centerOfMassLocationZ = 0.0f;

        effectors.elevon1Angle = 0.0f;
        effectors.elevon2Angle = 0.0f;
        effectors.tilt1Angle = 0.0f;
        effectors.tilt2Angle = 0.0f;
        effectors.motor1Thrust = 0.0f;
        effectors.motor2Thrust = 0.0f;
        effectors.motor3Thrust = 0.0f;
    }


    /* Do not allow copies */
    BTOL_Controller(const BTOL_Controller &other) = delete;
    BTOL_Controller &operator=(const BTOL_Controller&) = delete;

    void setDesiredPitchAttitude(float pitchAttitudeTarget);
    void setDesiredRollAttitude(float rollAttitudeTarget);
    void setDesiredYawRate(float yawRateTarget);
    void setDesiredAccelerationBodyX(float aX);
    void setDesiredAccelerationBodyZ(float aZ);
    void setDesiredPassthroughAngularAccelerationRoll(float waX);
    void setDesiredPassthroughAngularAccelerationPitch(float waY);
    void setDesiredPassthroughAngularAccelerationYaw(float waZ);
    int16_t calculateServoValueFromAngle(float desiredAngle, float minimumAngle, float maximumAngle, int16_t minimumPWM, int16_t maximumPWM);
    float calculateMotorThrustBasedOnTiltAngle(float attainedTiltAngle, float desiredForceForward, float desiredForceUp, float satisfactionAngleLow, float satisfactionAngleHigh); //use this function carefully to avoid div/0!

    EffectorList calculateEffectorPositions(float dt);


    //void setDesiredPitchRate

//	int32_t get_rate_out(float desired_rate, float scaler);
//	int32_t get_servo_out(int32_t angle_err, float scaler, bool disable_integrator);

//	void reset_I();

    /*
      reduce the integrator, used when we have a low scale factor in a quadplane hover
    */
    void decay_I() {
        // this reduces integrator by 95% over 2s
      //  _pid_info.I *= 0.995f;
    }
    
    //void autotune_start(void) { autotune.start(); }
    //void autotune_restore(void) { autotune.stop(); }

    //const       AP_Logger::PID_Info& get_pid_info(void) const { return _pid_info; }

	static const struct AP_Param::GroupInfo var_info[];


    // tuning accessors
    void kP(const float v) { testValue1.set(v); }
    void kI(const float v) { testValue2.set(v); }
    void kD(const float v) { testValue3.set(v); }
    void kFF(const float v) { testValue4.set(v); }

    AP_Float &kP(void) { return testValue1; }
    AP_Float &kI(void) { return testValue2; }
    AP_Float &kD(void) { return testValue3; }
    AP_Float &kFF(void) { return testValue4; }
        // pid accessors

private:
    AP_AHRS &_ahrs;
    const AP_Vehicle::FixedWing &aparm;
    //AP_AutoTune::ATGains gains;
    AP_Float testValue1;
    AP_Float testValue2;
    AP_Float testValue3;
    AP_Float testValue4;


    float targetPitchAttitude;
    float targetRollAttitude;
    float targetPitchRate;
    float targetRollRate;
    float targetYawRate;
    float targetHeading;

    float pitchRateError;
    float rollRateError;
    float yawRateError;

    float targetAccelerationZ;
    float targetAccelerationX;
    float passthroughAngularAccelerationRoll;
    float passthroughAngularAccelerationPitch;
    float passthroughAngularAccelerationYaw;


    //From sub attitude control code...
    AC_PID   _pid_rate_roll;
    AC_PID   _pid_rate_pitch;
    AC_PID   _pid_rate_yaw;
    //AP_AutoTune autotune;
	uint32_t _last_t;
	float _last_out;

    EffectorList effectors;
    AircraftProperties aircraftProperties;

    //AP_Logger::PID_Info _pid_info;

	//int32_t _get_rate_out(float desired_rate, float scaler, bool disable_integrator);

};
