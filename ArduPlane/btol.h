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

struct MassProperties
{ 
    float motor1Thrust;

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
    void setDesiredYawRateAttitude(float yawRateTarget);
    //void setDesiredAccelerationX(float aX);
    //void setDesiredAccelerationZ(float aZ);

    EffectorList calculateEffectorPositions(void);


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


    //From sub attitude control code...
    AC_PID   _pid_rate_roll;
    AC_PID   _pid_rate_pitch;
    AC_PID   _pid_rate_yaw;
    //AP_AutoTune autotune;
	uint32_t _last_t;
	float _last_out;

    EffectorList effectors;

    //AP_Logger::PID_Info _pid_info;

	//int32_t _get_rate_out(float desired_rate, float scaler, bool disable_integrator);

};
