#pragma once

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Common/AP_Common.h>
#include <AP_Vehicle/AP_Vehicle.h>
//#include "AP_AutoTune.h"
#include <AP_Logger/AP_Logger.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

class BTOL_Controller {
public:
    //https://www.geeksforgeeks.org/when-do-we-use-initializer-list-in-c/
    BTOL_Controller(AP_AHRS &ahrs, const AP_Vehicle::FixedWing &parms): _ahrs(ahrs), aparm(parms)
    {
        AP_Param::setup_object_defaults(this, var_info);
    }

    /* Do not allow copies */
    BTOL_Controller(const BTOL_Controller &other) = delete;
    BTOL_Controller &operator=(const BTOL_Controller&) = delete;

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

private:
    AP_AHRS &_ahrs;
    const AP_Vehicle::FixedWing &aparm;
    //AP_AutoTune::ATGains gains;
    AP_Float testValue1;
    AP_Float testValue2;
    AP_Float testValue3;
    AP_Float testValue4;
    //AP_AutoTune autotune;
	uint32_t _last_t;
	float _last_out;

    //AP_Logger::PID_Info _pid_info;

	//int32_t _get_rate_out(float desired_rate, float scaler, bool disable_integrator);

};
