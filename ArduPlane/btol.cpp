#include "Plane.h"
#include "btol.h"
#include <utility>
#if BTOL_ENABLED == ENABLED
//extern const AP_HAL::HAL& hal;

/*  //already in Plane.h
#include <AP_HAL/AP_HAL.h>

#if HAL_OS_POSIX_IO
#include <stdio.h>
#endif
*/ 
/*
*  ArduSoar support function
*
*  Peter Braswell, Samuel Tabor, Andrey Kolobov, and Iain Guilliard
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

static float rcCommandInputPitchStickAft = 0;
static float rcCommandInputRollStickRight = 0;
static float rcCommandInputYawStickRight = 0;
static float rcCommandInputThrottleStickForward = 0;



void Plane::update_btol() {  //50Hz
    //take pilot input
    //get the rc input and map them to a range of -1.0 to +1.0;
    //TODO: Add in a something which zeros the values if the incoming raw signal is zero.
    rcCommandInputPitchStickAft = constrain_float( ((float)hal.rcin->read(RC_CHANNEL_NUMBER_FOR_PITCH_STICK) - RC_CHANNEL_INPUT_CENTER_VALUE)/RC_CHANNEL_INPUT_HALF_RANGE, -1.0, 1.0);  //this should be a function, private
    rcCommandInputRollStickRight = constrain_float( ((float)hal.rcin->read(RC_CHANNEL_NUMBER_FOR_PITCH_STICK) - RC_CHANNEL_INPUT_CENTER_VALUE)/RC_CHANNEL_INPUT_HALF_RANGE, -1.0, 1.0); 
    rcCommandInputYawStickRight = constrain_float( ((float)hal.rcin->read(RC_CHANNEL_NUMBER_FOR_PITCH_STICK) - RC_CHANNEL_INPUT_CENTER_VALUE)/RC_CHANNEL_INPUT_HALF_RANGE, -1.0, 1.0); 
    rcCommandInputThrottleStickForward = constrain_float( ((float)hal.rcin->read(RC_CHANNEL_NUMBER_FOR_PITCH_STICK) - RC_CHANNEL_INPUT_CENTER_VALUE)/RC_CHANNEL_INPUT_HALF_RANGE, -1.0, 1.0); 

    //calculate desired attiutdes and rates

    //set control parameter targets


   //gcs().send_text(MAV_SEVERITY_INFO, "UPD BTOL");// %5.3f", (double)3.142f);  //Checked, this is getting called.



  //DOESN'T SEEM TO WORK hal.console->printf("RC: %i\n", plane.channel_rudder->get_control_in_zero_dz());

 //   SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, plane.channel_rudder->get_control_in_zero_dz());
 //   SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, plane.channel_rudder->get_control_in_zero_dz());

   //Doesn't work printf(" test2\n");
    //doesn't work ::printf(" test\n");
 

   //hal.console->printf("test4");  //didn't work.

    //plane.steering_control.steering = plane.steering_control.rudder = plane.channel_rudder->get_control_in_zero_dz();

    



}

void Plane::initialize_btol(){


    hal.rcout->set_default_rate(50);
    hal.rcout->enable_ch(CH_1);
    hal.rcout->enable_ch(CH_2);
    hal.rcout->enable_ch(CH_3);
    hal.rcout->enable_ch(CH_4);
    hal.rcout->enable_ch(CH_5);
    hal.rcout->enable_ch(CH_6);
    hal.rcout->enable_ch(CH_7);
    hal.rcout->enable_ch(CH_8);
     //   hal.rcout->set_default_rate(50);
   // hal.rcout->set_output_mode(CH_1, 
   hal.console->printf("initialize_btol\n");
}

 //Presently running at 400Hz.  See ArduPlane.cpp :   SCHED_TASK(btol_stabilize,         400,   200),  //blake added.
void Plane::btol_stabilize() {   

    //Plane.ahrs.roll_sensor 
    //ahrs.get_roll();  //    // integer Euler angles (Degrees * 100)   //int32_t roll_sensor; 
	static uint32_t _last_t;
    uint32_t tnow = AP_HAL::millis();
	uint32_t dt = tnow - _last_t;
	if (_last_t == 0 || dt > 1000) {
		dt = 0;
	}
	_last_t = tnow;

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
    int16_t servoControlValue1 = SERVO_CONTROL_CENTER_VALUE + deltaAmount;
   // SRV_Channels::set_output_pwm_chan(4, servoControlValue1);  //this doesn't work.

    //SRV_Channels::output_ch_all(); //this 
 //   hal.rcout->enable_ch(0);
 //   hal.rcout->enable_ch(1);
 //   hal.rcout->enable_ch(2);
 //   hal.rcout->enable_ch(3);
 //   hal.rcout->set_default_rate(50);
   // hal.rcout->set_output_mode()

    // Get body rate vector (radians/sec)
	//float omega_x = _ahrs.get_gyro().x;
    float omega_x =  ahrs.get_gyro().x;
    
    int16_t servoControlValue2 = SERVO_CONTROL_CENTER_VALUE + constrain_int16(int16_t(omega_x*500), -500, 500);  //doesn't work
    int16_t servoControlValue3 = SERVO_CONTROL_CENTER_VALUE + constrain_int16(int16_t(ahrs.get_yaw_rate_earth()*500), -500, 500);  //works  (float)
    int16_t servoControlValue4 = SERVO_CONTROL_CENTER_VALUE + constrain_int16(int16_t(ahrs.pitch*500), -500, 500);  //works (float)
    int16_t servoControlValue5 = SERVO_CONTROL_CENTER_VALUE + constrain_int16(int16_t(rcCommandInputPitchStickAft*500), -500, 500);  //works (float)

    hal.rcout->cork();  //  SRV_Channels::cork();
    hal.rcout->write(CH_1, servoControlValue1);
    hal.rcout->write(CH_2, servoControlValue2);  //not working...perhaps base zero??
    hal.rcout->write(CH_3, servoControlValue3);
    hal.rcout->write(CH_4, servoControlValue4);
    hal.rcout->write(CH_5, servoControlValue5);
    hal.rcout->write(CH_6, 1600);
    hal.rcout->write(CH_7, 1700);
    hal.rcout->write(CH_8, 1800);
    hal.rcout->push();  //will need to use: SRV_Channels::push(); or parts of it when we use BL Heli or D-shot!
}

#endif //BTOL_ENABLED
