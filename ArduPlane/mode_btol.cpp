#include "mode.h"
#include "Plane.h"

//Copied from Stablized.

bool ModeBTOL::_enter()
{
    plane.throttle_allows_nudging = false;
    plane.auto_throttle_mode = false;
    plane.auto_navigation_mode = false;

    //Send text to the groundstation
    //gcs().send_text(MAV_SEVERITY_CRITICAL, "BTOL ENTERED!");// %5.3f", (double)3.142f);

   //    MAV_SEVERITY_EMERGENCY=0, /* System is unusable. This is a "panic" condition. | */
   //MAV_SEVERITY_ALERT=1, /* Action should be taken immediately. Indicates error in non-critical systems. | */
   //MAV_SEVERITY_CRITICAL=2, /* Action must be taken immediately. Indicates failure in a primary system. | */
   //MAV_SEVERITY_ERROR=3, /* Indicates an error in secondary/redundant systems. | */
   //MAV_SEVERITY_WARNING=4, /* Indicates about a possible future error if this is not resolved within a given timeframe. Example would be a low battery warning. | */
   //MAV_SEVERITY_NOTICE=5, /* An unusual event has occurred, though not an error condition. This should be investigated for the root cause. | */
   //MAV_SEVERITY_INFO=6, /* Normal operational messages. Useful for logging. No action is required for these messages. | */
   //MAV_SEVERITY_DEBUG=7, /* Useful non-operational messages that can assist in debugging. These should not occur during normal operation. | */
   //MAV_SEVERITY_ENUM_END=8, /*  | */
   gcs().send_text(MAV_SEVERITY_INFO, "BTOL STARTED");// %5.3f", (double)3.142f);
   

    return true;
}

void ModeBTOL::update()
{

    /*SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, plane.channel_roll->get_control_in_zero_dz());
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, plane.channel_pitch->get_control_in_zero_dz());
    SRV_Channels::set_output_scaled(SRV_Channel::k_flap, plane.channel_roll->get_control_in_zero_dz());
    SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, plane.channel_rudder->get_control_in_zero_dz());

    plane.steering_control.steering = plane.steering_control.rudder = plane.channel_rudder->get_control_in_zero_dz();


    static int16_t servoControlValue1 = 0;
    #define SERVO_CONTROL_VALUE_1_HALF_WIDTH 500
    #define SERVO_CONTROL_CENTER_VALUE 1500
    #define DELTA_VALUE_PER_TIMESTEP 10 //(50Hz)?

    servoControlValue1 = servoControlValue1 + DELTA_VALUE_PER_TIMESTEP;
    if(servoControlValue1 > SERVO_CONTROL_VALUE_1_HALF_WIDTH)
    {
        servoControlValue1 = -SERVO_CONTROL_VALUE_1_HALF_WIDTH;
    }
    servoControlValue1 = servoControlValue1 + SERVO_CONTROL_CENTER_VALUE;
    SRV_Channels::set_output_pwm(SRV_Channel::k_elevon_right, servoControlValue1);  //this doesn't work.
    //SRV_Channels::set_output_scaled(SRV_Channel::k_mount_pan, servoControlValue1);
    */



/*
    //plane.nav_roll_cd = 0;
    //plane.nav_pitch_cd = 0;

      //SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, plane.channel_rudder->get_control_in_zero_dz());
      //SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, plane.channel_rudder->get_control_in_zero_dz());
      //plane.steering_control.steering = plane.steering_control.rudder = plane.channel_rudder->get_control_in_zero_dz();
    
     // SRV_Channels::set_output_scaled(SRV_Channel::k_motor1, plane.cha);
    //  SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, );

    //SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, plane.channel_roll->get_control_in_zero_dz());

    static int16_t servoControlValue1 = 0;
    #define SERVO_CONTROL_VALUE_1_HALF_WIDTH 4500
    #define SERVO_CONTROL_CENTER_VALUE 0
    #define DELTA_VALUE_PER_TIMESTEP 10 //(50Hz)?
    servoControlValue1 = servoControlValue1 + DELTA_VALUE_PER_TIMESTEP;

    if(servoControlValue1 > SERVO_CONTROL_VALUE_1_HALF_WIDTH)
    {
        servoControlValue1 = -SERVO_CONTROL_VALUE_1_HALF_WIDTH;
    }
    servoControlValue1 = servoControlValue1 + SERVO_CONTROL_CENTER_VALUE;
    //SRV_Channels::set_output_pwm(SRV_Channel::k_flap, servoControlValue1);  //this doesn't work.
    SRV_Channels::set_output_scaled(SRV_Channel::k_flap, servoControlValue1);


    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, plane.channel_roll->get_control_in_zero_dz());



    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, plane.channel_pitch->get_control_in_zero_dz());
    //SRV_Channel::set_angle()

//Testing non primary (aux channels)
    SRV_Channels::set_output_scaled(SRV_Channel::k_mount_pan, plane.channel_rudder->get_control_in_zero_dz());




   // SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, plane.channel_pitch->get_control_in());


          // set output value for a specific function channel as a pwm value
        //  rc().read_input();

       //SRV_Channels::set_output_pwm_chan(1, rc().channel(2)->get_control_in());

      //SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, rollController.get_servo_out(nav_roll_cd - ahrs.roll_sensor, 
                                                                                  //       speed_scaler, 
                                                                                   //      disable_integrator));



    //plane.a
    #if BTOL_ENABLED == ENABLED
    // for ArduSoar soaring_controller
    //plane.g2.soaring_controller.init_cruising();
    //plane.update_btol();




    #endif
*/
/*
    plane.ahrs.getAOA(); //test





    //plane.training_manual_roll = false;
    //plane.training_manual_pitch = false;
    plane.update_load_factor();

    // if the roll is past the set roll limit, then
    // we set target roll to the limit
    if (plane.ahrs.roll_sensor >= plane.roll_limit_cd) {
        plane.nav_roll_cd = plane.roll_limit_cd;
    } else if (plane.ahrs.roll_sensor <= -plane.roll_limit_cd) {
        plane.nav_roll_cd = -plane.roll_limit_cd;
    } else {
        plane.training_manual_roll = true;
        plane.nav_roll_cd = 0;
    }

    // if the pitch is past the set pitch limits, then
    // we set target pitch to the limit
    if (plane.ahrs.pitch_sensor >= plane.aparm.pitch_limit_max_cd) {
        plane.nav_pitch_cd = plane.aparm.pitch_limit_max_cd;
    } else if (plane.ahrs.pitch_sensor <= plane.pitch_limit_min_cd) {
        plane.nav_pitch_cd = plane.pitch_limit_min_cd;
    } else {
        plane.training_manual_pitch = true;
        plane.nav_pitch_cd = 0;
    }
    if (plane.fly_inverted()) {
        plane.nav_pitch_cd = -plane.nav_pitch_cd;
    }
    */
    
}

