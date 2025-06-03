#include "Copter.h"

float roll_cmd, pitch_cmd, yaw_cmd, thr_cmd;

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void ModeStabilize::run()
{
    if (!motors->armed()) 
     { motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN); } // Motors should be Stopped
     else if (copter.ap.throttle_zero || (copter.air_mode == AirMode::AIRMODE_ENABLED && motors->get_spool_state() == AP_Motors::SpoolState::SHUT_DOWN))
     { motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE); }
     else { motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED); }
  
     switch (motors->get_spool_state()) { 
     case AP_Motors::SpoolState::SHUT_DOWN: // Motors Stopped
         attitude_control->reset_rate_controller_I_terms(); break; 
     case AP_Motors::SpoolState::GROUND_IDLE: // Landed
         attitude_control->reset_rate_controller_I_terms_smoothly(); break; 
     case AP_Motors::SpoolState::THROTTLE_UNLIMITED: 
     case AP_Motors::SpoolState::SPOOLING_UP: 
     case AP_Motors::SpoolState::SPOOLING_DOWN: break; }
  
     attitude_control->pend_attitude_controller_run(roll_cmd, pitch_cmd, yaw_cmd, thr_cmd); // 运行姿态控制器
}

void ModeStabilize::set_rate(const Vector3f &rate_cmd, float thrust_cmd)
{
    roll_cmd  = rate_cmd.x;
    pitch_cmd = rate_cmd.y;
    yaw_cmd   = rate_cmd.z;
    thr_cmd   = thrust_cmd;
}
