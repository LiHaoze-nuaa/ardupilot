#include "Copter.h"

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
 
    attitude_control->falcon_attitude_controller_run(); // 运行姿态控制器
}
