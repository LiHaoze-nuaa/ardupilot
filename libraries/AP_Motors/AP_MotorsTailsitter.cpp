// AP_MotorsTailsitter.cpp - ArduCopter motors library for tailsitters and bicopters

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "AP_MotorsTailsitter.h"
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>
#include <RC_Channel/RC_Channel.h>
#include <AC_AttitudeControl/AC_AttitudeControl.h>

extern const AP_HAL::HAL& hal;
#define SERVO_OUTPUT_RANGE  4500

// init
void AP_MotorsTailsitter::init(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // setup default motor and servo mappings
    _has_diff_thrust = SRV_Channels::function_assigned(SRV_Channel::k_throttleRight) || SRV_Channels::function_assigned(SRV_Channel::k_throttleLeft);

    // right throttle defaults to servo output 1
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_throttleRight, CH_1);

    // left throttle defaults to servo output 2
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_throttleLeft, CH_2);

    // right servo defaults to servo output 3
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_tiltMotorRight, CH_3);
    SRV_Channels::set_angle(SRV_Channel::k_tiltMotorRight, SERVO_OUTPUT_RANGE);

    // left servo defaults to servo output 4
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_tiltMotorLeft, CH_4);
    SRV_Channels::set_angle(SRV_Channel::k_tiltMotorLeft, SERVO_OUTPUT_RANGE);

    _mav_type = MAV_TYPE_VTOL_DUOROTOR;

    // record successful initialisation if what we setup was the desired frame_class
    set_initialised_ok(frame_class == MOTOR_FRAME_TAILSITTER);
    FOCCAN = AP_FOCCAN::get_singleton(); // 很关键
}


/// Constructor
AP_MotorsTailsitter::AP_MotorsTailsitter(uint16_t speed_hz) :
    AP_MotorsMulticopter(speed_hz)
{
    set_update_rate(speed_hz);
}


// set update rate to motors - a value in hertz
void AP_MotorsTailsitter::set_update_rate(uint16_t speed_hz)
{
    // record requested speed
    _speed_hz = speed_hz;

    SRV_Channels::set_rc_frequency(SRV_Channel::k_throttleLeft, speed_hz);
    SRV_Channels::set_rc_frequency(SRV_Channel::k_throttleRight, speed_hz);
}

void AP_MotorsTailsitter::output_to_motors()
{
    if (!initialised_ok()) {
        return;
    }

    switch (_spool_state) {
        case SpoolState::SHUT_DOWN:
            _actuator[0] = 0.0f;
            _actuator[1] = 0.0f;
            _actuator[2] = 0.0f;
            _external_min_throttle = 0.0;
            break;
        case SpoolState::GROUND_IDLE:
            set_actuator_with_slew(_actuator[0], actuator_spin_up_to_ground_idle());
            set_actuator_with_slew(_actuator[1], actuator_spin_up_to_ground_idle());
            set_actuator_with_slew(_actuator[2], actuator_spin_up_to_ground_idle());
            _external_min_throttle = 0.0;
            break;
        case SpoolState::SPOOLING_UP:
        case SpoolState::THROTTLE_UNLIMITED:
        case SpoolState::SPOOLING_DOWN:
            set_actuator_with_slew(_actuator[0], thr_lin.thrust_to_actuator(_thrust_left));
            set_actuator_with_slew(_actuator[1], thr_lin.thrust_to_actuator(_thrust_right));
            set_actuator_with_slew(_actuator[2], thr_lin.thrust_to_actuator(_throttle));
            break;
    }

    SRV_Channels::set_output_pwm(SRV_Channel::k_throttleLeft, output_to_pwm(_actuator[0]));
    SRV_Channels::set_output_pwm(SRV_Channel::k_throttleRight, output_to_pwm(_actuator[1]));

    // use set scaled to allow a different PWM range on plane forward throttle, throttle range is 0 to 100
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, _actuator[2]*100);

    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, _tilt_left*SERVO_OUTPUT_RANGE);
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, _tilt_right*SERVO_OUTPUT_RANGE);

}

// get_motor_mask - returns a bitmask of which outputs are being used for motors (1 means being used)
//  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
uint32_t AP_MotorsTailsitter::get_motor_mask()
{
    uint32_t motor_mask = 0;
    uint8_t chan;
    if (SRV_Channels::find_channel(SRV_Channel::k_throttleLeft, chan)) {
        motor_mask |= 1U << chan;
    }
    if (SRV_Channels::find_channel(SRV_Channel::k_throttleRight, chan)) {
        motor_mask |= 1U << chan;
    }

    // add parent's mask
    motor_mask |= AP_MotorsMulticopter::get_motor_mask();

    return motor_mask;
}

// calculate outputs to the motors
void AP_MotorsTailsitter::output_armed_stabilizing()
{
    float   roll_thrust;                // roll thrust input value, +/- 1.0
    float   pitch_thrust;               // pitch thrust input value, +/- 1.0
    float   yaw_thrust;                 // yaw thrust input value, +/- 1.0
    float   throttle_thrust;            // throttle thrust input value, 0.0 - 1.0

    // apply voltage and air pressure compensation
    uint16_t rcin[8] = {};
    rc().get_radio_in (rcin, 8); // 获取遥控器数据
    roll_thrust = _roll_in;
    pitch_thrust = _pitch_in;
    yaw_thrust = _yaw_in;
    
    #ifdef VECTOR_MODE // 矢量模式
    if ( rcin[4] < 1200) { // 自稳模式
        throttle_thrust = (rcin[2] - 1100) * 0.001f; }
    else if ( rcin[4] >= 1200) { // MPC模式
        throttle_thrust = thr_mpc; }
    #else // 力矩摆模式
    float forward_vector;    
    if ( rcin[4] < 1200) { // 自稳模式
        forward_vector = (rcin[1] - 1500) * 0.001f;
        throttle_thrust = (rcin[2] - 1100) * 0.001f; }
    else if ( rcin[4] >= 1200) { // MPC模式
        forward_vector = vct_mpc;
        throttle_thrust = thr_mpc; }
    pend_current = (int16_t)(-pitch_thrust*600); // 力矩摆电流信号处理
    if ( pend_current >  1 ) { pend_current += 25; } // 无响应死区截断
    if ( pend_current < -1 ) { pend_current -= 25; }
    // const AP_AHRS &ahrs_pend = AP::ahrs();
    // const float pth = ahrs_pend.get_pitch();
    if ( rcin[2] < 1200 ) { pend_current = 0; }
    #endif

    if (throttle_thrust <= 0.0f) 
    { throttle_thrust = 0.0f; limit.throttle_lower = true; }
    if (throttle_thrust >= 1.0f) 
    { throttle_thrust = 1.0f; limit.throttle_upper = true; }    
        
    // 低通滤波
    _roll_filt  = 0.5f * _roll_filt  + 0.5f * roll_thrust;
    _pitch_filt = 0.5f * _pitch_filt + 0.5f * pitch_thrust;
    _yaw_filt   = 0.5f * _yaw_filt   + 0.5f * yaw_thrust;
    roll_thrust  = _roll_filt;
    pitch_thrust = _pitch_filt;
    yaw_thrust   = _yaw_filt;

    #ifdef VECTOR_MODE // 矢量模式控制分配
    _thrust_left  = throttle_thrust + roll_thrust * 0.5f;
    _thrust_right = throttle_thrust - roll_thrust * 0.5f;
    _tilt_left  = pitch_thrust - yaw_thrust;
    _tilt_right = pitch_thrust + yaw_thrust;
    #else // 力矩摆模式控制分配
    _thrust_left  = throttle_thrust + roll_thrust * 0.5f;
    _thrust_right = throttle_thrust - roll_thrust * 0.5f;
    _tilt_left  = forward_vector - yaw_thrust;
    _tilt_right = forward_vector + yaw_thrust;
    FOCCAN->setCurrent(pend_current);
    #endif

    _thrust_left  = constrain_float(_thrust_left , 0.0f, 1.0f);
    _thrust_right = constrain_float(_thrust_right, 0.0f, 1.0f);
    _tilt_left  = constrain_float(_tilt_left , -1.0f, 1.0f);
    _tilt_right = constrain_float(_tilt_right, -1.0f, 1.0f);
    _throttle = throttle_thrust; 
    _throttle_out = throttle_thrust;
}

// output_test_seq - spin a motor at the pwm value specified
//  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
//  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
void AP_MotorsTailsitter::_output_test_seq(uint8_t motor_seq, int16_t pwm)
{
    // output to motors and servos
    switch (motor_seq) {
        case 1:
            // right throttle
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleRight, pwm);
            break;
        case 2:
            // right tilt servo
            SRV_Channels::set_output_pwm(SRV_Channel::k_tiltMotorRight, pwm);
            break;
        case 3:
            // left throttle
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleLeft, pwm);
            break;
        case 4:
            // left tilt servo
            SRV_Channels::set_output_pwm(SRV_Channel::k_tiltMotorLeft, pwm);
            break;
        default:
            // do nothing
            break;
    }
}
