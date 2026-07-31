// AP_MotorsSingle.cpp - ArduCopter motors library
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "AP_MotorsSingle.h"
#include <GCS_MAVLink/GCS.h>
extern const AP_HAL::HAL& hal;

// init
void AP_MotorsSingle::init(motor_frame_class frame_class, motor_frame_type frame_type)
{   
    for (uint8_t i = 0; i < 4; i++) { add_motor_num(CH_1 + i); }
    motor_enabled[AP_MOTORS_MOT_4] = true;
    for (uint8_t i = 0; i < NUM_SERVOS; i++)
    { SRV_Channels::set_angle(SRV_Channels::get_motor_function(i), AP_MOTORS_SINGLE_SERVO_INPUT_RANGE); }
    _flags.initialised_ok = (frame_class == MOTOR_FRAME_SINGLE);
}

// set frame class (i.e. quad, hexa, heli) and type (i.e. x, plus)
void AP_MotorsSingle::set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type){ }

// set update rate to motors - a value in hertz
void AP_MotorsSingle::set_update_rate(uint16_t speed_hz)
{   
    _speed_hz = speed_hz;     // record requested speed
    uint32_t mask = 1U << AP_MOTORS_MOT_4 ;
    rc_set_freq(mask, _speed_hz);
}

void AP_MotorsSingle::output_to_motors()
{   
    if (!_flags.initialised_ok) { return; }
    switch (_spool_state) {
        case SpoolState::SHUT_DOWN:
            rc_write_angle(AP_MOTORS_MOT_1, _roll_radio_passthrough  * AP_MOTORS_SINGLE_SERVO_INPUT_RANGE);
            rc_write_angle(AP_MOTORS_MOT_2, _pitch_radio_passthrough * AP_MOTORS_SINGLE_SERVO_INPUT_RANGE);
            rc_write_angle(AP_MOTORS_MOT_3, _yaw_radio_passthrough   * AP_MOTORS_SINGLE_SERVO_INPUT_RANGE);
            rc_write(AP_MOTORS_MOT_4, output_to_pwm(0));
            break;
        case SpoolState::GROUND_IDLE:
            for (uint8_t i = 0; i < NUM_SERVOS; i++)
            { rc_write_angle(AP_MOTORS_MOT_1 + i, _spin_up_ratio * _actuator_out[i] * AP_MOTORS_SINGLE_SERVO_INPUT_RANGE); }
            set_actuator_with_slew(_actuator[4], actuator_spin_up_to_ground_idle());
            rc_write(AP_MOTORS_MOT_4, output_to_pwm(_actuator[4]));
            break;
        case SpoolState::SPOOLING_UP:
        case SpoolState::THROTTLE_UNLIMITED:
        case SpoolState::SPOOLING_DOWN:
            for (uint8_t i = 0; i < NUM_SERVOS; i++)
            { rc_write_angle(AP_MOTORS_MOT_1 + i, _actuator_out[i] * AP_MOTORS_SINGLE_SERVO_INPUT_RANGE); }
            set_actuator_with_slew(_actuator[4], thrust_to_actuator(_thrust_out));
            rc_write(AP_MOTORS_MOT_4, output_to_pwm(_actuator[4]));
            break;  }
}

// get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
//  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
uint16_t AP_MotorsSingle::get_motor_mask()
{   
    uint32_t motor_mask = 1U << AP_MOTORS_MOT_1 | 1U << AP_MOTORS_MOT_2 | 1U << AP_MOTORS_MOT_3 | 1U << AP_MOTORS_MOT_4 ;
    uint16_t mask = rc_map_mask(motor_mask);
    mask |= AP_MotorsMulticopter::get_motor_mask();
    return mask;
}

// sends commands to the motors
void AP_MotorsSingle::output_armed_stabilizing()
{   
    float   roll_thrust;                // roll thrust input value, +/- 1.0
    float   pitch_thrust;               // pitch thrust input value, +/- 1.0
    float   yaw_thrust;                 // yaw thrust input value, +/- 1.0
    float   throttle_thrust;            // throttle thrust input value, 0.0 - 1.0
    const float compensation_gain = get_compensation_gain();

    roll_thrust = (_roll_in + _roll_in_ff) * compensation_gain;
    pitch_thrust = (_pitch_in + _pitch_in_ff) * compensation_gain;
    yaw_thrust = (_yaw_in + _yaw_in_ff) * compensation_gain;
    throttle_thrust = get_throttle() * compensation_gain;

    if (throttle_thrust <= 0.0f)
    { throttle_thrust = 0.0f; limit.throttle_lower = true; }
    if (throttle_thrust >= _throttle_thrust_max)
    { throttle_thrust = _throttle_thrust_max; limit.throttle_upper = true; }

    _thrust_out = throttle_thrust / compensation_gain;
    _actuator_out[0] = roll_thrust  ;
    _actuator_out[1] = pitch_thrust ;
    _actuator_out[2] = yaw_thrust   ;
}

// output_test_seq - spin a motor at the pwm value specified
//  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
//  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
void AP_MotorsSingle::output_test_seq(uint8_t motor_seq, int16_t pwm)
{   
    if (!armed()) {  return;  }                         // exit immediately if not armed
    switch (motor_seq) {                                // output to motors and servos
    case 1:  rc_write(AP_MOTORS_MOT_1, pwm);  break;
    case 2:  rc_write(AP_MOTORS_MOT_2, pwm);  break;
    case 3:  rc_write(AP_MOTORS_MOT_3, pwm);  break;
    case 4:  rc_write(AP_MOTORS_MOT_4, pwm);  break;
    default:                                  break; }
}
