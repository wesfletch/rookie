/**
 * @file motors.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-03-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef MOTORS_H
#define MOTORS_H

// We are using pins 0 and 1, but see the GPIO function select table in the
// datasheet for information on which other pins can be used.
#define PWM_1_PIN           16
#define PWM_2_PIN           17
#define DIR_1_PIN           18
#define DIR_2_PIN           19

// function prototypes
void right_enc_callback(uint gpio, uint32_t events);
int configure_PWM();
float get_vel_left();
float get_vel_right();
void set_PWM(bool left_dir, int left_speed, bool right_dir, int right_speed);

int64_t getEncCounter();


#endif