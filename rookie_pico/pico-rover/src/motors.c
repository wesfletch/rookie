/**
 * @file motors.c
 * @author Wesley Fletcher (wkfletcher@knights.ucf.com)
 * @brief Functions for controlling the motors via PWM and reading encoders
 * @version 0.1
 * @date 2022-03-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */



// general includes
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// hardware includes
#include "hardware/pwm.h"
#include "pico/stdlib.h"

#include "motors.h"

/**
 * @brief 
 * 
 * @return int 
 */
int configure_PWM()
{
    // configure PWM pins for PWM
    gpio_set_function(PWM_1_PIN, GPIO_FUNC_PWM);
    gpio_set_function(PWM_2_PIN, GPIO_FUNC_PWM);

    // configure DIR pins as GPIO
    gpio_set_function(DIR_1_PIN, GPIO_FUNC_SIO);
    gpio_set_function(DIR_2_PIN, GPIO_FUNC_SIO);
    gpio_set_dir(DIR_1_PIN, GPIO_OUT);
    gpio_set_dir(DIR_2_PIN, GPIO_OUT);

    uint slice1 = pwm_gpio_to_slice_num(PWM_1_PIN);
    uint slice2 = pwm_gpio_to_slice_num(PWM_2_PIN);
    if (slice1 != slice2)
    {
        printf("ERROR: PWM slice mismatch, slice1: %d, slice2: %d", slice1, slice2);
        return EXIT_FAILURE;
    }

    // set "wrap": number of cycles for each pulse
    pwm_set_wrap(slice1, 12500);

    // start PWMs at 0 = STOP
    pwm_set_chan_level(slice1, PWM_CHAN_A, 0);     // right
    pwm_set_chan_level(slice1, PWM_CHAN_B, 0);     // left

    // set the PWM running
    pwm_set_enabled(slice1, true);

    // gpio_set_dir(20, GPIO_IN);
    // gpio_set_dir(21, GPIO_IN);

    // gpio_pull_down(20);

    // prevA = gpio_get(20);
    // prevB = gpio_get(21);

    return EXIT_SUCCESS;
}

/**
 * @brief 
 * 
 * @param left_dir true for forward, false for reverse
 * @param left_speed 0-100
 * @param right_dir true for forward, false for reverse
 * @param right_speed 0-100
 */
void set_PWM(bool left_dir, int left_speed, bool right_dir, int right_speed)
{
    // set DIR pins
    gpio_put(DIR_1_PIN, left_dir);
    gpio_put(DIR_2_PIN, right_dir);

    // set PWM pins
    pwm_set_gpio_level(PWM_1_PIN, (int16_t)left_speed * 125);
    pwm_set_gpio_level(PWM_2_PIN, (int16_t)right_speed * 125);

}


