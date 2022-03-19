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

// static int counter = 0;
static bool flag = false;

// bool prevA, prevB;
// // uint32_t prevA = GPIO_IRQ_LEVEL_LOW;
// // uint32_t prevB = GPIO_IRQ_LEVEL_LOW;
// uint32_t currA, currB;

// uint16_t prev, curr;

static const int64_t DEBOUNCE_TIME = 5000; 
static absolute_time_t last_interrupt_time = 0;

// enc_count = (int64_t)0;
static volatile int64_t enc_count;

void right_enc_callback(uint gpio, uint32_t events) 
{
    absolute_time_t interrupt_time = get_absolute_time();
    
    int64_t time_diff = absolute_time_diff_us(last_interrupt_time, interrupt_time);
    if (time_diff > DEBOUNCE_TIME)
    {   
        enc_count++;
    }

    last_interrupt_time = interrupt_time;
}

int64_t getEncCounter()
{
    return enc_count;
}

// // enum gpio_irq_level prevA, prevB;
// bool prevA, prevB;
// void right_enc_callback(uint gpio, uint32_t events)
// {
    
//     uint32_t gpio_state = 0;

// 	gpio_state = (gpio_get_all() >> 20) & 0b0011;  	// get all GPIO them mask out all but bits 10, 11, 12
// 													// This will need to change to match which GPIO pins are being used.

	
// 	static bool ccw_fall = 0;  //bool used when falling edge is triggered
// 	static bool cw_fall = 0;
	
// 	uint8_t enc_value = 0;
// 	enc_value = (gpio_state & 0x03);

// 	if (gpio == 20) 
// 	{
// 		if ((!cw_fall) && (enc_value == 0b10)) // cw_fall is set to TRUE when phase A interrupt is triggered
// 			cw_fall = 1; 

// 		if ((ccw_fall) && (enc_value == 0b00)) // if ccw_fall is already set to true from a previous B phase trigger, the ccw event will be triggered 
// 		{
// 			cw_fall = 0;
// 			ccw_fall = 0;
// 			//do something here,  for now it is just printing out CW or CCW
// 			printf("CCW \r\n");
// 		}

// 	}	
// 	if (gpio == 21) 
// 	{
// 		if ((!ccw_fall) && (enc_value == 0b01)) //ccw leading edge is true
// 			ccw_fall = 1;

// 		if ((cw_fall) && (enc_value == 0b00)) //cw trigger
// 		{
// 			cw_fall = 0;
// 			ccw_fall = 0;
// 			//do something here,  for now it is just printing out CW or CCW
// 			printf("CW \r\n");
// 		}

// 	}
// }


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
        return -1;
    }

    // set "wrap": number of cycles for each pulse
    pwm_set_wrap(slice1, 100);

    // start PWMs at 50 = STOP
    pwm_set_chan_level(slice1, PWM_CHAN_A, 50);     // right
    pwm_set_chan_level(slice1, PWM_CHAN_B, 50);     // left

    // set the PWM running
    pwm_set_enabled(slice1, true);

    gpio_set_dir(20, GPIO_IN);
    gpio_set_dir(21, GPIO_IN);

    gpio_pull_down(20);

    // prevA = gpio_get(20);
    // prevB = gpio_get(21);

    return 1;
}

float get_vel_left()
{
    return 0.0;
}

float get_vel_right()
{
    return 0.0;
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
    pwm_set_gpio_level(PWM_1_PIN, left_speed);
    pwm_set_gpio_level(PWM_2_PIN, right_speed);
}


