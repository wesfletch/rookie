/**
 * @file main.h
 * @author Wesley Fletcher (wkfletcher@knights.ucf.com)
 * @brief Function prototypes for the "main" program to be loaded onto the Pico
 * @version 0.1
 * @date 2022-03-07
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef MAIN_H
#define MAIN_H

// general includes
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// hardware includes
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/util/queue.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"

void on_UART_GPS_rx();
void on_UART_LORA_rx();
// int configure_UART(uart_inst_t *UART_ID, uint BAUDRATE, uint TX_PIN, uint RX_PIN, uint DATA_BITS, uint STOP_BITS, uint PARITY, irq_handler_t IRQ_FUN, bool useIRQ);

int configure_PWM();
void setPWM();

int handle_input(char *in);

extern queue_t data_queue;

#endif



