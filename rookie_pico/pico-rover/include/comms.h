/**
 * @file comms.h
 * @author Wesley Fletcher (wkfletcher@knights.ucf.com)
 * @brief Function prototypes and data structure definitions for LoRa communications
 * @version 0.1
 * @date 2022-03-07
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef COMMS_H
#define COMMS_H

#include "pico/util/queue.h"

typedef enum COMM_STATE {
    CLOSED,
    SYNSENT,
    ESTABLISHED,
    LASTACK
} COMM_STATE;

typedef struct STATE
{
    COMM_STATE state;      // connection status string
    int seq;                // sequence number
    int ack;                // ACK number

} STATE;

// define UART connection for LORA
#define UART_ID_LORA        uart1
#define BAUD_RATE_LORA      115200
#define DATA_BITS_LORA      8
#define STOP_BITS_LORA      1
#define PARITY_LORA         UART_PARITY_NONE
#define UART_TX_PIN_LORA    4
#define UART_RX_PIN_LORA    5

#define LORA_SIZE   240
#define GS_ADDRESS  101

// function prototypes
void protocol(STATE *state, char *in, char *out);
void write(char *tx, int buffer_size);
int parseMessage(char *in);
int parseData(STATE *state, char *in, char *flag);
int initLora(char *rx_buffer);
void comm_run();

queue_t receive_queue;
queue_t transmit_queue;

#endif