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

// function prototypes
void protocol(STATE *state, char *in, char *out);
void write(char *tx, int buffer_size);
int parseMessage(char *in);
int parseData(STATE *state, char *in, char *flag);
void comm_run();

#endif