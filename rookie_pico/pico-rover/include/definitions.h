// general includes
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// hardware includes
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"

// define UART connection for GPS
#define UART_ID_GPS     uart0
#define BAUD_RATE_GPS   9600
#define DATA_BITS_GPS   8
#define STOP_BITS_GPS   1
#define PARITY_GPS      UART_PARITY_NONE

// We are using pins 0 and 1, but see the GPIO function select table in the
// datasheet for information on which other pins can be used.
#define UART_TX_PIN 0
#define UART_RX_PIN 1
#define PWM_1_PIN   16
#define PWM_2_PIN   17

// for string parsing
#define ENDSTDIN    255 // NULL
#define NL          10  // NEWLINE
#define CR          13  // CARRIAGE RETURN

// message definitions
static const char * MSG_MOTORS = "$MOTORS\0";     // command for motor controller
static const char * MSG_TX     = "$TX\0";         // transmit a string on the LoRa
static const char * MSG_CMD    = "$CMD\0";        // generic command, passed to SBC through serial
static const char * MSG_REQ    = "$REQ\0";        // a request for data update, new rate, etc...
static const char * MSG_ACK    = "$ACK\0";        // an acknowledgement that a message was received

// for later
typedef struct MESSAGE {
    uint32_t timestamp;
    char *msg;
} MESSAGE;