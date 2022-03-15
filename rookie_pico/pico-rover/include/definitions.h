#ifndef DEFINITIONS_H
#define DEFINITIONS_H

// define UART connection for GPS
#define UART_ID_GPS         uart0
#define BAUD_RATE_GPS       9600
#define DATA_BITS_GPS       8
#define STOP_BITS_GPS       1
#define PARITY_GPS          UART_PARITY_NONE
#define UART_TX_PIN_GPS     0
#define UART_RX_PIN_GPS     1

// for string parsing
#define ENDSTDIN    255 // NULL
#define NL          10  // NEWLINE
#define CR          13  // CARRIAGE RETURN

// message definitions
static const char * MSG_MOTORS = "$MTR\0";      // command for motor controller
static const char * MSG_TX     = "$TXR\0";      // transmit a string on the LoRa
static const char * MSG_CMD    = "$CMD\0";      // generic command, passed to SBC through serial
static const char * MSG_REQ    = "$REQ\0";      // a request for data update, new rate, etc...
static const char * MSG_ACK    = "$ACK\0";      // an acknowledgement that a message was received
// message buffer sizes
#define NMEA_SIZE   83


#endif