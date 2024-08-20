#ifndef DEFINITIONS_HPP
#define DEFINITIONS_HPP

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
static const std::string MSG_MOTORS = "$MTR";      // command for motor controller
static const std::string MSG_TX     = "$TXR";      // transmit a string on the LoRa
static const std::string MSG_CMD    = "$CMD";      // generic command, passed to SBC through serial
static const std::string MSG_REQ    = "$REQ";      // a request for data update, new rate, etc...
static const std::string MSG_ACK    = "$ACK";      // an acknowledgement that a message was received
// message buffer sizes
#define NMEA_SIZE   83


#endif // DEFINITIONS_HPP