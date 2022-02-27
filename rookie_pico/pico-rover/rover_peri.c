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

// define UART connection
#define UART_ID     uart0
#define BAUD_RATE   9600
#define DATA_BITS   8
#define STOP_BITS   1
#define PARITY      UART_PARITY_NONE

// We are using pins 0 and 1, but see the GPIO function select table in the
// datasheet for information on which other pins can be used.
#define UART_TX_PIN 0
#define UART_RX_PIN 1
#define PWM_1_PIN   16
#define PWM_2_PIN   17

// for string parsing
#define ENDSTDIN	255 // NULL
#define NL          10  // NEWLINE
#define CR		    13  // CARRIAGE RETURN

// message definitions
static const char * MSG_MOTORS = "$MOTORS\0";     // command for motor controller
static const char * MSG_TX     = "$TX\0";         // transmit a string on the LoRa
static const char * MSG_CMD    = "$CMD\0";        // generic command
static const char * MSG_REQ    = "$REQ\0";        // a request for data update, new rate, etc...
static const char * MSG_ACK    = "$ACK\0";        // an acknowledgement that a message was received

// for later
typedef struct MESSAGE {
    uint32_t timestamp;
    char *msg;
} MESSAGE;

// function defs
int handle_input(char *in);

static int chars_rxed = 0;
const uint LED_PIN = PICO_DEFAULT_LED_PIN;

// RX interrupt for GPS over UART
// expected freq.: 
void on_UART_GPS_rx() 
{
    // gpio_put(LED_PIN, 1);

    char buffer[83];    // max size of NMEA sentence is 82 bytes (according to NMEA-0183)
    int idx = 0;
    while (uart_is_readable(UART_ID)) 
    {
        uint8_t ch = uart_getc(UART_ID);
        while (ch != ENDSTDIN)
        {
            buffer[idx++] = ch;

            // if the string ends or we run out of space, we're done with this string
            if (ch == CR || ch == NL || idx == (sizeof(buffer)-1))
            {
                buffer[idx] = 0; // terminate the string
                idx = 0;    // reset index
                // printf("This is the string I received: %s\n", in_string);
                
                break;
            }
            ch = uart_getc(UART_ID);

        }
        printf("%s", buffer);
    }
}

// RX interrupt for LORA over UART
// BLOCKING
// expected freq.: as needed
void on_UART_LORA_rx()
{
    // larger than the max size of a LoRa transmission
    char buffer[255];
    
    // 0 if no bytes available, otherwise the size
    int size = uart_is_readable(UART_ID);
    if (size)
    {
        // make sure to completely read the UART before allowing interrupts
        uart_read_blocking(UART_ID, buffer, size);
        printf("Received this buffer from LORA: %s\n", buffer);
        handle_input(buffer);
    }
}

// initialize a UART to handle our GPS
void configure_UART_GPS()
{
    // Set up our UART with the baudrate defined in NEO-6 datasheet
    uart_init(UART_ID, BAUD_RATE);

    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // Set UART flow control CTS/RTS, we don't want these, so turn them off
    uart_set_hw_flow(UART_ID, false, false);

    // Set our data format
    uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);

    // Turn off FIFO's - we want to do this character by character
    uart_set_fifo_enabled(UART_ID, false);

    // Set up a RX interrupt
    // We need to set up the handler first
    // Select correct interrupt for the UART we are using
    int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;

    // And set up and enable the interrupt handlers
    irq_set_exclusive_handler(UART_IRQ, on_UART_GPS_rx);
    irq_set_enabled(UART_IRQ, true);

    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(UART_ID, true, false);
}

int configure_PWM()
{
    // configure pins for PWM
    gpio_set_function(PWM_1_PIN, GPIO_FUNC_PWM);
    gpio_set_function(PWM_2_PIN, GPIO_FUNC_PWM);

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
    pwm_set_chan_level(slice1, PWM_CHAN_A, 75);     // right
    pwm_set_chan_level(slice1, PWM_CHAN_B, 25);     // left

    // set the PWM running
    pwm_set_enabled(slice1, true);

    return 1;
}

void setPWM()
{
}

// provided a string to be transmitted, sends it over LORA (via UART connection)
// string must end w/ \r\n
// BLOCKING
void LORA_tx(char *tx, int buffer_size)
{
    // wait for TX fifo to be empty
    uart_tx_wait_blocking(UART_ID);

    if (uart_is_writable(UART_ID))
    {
        uart_write_blocking(UART_ID, tx, buffer_size);
    }
}

// process a given string, dispatch based on contents
int handle_input(char *in)
{
    // for tokenizing input string
    char * delim = " ";
    char * token;

    int seq;

    // tokenize string (strtok modifies the original string)
    token = strtok(in, delim);
    printf("Got this as first token: %s\n", token);

    // "switch" on first token == message type
    // ACK messages are used for confirmation that sent data was received
    if (strcmp(token, MSG_ACK) == 0)
    {
        // check the seq number
        token = strtok(NULL, delim);
        printf("Next token: %s", token);
        seq = atoi(token);
        printf("Got an ACK message. SEQ: %d\n", seq);
        return 1;
    }
    // CMD messages come from the GS, are to be passed up to the SBC
    else if (strcmp(token, MSG_CMD) == 0)
    {        
        token = strtok(NULL, delim);
        // to avoid having to copy the string, just re-adding '$CMD' manually
        printf("$CMD %s\n", token);
        return 1;
    }
    // MOTOR messages are used for PWM commands through the Pico
    else if (strcmp(token, MSG_MOTORS) == 0)
    {
        // setPWM()
        return 1;
    }
    else if (strcmp(token, MSG_REQ) == 0)
    {
        // will depend
        return 1;
    }
    // TX messages are from the SBC, meant to be transmitted on LORA to the GS
    else if (strcmp(token, MSG_TX) == 0)
    {
        // transmit over UART
        return 1;
    }

    // something went wrong
    return 0;
}

int main() 
{
    stdio_init_all();

    // STDIN/STDOUT IO
    char ch;
    int idx;
    char in_string[255];
    char out_string[255];

    int status = 0;

    configure_UART_GPS();

    // status = configure_PWM();

    // configure status LED
    // gpio_init(LED_PIN);
    // gpio_set_dir(LED_PIN, GPIO_OUT);

    // spin
    while (1)
    {
        // attempt to read char from stdin
        // no timeout makes it non-blocking
        ch = getchar_timeout_us(0);
        while (ch != ENDSTDIN)
        {
            in_string[idx++] = ch;

            // if the string ends or we run out of space, we're done with this string
            if (ch == CR || ch == NL || idx == (sizeof(in_string)-1))
            {
                in_string[idx] = 0; // terminate the string
                idx = 0;    // reset index
                // printf("This is the string I received: %s\n", in_string);
                
                status = handle_input(in_string);
                if (!status)
                {
                    printf("Failed to process string: %s\n", in_string);
                }
                break;
            }

            ch = getchar_timeout_us(0);
        }


        tight_loop_contents();
    }
}
