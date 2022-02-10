#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/i2c.h"

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

static int chars_rxed = 0;
const uint LED_PIN = PICO_DEFAULT_LED_PIN;

// RX interrupt handler
void on_uart_rx() {

    gpio_put(LED_PIN, 1);

    char buffer[1024];
    int cnt = 0;
    while (uart_is_readable(UART_ID)) {
        uint8_t ch = uart_getc(UART_ID);        
        printf("%c", ch);
        // buffer[cnt++] = ch;
        // snprintf(buffer, sizeof(buffer), "%uZZZ", ch);

    }
    // printf("\n%s---", *buffer);
    gpio_put(LED_PIN, 0);
}

int main() 
{
    stdio_init_all();

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
    irq_set_exclusive_handler(UART_IRQ, on_uart_rx);
    irq_set_enabled(UART_IRQ, true);

    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(UART_ID, true, false);

    // configure status LED
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // spin
    while (1)
    {
        tight_loop_contents();
    }


}
