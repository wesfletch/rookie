#include "config.h"

#include <stdlib.h>

/**
 * @brief Configures a UART with the given parameters (convenience function)
 * 
 * @param UART_ID   which UART (uart0, uart1) to use
 * @param BAUDRATE  baudrate
 * @param TX_PIN    UART TX pin
 * @param RX_PIN    UART RX pin
 * @param DATA_BITS databits
 * @param STOP_BITS stopbits
 * @param PARITY    parity
 * @param IRQ_FUN   the IRQ handler (function to call when something is received on UART)
 * @return status 
 */
int configure_UART(uart_inst_t *UART_ID, uint BAUDRATE, uint TX_PIN, uint RX_PIN, uint DATA_BITS, uint STOP_BITS, uint PARITY, irq_handler_t IRQ_FUN, bool useIRQ)
{
    int status;

    // Set up our UART with provided UART_ID and BAUDRATE
    status = uart_init(UART_ID, BAUDRATE);
    // if (!status) { return EXIT_FAILURE; }

    // Set the TX and RX pins by using the function select on the GPIO
    // See datasheet for more information on function select
    gpio_set_function(TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(RX_PIN, GPIO_FUNC_UART);

    // Set UART flow control CTS/RTS, we don't want these, so turn them off
    uart_set_hw_flow(UART_ID, false, false);

    // Set our data format
    uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);

    // Turn off FIFO's - we want to do this character by character
    // uart_set_fifo_enabled(UART_ID, false);

    if (useIRQ)
    {
        // Set up a RX interrupt
        // We need to set up the handler first
        // Select correct interrupt for the UART we are using
        int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;

        // And set up and enable the interrupt handlers
        irq_set_exclusive_handler(UART_IRQ, IRQ_FUN);
        irq_set_enabled(UART_IRQ, true);
        
        // Now enable the UART to send interrupts - RX only
        uart_set_irq_enables(UART_ID, true, false);
    }

    return EXIT_SUCCESS;
}