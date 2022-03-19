#include "main.h"
#include "../include/definitions.h"
#include "../include/comms.h"
#include "../include/motors.h"
#include "../include/config.h"

queue_t data_queue;

// /**
//  * @brief RX interrupt for GPS over UART, blocks until message is terminated
//  * 
//  */
void on_UART_GPS_rx()
{
    char buffer[83];    // max size of NMEA sentence is 82 bytes (according to NMEA-0183) + 1 for termination (\0)
    int idx = 0;
    while (uart_is_readable(UART_ID_GPS)) 
    {
        uint8_t ch = uart_getc(UART_ID_GPS);
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
            ch = uart_getc(UART_ID_GPS);

        }
        // don't send empty buffers
        if (strlen(buffer) > 1)
        {
            // printf("$GPS %s\n", buffer);
        }
    }
}

/**
 * @brief RX interrupt for LORA over UART, blocks until message is terminated
 * 
 */
void on_UART_LORA_rx()
{
    // // larger than the max size of a LoRa transmission
    char buffer[255];
    int idx = 0;
    int status;
    char ch;
    
    // 0 if no bytes available, otherwise the size
    // int size = uart_is_readable(UART_ID_LORA);
    // printf("THIS IS THE SIZE I GOT RIGHT HERE, JOSH: %d\n", size);
    // if (size)
    // {
    //     // make sure to completely read the UART before allowing interrupts
    //     uart_read_blocking(UART_ID_LORA, buffer, LORA_SIZE);
    //     printf("Received this buffer from LORA: %s\n", buffer);
    //     handle_input(buffer);
    // }
    // printf("$LRA %s\n", buffer);

    ch = getchar_timeout_us(0);
    while (ch != ENDSTDIN)
        {
            buffer[idx++] = ch;

            // if the string ends or we run out of space, we're done with this string
            if (ch == CR || ch == NL || idx == (sizeof(buffer)-1))
            {
                buffer[idx] = 0; // terminate the string
                idx = 0;    // reset index
                // printf("This is the string I received: %s\n", buffer);
                
                status = handle_input(buffer);
                if (!status)
                {
                    printf("Failed to process string: %s\n", buffer);
                }
                break;
            }

            ch = getchar_timeout_us(0);
    }
    // printf("$LRA %s\n", buffer);

}

// /**
//  * @brief Configures a UART with the given parameters (convenience function)
//  * 
//  * @param UART_ID   which UART (uart0, uart1) to use
//  * @param BAUDRATE  baudrate
//  * @param TX_PIN    UART TX pin
//  * @param RX_PIN    UART RX pin
//  * @param DATA_BITS databits
//  * @param STOP_BITS stopbits
//  * @param PARITY    parity
//  * @param IRQ_FUN   the IRQ handler (function to call when something is received on UART)
//  * @return status 
//  */
// int configure_UART(uart_inst_t *UART_ID, uint BAUDRATE, uint TX_PIN, uint RX_PIN, uint DATA_BITS, uint STOP_BITS, uint PARITY, irq_handler_t IRQ_FUN, bool useIRQ)
// {
//     int status;


//     // Set up our UART with provided UART_ID and BAUDRATE
//     status = uart_init(UART_ID, BAUDRATE);
//     if (!status) { return 0; }


//     // Set the TX and RX pins by using the function select on the GPIO
//     // See datasheet for more information on function select
//     gpio_set_function(TX_PIN, GPIO_FUNC_UART);
//     gpio_set_function(RX_PIN, GPIO_FUNC_UART);

//     // Set UART flow control CTS/RTS, we don't want these, so turn them off
//     uart_set_hw_flow(UART_ID, false, false);

//     // Set our data format
//     uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);

//     // Turn off FIFO's - we want to do this character by character
//     // uart_set_fifo_enabled(UART_ID, false);

//     if (useIRQ)
//     {
//         // Set up a RX interrupt
//         // We need to set up the handler first
//         // Select correct interrupt for the UART we are using
//         int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;

//         // And set up and enable the interrupt handlers
//         irq_set_exclusive_handler(UART_IRQ, IRQ_FUN);
//         irq_set_enabled(UART_IRQ, true);
        
//         // Now enable the UART to send interrupts - RX only
//         uart_set_irq_enables(UART_ID, true, false);
//     }

//     return 1;
// }

/**
 * @brief   process a given string, dispatch based on contents
 * 
 * @param in the input string
 * @return status of input handling
 */
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
    // MTR messages are used for PWM commands through the Pico
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

/**
 * @brief Program entrypoint.
 * 
 * @return int 
 */
int main() 
{
    stdio_init_all();

    // STDIN/STDOUT IO
    char ch;
    int idx;
    char in_string[255];
    char out_string[255];
    char data[LORA_SIZE];

    int status;

    sleep_ms(2000);

    // configure UART for GPS
    // status = configure_UART(UART_ID_GPS,
    //                         BAUD_RATE_GPS,
    //                         UART_TX_PIN_GPS, UART_RX_PIN_GPS,
    //                         DATA_BITS_GPS, STOP_BITS_GPS, PARITY_GPS,
    //                         on_UART_GPS_rx, 1);
    // if (!status)
    // {
    //     printf("$ERR Failed to initialize UART for GPS.");
    //     return EXIT_FAILURE;
    // }
    
    queue_init(&data_queue, LORA_SIZE, 5);
    multicore_launch_core1(comm_run); // Start core 1 - Do this before any interrupt configuration

    // configure UART for LORA
    status = configure_UART(UART_ID_LORA,
                            BAUD_RATE_LORA,
                            UART_TX_PIN_LORA, UART_RX_PIN_LORA,
                            DATA_BITS_LORA, STOP_BITS_LORA, PARITY_LORA,
                            on_UART_LORA_rx, 0);
    if (!status)
    {
        printf("$ERR Failed to initialize UART for LoRa.");
        // return EXIT_FAILURE;
    }

    status = configure_PWM();

    // configure encoder interrupts
    // gpio_set_irq_enabled_with_callback(20, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &right_enc_callback);
    // gpio_set_irq_enabled(21, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    // configure status LED
    // gpio_init(LED_PIN);
    // gpio_set_dir(LED_PIN, GPIO_OUT);

    set_PWM(true, 65, false, 0);
    
    // spin
    while (1)
    {
        sleep_ms(100);

        // print any data from core 1
        if (queue_try_remove(&data_queue, data))
        {
            printf("\nCORE 0 DATA: %s\n", data);
        } 
            
        // attempt to read char from stdin (serial)
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
