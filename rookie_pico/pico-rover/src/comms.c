#include "../include/comms.h"
#include "../include/main.h"

// #include "pico/mutex.h"

/**  @brief  Steps through the communication protocol using input string and current state COMMS_STATE
 *   @param  state the STATE for this communication instance
 *   @param  in the input string
 *   @param  out destination of response
 */    
void protocol(STATE *state, char *in, char *out)
{
    int status;

    switch(state->state)
    {
        case CLOSED:
            strcpy(out, "SYN");
            state->state++;
            break;
        case SYNSENT:
            status = parseMessage(in);
            if (!status)
            {
                printf("$ERR failed to parse message: %s", in);
                return;
            }
            break;
        case ESTABLISHED:
            break;
        case CLOSEWAIT:
            break;
        case LASTACK:
            break;
    }
}

int parseMessage(char *in)
{
    // for tokenizing input string
    char *delim = "=";
    char *token;

    token = strtok(in, "=");

    if (strcmp(token, "+RCV") == 0)
    {
        // get to third index == DATA
        token = strtok(in, ",");
        token = strtok(in, ",");

        printf("Token: %s\n", token);
        strcpy(in, token);
    }
    else
    {
        return 0;
    }

    return 1;

}

void transmit(char *tx, int buffer_size)
{
    printf("TRANSMIT(): %s\n\n", tx);

    // wait for TX fifo to be empty
    uart_tx_wait_blocking(UART_ID_LORA);

    if (uart_is_writable(UART_ID_LORA))
    {
        uart_puts(UART_ID_LORA, tx);
    }

}

int transmit_and_wait(char *tx, int buffer_size, char *compare)
{
    char rx_buffer[LORA_SIZE];
    char ch;
    int idx;

    transmit(tx, buffer_size);

    if (uart_is_readable_within_us(UART_ID_LORA, 1000000))
    {
        ch = uart_getc(UART_ID_LORA);
        while (ch != ENDSTDIN)
        {
            rx_buffer[idx++] = ch;

            // if the string ends or we run out of space, we're done with this string
            if (ch == CR || ch == NL || idx == (sizeof(rx_buffer)-1))
            {
                rx_buffer[idx] = 0; // terminate the string
                idx = 0;    // reset index
                break;
            }

            ch = uart_getc(UART_ID_LORA);
        }
        printf("TRANSMIT_AND_WAIT() RESPONSE: %s\n", rx_buffer);

        sleep_ms(100);
    }

    return (strcmp(rx_buffer, compare) == 0);

}


/**
 * @brief 
 * 
 * @return int 
 */
void comm_run()
{
    size_t buffer_size;
    char rx_buffer[LORA_SIZE];
    char tx_buffer[LORA_SIZE];
    char ch;
    int idx = 0;
    int status;

    sleep_ms(5000);

    // initialize the communication instance
    STATE state = {CLOSED, 0, 0};

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

    // configure network ID LoRa
    strcpy(tx_buffer, "AT+NETWORKID=5\n");
    status = transmit_and_wait(tx_buffer, strlen(tx_buffer), "+OK\r\n");
    if (!status)
    {
        printf("$ERR asn'asw mgfaw 1\n");
    }

    // configure LoRa address
    strcpy(tx_buffer, "AT+ADDRESS=102\n");
    status = transmit_and_wait(tx_buffer, strlen(tx_buffer), "+OK\r\n");
    if (!status)
    {
        printf("$ERR asn'asw mgfaw 2\n");
    }

    // while (1)
    // {
    //     // sleep_ms(1000);
    //     // tight_loop_contents();
    //     // if we have data to read, read it

    //     if (!uart_is_readable(UART_ID_LORA))
    //     {
    //         continue;
    //     }
    //     ch = uart_getc(UART_ID_LORA);
    //     while (ch != ENDSTDIN)
    //     {
    //         rx_buffer[idx++] = ch;

    //         // if the string ends or we run out of space, we're done with this string
    //         if (ch == CR || ch == NL || idx == (sizeof(rx_buffer)-1))
    //         {
    //             rx_buffer[idx] = 0; // terminate the string
    //             idx = 0;    // reset index
    //             // printf("This is the string I received: %s\n", rx_buffer);
                
    //             // status = protocol(rx_buffer);
    //             // if (!status)
    //             // {
    //             //     printf("Failed to process string: %s\n", rx_buffer);
    //             // }
    //             break;
    //         }

    //         ch = uart_getc(UART_ID_LORA);
    //     }
    //     if (strlen(rx_buffer) >= 1)
    //     {
    //         // printf("$LRA %s\n", rx_buffer);
    //         // protocol(&state, rx_buffer, tx_buffer);

    //         printf("HERE IS THE RX BUFFER, JOSHUA: %s\n", rx_buffer);
    //         // sleep_ms(100);

    //         // transmit(tx_buffer, strlen(tx_buffer));
    //     }

    //     // transmit(TX, strlen(TX));

    // }
}
