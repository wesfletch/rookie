#include "../include/comms.h"
#include "../include/main.h"

#include "pico/mutex.h"

/**  @brief  Steps through the communication protocol using input string and current state COMMS_STATE
 *   @param  state the STATE for this communication instance
 *   @param  in the input string
 *   @param  out destination of response
 */    
void protocol(STATE *state, char *in, char *out)
{
    switch(state->state)
    {
        case CLOSED:
            strcpy(out, "SYN");
            state->state++;
            break;
        case SYNSENT:
            break;
        case ESTABLISHED:
            break;
        case CLOSEWAIT:
            break;
        case LASTACK:
            break;
    }
}

void transmit(char *tx)
{

}

/**
 * @brief 
 * 
 * @return int 
 */
int run()
{
    size_t buffer_size;
    char received[LORA_SIZE];
    char transmit[LORA_SIZE];

    // initialize the communication instance
    STATE state = {CLOSED, 0, 0};

    while (1)
    {
        // if we have data to read, read it

        buffer_size = uart_is_readable(UART_ID_LORA);   // ISSUE (hardware limitations?): https://raspberrypi.github.io/pico-sdk-doxygen/group__hardware__uart.html#ga4752e5d03dd98a08d95705f68784fd15
        if (buffer_size)
        {
            // attempt to read <buffer_size> bytes to buffer
            uart_read_blocking(UART_ID_LORA, received, buffer_size);

            // run through the protocol, store response in transmit
            protocol(&state, received, transmit);

            // transmit the response
            transmit(transmit);
        }
    }
}




















