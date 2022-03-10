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

void write(char *tx, int buffer_size)
{
    printf("TRANSMIT(): %s", tx);

    // wait for TX fifo to be empty
    uart_tx_wait_blocking(UART_ID_LORA);

    if (uart_is_writable(UART_ID_LORA))
    {
        uart_puts(UART_ID_LORA, tx);
    }

}

void read(char *buffer) {
    char ch;
    int i = 0;
    // zero the rx buffer
    memset(buffer, 0, sizeof(buffer));
    while(uart_is_readable_within_us(UART_ID_LORA, 1000000) && ch != '\n') {
        buffer[i++] = uart_getc(UART_ID_LORA);
    }
    buffer[i] = '\0';
    printf("%s", buffer);
}

void initLora(char* rx_buffer) {
    int status;
    // clear rx fifo
    read(rx_buffer);
    // set network ID
    write("AT+NETWORKID=5\r\n", 16);
    read(rx_buffer);
    status = strcmp(rx_buffer, "+OK\r\n");
    // set rover address
    write("AT+ADDRESS=102\r\n", 16);
    read(rx_buffer);
    status = strcmp(rx_buffer, "+OK\r\n");
    // error check
    if (status) printf("Error\n");
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
    // configure LoRa
    initLora(rx_buffer);

    while (1)
    {
        // poll rx fifo
        read(rx_buffer);
        if(*rx_buffer) {
            //protocol(&state, rx_buffer, tx_buffer);
        } else printf("0");
    }
}
