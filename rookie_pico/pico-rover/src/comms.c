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
    char flag[4];
    // clear flag
    memset(flag, 0, sizeof(flag));

    switch(state->state)
    {
        case CLOSED:
            strcpy(out, "SYN");
            state->state++;
            break;
        case SYNSENT:
            status = parseMessage(in);
            if(!status) {
                printf("$ERR failed to parse message: %s", in);
                exit(-1);
            }
            status = parseData(state, in, flag);
            if(!status) {
                printf("$ERR failed to parse data: %s", in);
                exit(-1);
            }
            if(strcmp(flag, "SYN") == 0) {
                strcpy(out, "ACK");
                state->state++;
            }
            break;
        case ESTABLISHED:
            status = parseMessage(in);
            if(!status) {
                printf("$ERR failed to parse message: %s", in);
                exit(-1);
            }
            status = parseData(state, in, flag);
            if(!status) {
                printf("$ERR failed to parse data: %s", in);
                exit(-1);
            }
            if(strcmp(flag, "FIN") == 0) {
                strcpy(out, "FIN");
                state->state++;
            } else if (strcmp(flag, "ACK") == 0) {
                // send telemetry
                strcpy(out, "ACK");
            } else if (strcmp(flag, "COM") == 0) {
                // send data to core 0
                if(queue_try_add(&data_queue, in)) printf("CORE 1: DATA SENT\n");
                strcpy(out, "ACK");
            }
            break;
        case LASTACK:
            status = parseMessage(in);
            if(!status) {
                printf("$ERR failed to parse message: %s", in);
                exit(-1);
            }
            status = parseData(state, in, flag);
            if(!status) {
                printf("$ERR failed to parse data: %s", in);
                exit(-1);
            }
            if(strcmp(flag, "ACK") == 0) {
                printf("\nConnection successfully terminated\n");
                sleep_ms(3000);
                *out = '\0';
                state->seq = 0;
                state->ack = 0;
                state->state = CLOSED;
            }
            break;
    }
}

int parseData(STATE *state, char *in, char *flag) {
    char *delim = " ";
    char *token;
    // get GS seq num
    token = strtok(in, delim);
    // get GS ack num
    token = strtok(NULL, delim);
    // get flag
    token = strtok(NULL, delim);
    printf("Flag: %s\n", token);
    // check if flag is valid
    if(*token) {
        strcpy(flag, token);
    } else {
        return 0;
    }
    // if there is data, get it
    token = strtok(NULL, delim);
    // check if data is valid
    if(token) {
        printf("Data: %s\n", token);
        strcpy(in, token);
    } 
    return 1;
}

int parseMessage(char *in)
{
    // for tokenizing input string
    char *delim = ",=";
    char *token;
    
    token = strtok(in, delim);
    // check for RCV
    if (strcmp(token, "+RCV") == 0) {
        // get to third index == DATA
        for(int i = 0; i < 3; i++) token = strtok(NULL, delim);
        printf("Token: %s\n", token);
        // check if data is valid, then update input
        if(*token) strcpy(in, token);
    } else {
        return 0;
    }
    return 1;
}

void write(char *tx, int buffer_size)
{
    printf("TX: %s", tx);

    // wait for TX fifo to be empty
    uart_tx_wait_blocking(UART_ID_LORA);

    if (uart_is_writable(UART_ID_LORA))
    {
        uart_puts(UART_ID_LORA, tx);
    }

}

void read(char *buffer, int timeout) {
    char ch;
    int i = 0;
    // zero the rx buffer
    memset(buffer, 0, sizeof(buffer));
    while(uart_is_readable_within_us(UART_ID_LORA, timeout) && ch != '\n') {
        buffer[i++] = uart_getc(UART_ID_LORA);
    }
    buffer[i] = '\0';
    if(*buffer) printf("%s", buffer);
}

void msgTx(STATE *state, char *out) {
    char data[LORA_SIZE]; 
    char msg[260]; 
    snprintf(data, sizeof(data), "%d %d %s", state->seq, state->ack, out);
    snprintf(msg, sizeof(msg), "AT+SEND=%d,%d,%s\r\n", GS_ADDRESS, strlen(data), data);
    write(msg, strlen(msg));
}

void initLora(char *rx_buffer) {
    int status;
    // flush rx fifo
    read(rx_buffer, 1000000);
    // set network ID
    write("AT+NETWORKID=5\r\n", 16);
    read(rx_buffer, 1000000);
    status = strcmp(rx_buffer, "+OK\r\n");
    // set rover address
    write("AT+ADDRESS=102\r\n", 16);
    read(rx_buffer, 1000000);
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
    absolute_time_t timer;

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
        read(rx_buffer, 1000);
        // discard "+OK" messages
        if(strcmp(rx_buffer, "+OK\r\n") == 0) continue;
        // check for valid data
        if(*rx_buffer || state.state == CLOSED) {
            protocol(&state, rx_buffer, tx_buffer);
            // check if there is something to send
            if(*tx_buffer) {
                // send message
                msgTx(&state, tx_buffer);
                // start timeout timer
                timer = make_timeout_time_ms(5000);
            }
        } else {
            // check for timeout
            if(time_reached(timer)) {
                // retransmit last message
                msgTx(&state, tx_buffer);
                // restart timer
                timer = make_timeout_time_ms(5000);
            }
        }
    }
}
