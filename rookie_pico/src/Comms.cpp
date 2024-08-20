// CPP headers
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <cstring>

// Pico headers
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"

#include <rookie_pico/Comms.hpp>

// Data queues
queue_t data_queue;
queue_t receive_queue;
queue_t transmit_queue;


/**  @brief  Steps through the communication protocol using input string and current state COMMS_STATE
 *   @param  state the STATE for this communication instance
 *   @param  in the input string
 *   @param  out destination of response
 */    
void protocol(
    [[maybe_unused]] STATE *state, 
    [[maybe_unused]] char *in, 
    [[maybe_unused]] char *out)
{
    // int status;
    // char flag[5];
    // char data[LORA_SIZE];

    // // clear flag
    // memset(flag, 0, sizeof(flag));

    // switch(state->state)
    // {
    //     case CLOSED:
    //         strcpy(out, "SYN");
    //         state->state++;
    //         break;
    //     case SYNSENT:
    //         // Parse message from ground station
    //         status = parseMessage(in);
    //         if(status) {
    //             printf("$ERR failed to parse message: %s", in);
    //             exit(-1);
    //         }
    //         // Parse message payload
    //         status = parseData(state, in, flag);
    //         if(status) {
    //             printf("$ERR failed to parse data: %s", in);
    //             exit(-1);
    //         }
    //         if(strcmp(flag, "SYN") == 0) {
    //             strcpy(out, "ACK");
    //             state->state++;
    //         }
    //         break;
    //     case ESTABLISHED:
    //         status = parseMessage(in);
    //         if(status) {
    //             printf("$ERR failed to parse message: %s", in);
    //             exit(-1);
    //         }
    //         status = parseData(state, in, flag);
    //         if(status) {
    //             printf("$ERR failed to parse data: %s", in);
    //             exit(-1);
    //         }
    //         if(strcmp(flag, "FIN") == 0) {
    //             strcpy(out, "FIN");
    //             state->state++;
    //         } else if (strcmp(flag, "ACK") == 0) {
    //             //if(queue_try_add(&receive_queue, in)) printf("CORE 1: SENT DATA\n");
    //             strcpy(out, "ACK");
    //             if(queue_try_remove(&transmit_queue, data)) printf("CORE 1: RECEIVED DATA: %s\n", data);
    //             strcat(strcat(out, " "), data);
    //         } else if (strcmp(flag, "$CMD") == 0) {
    //             if(queue_try_add(&receive_queue, in)) printf("CORE 1: SENT DATA\n");
    //             strcpy(out, "ACK");
    //             if(queue_try_remove(&transmit_queue, data)) printf("CORE 1: RECEIVED DATA: %s\n", data);
    //             strcat(strcat(out, " "), data);
    //         }
    //         break;
    //     case LASTACK:
    //         status = parseMessage(in);
    //         if(status) {
    //             printf("$ERR failed to parse message: %s\n", in);
    //             // exit(-1);
    //         }
    //         status = parseData(state, in, flag);
    //         if(status) {
    //             printf("$ERR failed to parse data: %s\n", in);
    //             // exit(-1);
    //         }
    //         if(strcmp(flag, "ACK") == 0) {
    //             printf("\nConnection terminated successfully\n");
    //             sleep_ms(3000);
    //             *out = '\0';
    //             state->seq = 0;
    //             state->ack = 0;
    //             state->state = CLOSED;
    //         }
    //         break;
    // }
}

/**
 * @brief Parses data within a message 
 * @param state the STATE for this communication instance
 * @param in data for protocol()
 * @param flag communication flag
 * @return int status; 0 = failure; 1 = success
 */
int parseData(
    [[maybe_unused]] STATE *state, 
    [[maybe_unused]] char *in, 
    [[maybe_unused]] char *flag) 
{
    // std::string delim = " ";
    // char *token;
    // // char data[LORA_SIZE];
    
    // // get GS seq num
    // token = strtok(in, delim.c_str());
    // // set rover ack num
    // state->ack = atoi(token) + 1;
    // // get GS ack num
    // token = strtok(NULL, delim.c_str());
    // // set rover seq num
    // state->seq = atoi(token);
    
    // // get flag
    // token = strtok(NULL, delim.c_str());
    // // printf("Flag: %s\n", token);
    // // check if flag is valid
    // if (*token) 
    // {
    //     strcpy(flag, token);
    // } 
    // else 
    // {
    //     return EXIT_FAILURE;
    // }

    // // if there is data, get it
    // token = strtok(NULL, "\r");
    // // check if data is valid
    // if (token) 
    // {
    //     printf("Data: %s\n", token);
    //     strcpy(in, token);
    // }

    return EXIT_SUCCESS;
}

/**
 * @brief Parses message from ground station; 
 * @param in message for protocol()
 * @return int status; 0 = failure; 1 = success
 */
int parseMessage(
    [[maybe_unused]] char *in)
{
    // // for tokenizing input string
    // char *delim = ",=";
    // char *token;
    
    // token = strtok(in, delim);
    // // check for RCV
    // if (strcmp(token, "+RCV") == 0) 
    // {
    //     // get to third index == DATA
    //     for (int i = 0; i < 3; i++) 
    //         token = strtok(NULL, delim);
    //     // printf("Token: %s\n", token);
    //     // check if data is valid, then update input
    //     if (*token) 
    //         strcpy(in, token);
    // } 
    // else 
    // {
    //     return EXIT_FAILURE;
    // }
    
    return EXIT_SUCCESS;
}

/**
 * @brief Sends rover telemetry to ground station by writing to LoRa's UART pins
 * @param tx message to be sent
 */
void write(
    [[maybe_unused]] std::string tx)
{
    // printf("TX: %s", tx);

    // // wait for TX fifo to be empty
    // uart_tx_wait_blocking(UART_ID_LORA);

    // if (uart_is_writable(UART_ID_LORA))
    // {
    //     uart_puts(UART_ID_LORA, tx.c_str());
    // }

}

/**
 * @brief Read from UART with timeout
 * 
 * @param buffer buffer where UART data is placed
 * @param timeout timeout (in us) before giving up on reading from UART
 */
void read(
    [[maybe_unused]] char *buffer, 
    [[maybe_unused]] int timeout) 
{
    // char ch;
    // int i = 0;
    
    // // zero the rx buffer
    // memset(buffer, 0, sizeof(buffer));

    // while (uart_is_readable_within_us(UART_ID_LORA, timeout) && ch != '\n') 
    // {
    //     buffer[i++] = uart_getc(UART_ID_LORA);
    // }
    // buffer[i] = '\0';
    
    // if (*buffer) {
    //     printf("%s", buffer);
    // }
}

/**
 * @brief Constructs a sendable message, then sends it to the ground station
 * @param state the STATE for this communication instance
 * @param out data to be sent
 */
void msgTx(
    [[maybe_unused]] STATE *state, 
    [[maybe_unused]] char *out) 
{
    // char data[LORA_SIZE]; 
    // char msg[260]; 
    // snprintf(data, sizeof(data), "%d %d %s", state->seq, state->ack, out);
    // snprintf(msg, sizeof(msg), "AT+SEND=%d,%d,%s\r\n", GS_ADDRESS, strlen(data), data);
    // write(msg);
}

/**
 * @brief Configures LoRa parameters
 * @param rx_buffer holds the incoming message
 */
int 
initLora(
    [[maybe_unused]] char *rx_buffer) 
{
    
    // int status = 0;
    
    // // flush rx fifo
    // read(rx_buffer, 1000000);
    
    // // set network ID
    // write("AT+NETWORKID=5\r\n");
    // read(rx_buffer, 1000000);
    // status = strcmp(rx_buffer, "+OK\r\n");
    // if (status)
    // {
    //     printf("$ERR failed to configure LoRa NETWORK ID\n");
    //     return EXIT_FAILURE;
    // }
    
    // // set rover address
    // write("AT+ADDRESS=102\r\n");
    // read(rx_buffer, 1000000);
    // status = strcmp(rx_buffer, "+OK\r\n");
    // if (status)
    // {
    //     printf("$ERR failed to configure LoRa ADDRESS\n");
    //     return EXIT_FAILURE;
    // }

    return EXIT_SUCCESS;
}

/**
 * @brief Handles communication with the ground station; runs on core 1 on Pi Pico
 */
void 
comm_run()
{
    // char rx_buffer[LORA_SIZE];
    // char tx_buffer[LORA_SIZE];
    // int status;
    // int restart_connection = 0; 
    // absolute_time_t timer;

    // // initialize the communication instance
    // STATE state = {CLOSED, 0, 0};

    // // // configure UART for LORA
    // // status = configure_UART(UART_ID_LORA,
    // //                         BAUD_RATE_LORA,
    // //                         UART_TX_PIN_LORA, UART_RX_PIN_LORA,
    // //                         DATA_BITS_LORA, STOP_BITS_LORA, PARITY_LORA,
    // //                         on_UART_LORA_rx, 0);
    // // if (!status)
    // // {
    // //     printf("$ERR Failed to initialize UART for LoRa.");
    // //     // return EXIT_FAILURE;
    // // }
    
    // // configure LoRa; if we fail, just kill this whole thread
    // status = initLora(rx_buffer);
    // if (status)
    // {
    //     printf("$ERR Failed to initialize LoRa. Killing LoRa core.\n");
    //     return;
    // }
    
    // while (1)
    // { 
    //     // poll rx fifo
    //     read(rx_buffer, 1000);
    //     // discard "+OK" messages
    //     if(strcmp(rx_buffer, "+OK\r\n") == 0) continue;
    //     // check for valid data
    //     if(*rx_buffer || state.state == CLOSED) {
    //         protocol(&state, rx_buffer, tx_buffer);
    //         restart_connection = 0;
    //         // check if there is something to send
    //         if(*tx_buffer) {
    //             // send message
    //             msgTx(&state, tx_buffer);
    //             // start timeout timer
    //             timer = make_timeout_time_ms(5000);
    //         }
    //     } else {
    //         // check for timeout
    //         if(time_reached(timer)) {
    //             restart_connection++;
    //             if(restart_connection >= 3) {
    //                 state.seq = 0;
    //                 state.ack = 0;
    //                 state.state = CLOSED;
    //                 printf("\nConnection terminated unsuccessfully\n");
    //             } else {
    //                 // retransmit last message
    //                 msgTx(&state, tx_buffer);
    //                 // restart timer
    //                 timer = make_timeout_time_ms(5000);
    //             }
    //         }
    //     }
    // }
}
