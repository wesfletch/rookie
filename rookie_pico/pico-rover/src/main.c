#include "main.h"
#include "../include/definitions.h"
#include "../include/comms.h"
#include "../include/motors.h"
#include "../include/config.h"

// /**
//  * @brief RX interrupt for GPS over UART, blocks until message is terminated
//  * 
//  */
void on_UART_GPS_rx()
{
    // printf("HERE");
    char buffer[83];    // max size of NMEA sentence is 82 bytes (according to NMEA-0183) + 1 for termination (\0)
    int idx = 0;
    while (uart_is_readable(UART_ID_GPS)) 
    {
        // printf("HERE 2");
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
        // printf("HERE 3");
        // don't send empty buffers
        if (strlen(buffer) > 1)
        {
            printf("$GPS %s\n", buffer);
        }
    }

    // printf("HERE 4");
}


static absolute_time_t tach_interrupt_stamp = 0;
static int revolutions = 0;

void tachometer_callback(uint gpio, uint32_t events)
{
    
    absolute_time_t interrupt_time = get_absolute_time();
    int64_t time_diff = absolute_time_diff_us(tach_interrupt_stamp, interrupt_time);

    if (time_diff < 50000)
    {
        return;
    }

    printf("%lld\n", (float)time_diff / 1000000.0);

    tach_interrupt_stamp = get_absolute_time();
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

/**
 * @brief   process a given string, dispatch based on contents
 * 
 * @param in the input string
 * @return status of input handling (EXIT_SUCCESS/EXIT_FAILURE)
 */
int handle_input(char *in)
{
    // for tokenizing input string
    char * delim = " ";
    char * token;

    int seq;

    // for $MTR commands
    bool dir1, dir2;
    int8_t pwm1, pwm2;

    // tokenize string (strtok modifies the original string)
    token = strtok(in, delim);
    // printf("Got this as first token: %s\n", token);

    // "switch" on first token == message type
    // ACK messages are used for confirmation that sent data was received
    if (strcmp(token, MSG_ACK) == 0)
    {
        // check the seq number
        token = strtok(NULL, delim);
        printf("Next token: %s", token);
        seq = atoi(token);
        printf("Got an ACK message. SEQ: %d\n", seq);
        return EXIT_SUCCESS;
    }
    // CMD messages come from the GS, are to be passed up to the SBC with printf
    else if (strcmp(token, MSG_CMD) == 0)
    {        
        token = strtok(NULL, "");
        // to avoid having to copy the string, just re-adding '$CMD' manually
        printf("$CMD %s\n", token);
        return EXIT_SUCCESS;
    }
    // MTR messages are used for PWM commands through the Pico
    else if (strcmp(token, MSG_MOTORS) == 0)
    {
        // unpack message to vars
        token = strtok(NULL, delim);
        dir1 = (strcmp(token, "0") != 0);   // DIR1
        token = strtok(NULL, delim);
        pwm1 = atoi(token);                 // PWM1
        token = strtok(NULL, delim);
        dir2 = (strcmp(token, "0") != 0);   // DIR2
        token = strtok(NULL, delim);
        pwm2 = atoi(token);                 // PWM2

        printf("DIR1: %d\nPWM1: %d\nDIR2: %d\nPWM2: %d\n", dir1, pwm1, dir2, pwm2);
        set_PWM(dir1, pwm1, dir2, pwm2);
        return EXIT_SUCCESS;
    }
    else if (strcmp(token, MSG_REQ) == 0)
    {
        // will depend
        return EXIT_SUCCESS;
    }
    // TX messages are from the SBC, meant to be transmitted on LORA to the GS
    else if (strcmp(token, MSG_TX) == 0)
    {
        // transmit over UART
        return EXIT_SUCCESS;
    }

    // something went wrong
    return EXIT_FAILURE;
}

/**
 * @brief Program entrypoint.
 * 
 * @return int 
 */
int main() 
{
    stdio_init_all();

    // queue_t receive_queue;
    // queue_t transmit_queue;

    // STDIN/STDOUT IO
    char ch;
    int idx;
    char in_string[255];
    char out_string[255];
    char received_data[LORA_SIZE];
    char sent_data[LORA_SIZE] = "data";


    int status;

    sleep_ms(2000);

    // configure UART for GPS
    status = configure_UART(UART_ID_GPS,
                            BAUD_RATE_GPS,
                            UART_TX_PIN_GPS, UART_RX_PIN_GPS,
                            DATA_BITS_GPS, STOP_BITS_GPS, PARITY_GPS,
                            on_UART_GPS_rx, 1);
    if (!status)
    {
        printf("$ERR Failed to initialize UART for GPS.");
        return EXIT_FAILURE;
    }
    
    printf("FUCK");

    queue_init(&receive_queue, LORA_SIZE, 5);
    queue_init(&transmit_queue, LORA_SIZE, 5);
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
        return EXIT_FAILURE;
    }

    // status = configure_PWM();

    // configure encoder interrupts
    // gpio_set_irq_enabled_with_callback(20, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &right_enc_callback);
    // gpio_set_irq_enabled(21, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    // configure status LED
    // gpio_init(LED_PIN);
    // gpio_set_dir(LED_PIN, GPIO_OUT);

    // set_PWM(true, 65, false, 0);
    
    // spin
    while (1)
    {
        // sleep_ms(100);
        // printf("I'M ALIVE...\n");
        if (queue_try_remove(&receive_queue, received_data)) 
        {
            // everything from CORE 1 is a $CMD
            printf("CORE 0 RECEIVED DATA: %s\n", received_data); 
            char cmd[LORA_SIZE] = "$CMD ";
            strcat(cmd, received_data);
            handle_input(cmd);
        }
        if (queue_try_add(&transmit_queue, sent_data)) 
        {
            printf("CORE 0 SENT DATA\n"); 
        }

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
                
                status = handle_input(in_string);
                if (!status)
                {
                    printf("Failed to process string: %s\n", in_string);
                }
                break;
            }

            ch = getchar_timeout_us(0);
        }

        // sleep_ms(10);
        // tight_loop_contents();
    }
}
