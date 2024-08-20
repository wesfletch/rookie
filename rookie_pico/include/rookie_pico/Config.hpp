#ifndef CONFIG_HPP
#define CONFIG_HPP

// hardware includes
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"

typedef enum CONFIG_ERROR {
    E_CONFIG_SUCCESS = 0,
    E_CONFIG_NO_ALARMS_AVAILABLE = 1,
} config_error_t;

#define STATUS_LED_PIN PICO_DEFAULT_LED_PIN
#define status_led_mask (1 << STATUS_LED_PIN)
// struct repeating_timer status_led_timer;

int configure_UART(uart_inst_t *UART_ID, uint BAUDRATE, uint TX_PIN, uint RX_PIN, uint DATA_BITS, uint STOP_BITS, uint PARITY, irq_handler_t IRQ_FUN, bool useIRQ);
bool status_led_timer_callback(struct repeating_timer *t);
config_error_t configure_status_LED(struct repeating_timer *timer);


#endif // CONFIG_HPP