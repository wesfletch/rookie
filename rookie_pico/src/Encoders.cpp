// CPP headers
#include <string>
#include <vector>
#include <memory>

// Pico headers
#include <pico/stdlib.h>
#include <pico/binary_info.h>
#include <pico/critical_section.h>
#include "hardware/i2c.h"
#include <machine/endian.h>

#include <rookie_pico/Encoders.hpp>

static constexpr float ENCODER_FREQ = 50; // Hz
static constexpr int32_t ENCODER_PERIOD_MS = (1 / ENCODER_FREQ) * 1000;
static std::map<uint, std::shared_ptr<Encoder>> pinToEncoderMap;


void
init_encoders(
    EncoderList* encoders,
    [[maybe_unused]] struct repeating_timer* timer)
{
    // Configure the callback that all GPIO interrupts (on this core) 
    // will use, unless they are explicitly configured with another.
    // Remember embedded systems 101?:
    // 1) Set callback in vector table -> (gpio_set_irq_callback())
    // 2) enable interrupt in the core -> (irq_set_enabled())
    // 3) enable in IRQ in peripheral  -> (gpio_set_irq_enabled)
    gpio_set_irq_callback(AMT102V_callback);
    irq_set_enabled(IO_IRQ_BANK0, true); 

    // initialize each encoder
    for (auto const& encoder: *encoders) {
        // add the encoder to the global map of encoders so the
        // callback can catch it
        pinToEncoderMap.insert({encoder->channelAPin, encoder});
        pinToEncoderMap.insert({encoder->channelBPin, encoder});
        
        encoder->init();
    }

    // // Set up timer for calculating encoder speeds
    add_repeating_timer_ms(
        ENCODER_PERIOD_MS /** milliseconds */,
        encoder_timer_callback,
        static_cast<void*>(encoders),
        timer);
}

static void
AMT102V_callback(
    uint gpio, 
    [[maybe_unused]] uint32_t event_mask)
{
    // get the Encoder struct of this GPIO
    // (if there is one...)
    auto it = pinToEncoderMap.find(gpio);
    if (it == pinToEncoderMap.end()) { return; }
    std::shared_ptr<Encoder> quad = it->second;

    // the delta for the tick count of Encoder
    int delta = 0;

    // Check if the fired pin is channel A or channel B for the encoder
    bool isChannelA = (gpio == quad->channelAPin);
    bool otherChannelState;

    // Determine the tick delta by determining  which wave is leading:
    //      If A leads B, rotation direction is positive
    //      If B leads A, rotation direction is negative
    // To figure out which is leading, check the state of the NOT fired pin
    if (isChannelA) {
        // CHANNEL A
        otherChannelState = gpio_get(quad->channelBPin);
        if (otherChannelState == false) {
            // If Channel A is going high, and channel B is low, A is leading
            delta = 1;
        } else if (otherChannelState == true) {
            // If channel A is going high, but B is already high, B is leading
            delta = -1;
        } else { return; }
    } else {
        // CHANNEL B
        otherChannelState = gpio_get(quad->channelAPin);
        if (otherChannelState == false) {
            // If channel B is going High, but A is low, B is leading
            delta = -1;
        } else if (otherChannelState == true) {
            // if channel B is going High, but A is already High, A is leading
            delta = 1;
        } else { return; }
    }

    // Modify our counter only within critical sections
    // (This may turn out to be completely unnecessary)
    critical_section_enter_blocking(&quad->criticalSection);
    quad->counter += delta;
    critical_section_exit(&quad->criticalSection);
}

bool
encoder_timer_callback(
    struct repeating_timer *t)
{
    // Doing this as a loop of all encoders instead
    // of a separate timer for each is lazy, but...

    // re-constitute our vector of encoders here
    EncoderList* encoders = 
        static_cast<EncoderList*>
        (t->user_data);

    int ticks = 0;
    for (auto const& encoder : *encoders) {
        critical_section_enter_blocking(&encoder->criticalSection);
        
        if (is_nil_time(encoder->lastUpdateStamp)) {
            encoder->lastUpdateStamp = get_absolute_time();
            encoder->angularVel = 0.0;
            continue; 
        }

        // get the diff of ticks between now and the last time this timer was called
        // printf("TICKS == %d - %d\n", encoder->counter, encoder->prevCounter);
        ticks = encoder->counter - encoder->prevCounter;
        
        encoder->prevCounter = encoder->counter;
        if (encoder->counter == INT32_MAX) {
            encoder->counter = 0;
            encoder->prevCounter = 0 - encoder->prevCounter;
        }

        critical_section_exit(&encoder->criticalSection);

        // get the time diff
        int64_t diff_us = absolute_time_diff_us(
            encoder->lastUpdateStamp,
            get_absolute_time()
        );
        float diffInSecs = diff_us / 1000000.0; 

        // using diff and ticks, determine rotational speed
        encoder->angularVel = ticksToRadians(ticks) / diffInSecs;
        encoder->lastUpdateStamp = get_absolute_time();

// #ifdef DEBUG
        // if (ticks != 0) {
        //     printf("ENCODER: ");
        //     printf(" TICKS=%d, RADIANS=%f,", ticks, ticksToRadians(ticks));
        //     printf(" DIFF=%llu,", diff_us);
        //     printf(" SECS=%f,", diffInSecs);
        //     printf(" VEL=%f", encoder->angularVel);
        //     printf(" RPM=%f", encoder->angularVel * (60 / (2 * M_PI)));
        //     printf("\n");
        // }
// #endif // DEBUG

    }

    return true;
}
