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
    EncoderList* encoders)
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

    // Determine the tick delta by determining which wave is leading:
    //      If A leads B, rotation direction is positive
    //      If B leads A, rotation direction is negative
    // To figure out which is leading, check the state of the NOT fired pin
    if (isChannelA) {
        // CHANNEL A is high, get Channel B
        otherChannelState = gpio_get(quad->channelBPin);
        if (otherChannelState == false) {
            // If Channel A is going high, and channel B is low, A is leading
            delta = 1;
        } else if (otherChannelState == true) {
            // If channel A is going high, but B is already high, B is leading
            delta = -1;
        } else { return; }
    } else {
        // CHANNEL B is high, get channel A
        otherChannelState = gpio_get(quad->channelAPin);
        if (otherChannelState == false) {
            // If channel B is going High, but A is low, B is leading
            delta = -1;
        } else if (otherChannelState == true) {
            // if channel B is going High, but A is already High, A is leading
            delta = 1;
        } else { return; }
    }

    quad->addToCounter(delta);
}
