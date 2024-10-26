#ifndef ENCODERS_HPP
#define ENCODERS_HPP

// CPP headers
#include <string>
#include <map>
#include <math.h>
#include <vector>
#include <memory>

// Pico headers
#include <pico/stdlib.h>
#include <pico/critical_section.h>
#include <pico/time.h>


static const std::string ENCODER_AMT102V_NAME = "AMT-102V";
static const uint LEFT_CHANNEL_A_GPIO = 20;
static const uint LEFT_CHANNEL_B_GPIO = 21;
static const uint RIGHT_CHANNEL_A_GPIO = 22;
static const uint RIGHT_CHANNEL_B_GPIO = 26;

static constexpr uint TICKS_PER_ROTATION = 2048;
static constexpr uint COUNTS_PER_TICK = 4;
static constexpr uint COUNTS_PER_ROTATION = COUNTS_PER_TICK * TICKS_PER_ROTATION;

/*
 * @brief Convert encoder ticks to radians
 * 
 * @param ticks Number of ticks
 * @return float equivalent radians for provided ticks.
 */
static inline float
ticksToRadians(int ticks)
{
    return ticks * ((2 * M_PI) / TICKS_PER_ROTATION); 
}


/**
 * @brief Represents a single quadrature encodder.
 */
class Encoder 
{
public:

    /* GPIO pin of channel A. */
    uint channelAPin;
    /* GPIO pin of channel B. */
    uint channelBPin;

    // /* Current rotational velocity of this encoder in rads/sec. */
    // float angularVel = 0.0;
    /* The last time this encoders velocity was updated. */
    absolute_time_t lastUpdateStamp = nil_time;

    Encoder(
        uint channelA,
        uint channelB)
        :
        channelAPin(channelA),
        channelBPin(channelB) {};

    void init() 
    {
        gpio_init(this->channelAPin);
        gpio_init(this->channelBPin);

        // Initialize critical section for use later    
        critical_section_init(&this->criticalSection);
    
        // Pre-set our updateStamp to avoid massive velocities
        // being reported at start-up
        this->lastUpdateStamp = get_absolute_time();

        // Enable the GPIO IRQ for our channel A and B pins
        // must be done AFTER the callback is specified with gpio_set_irq_callback() 
        // and the IRQ is enabled with irq_set_enabled()
        
        // CHANNEL A
        gpio_set_irq_enabled(
            this->channelAPin,
            GPIO_IRQ_EDGE_RISE,
            true
        );
        // CHANNEL B
        gpio_set_irq_enabled(
            this->channelBPin,
            GPIO_IRQ_EDGE_RISE,
            true
        );
    };

    void update()
    {        
        if (is_nil_time(this->lastUpdateStamp)) {
            this->_setAngularVel(0.0f);
            return; 
        }

        // get the diff of ticks between now and the last time update was called
        int ticks = this->counter - this->prevCounter;
        this->prevCounter = this->counter;

        // Roll-over if we reach the max counter.
        if (this->counter == INT32_MAX) {
            this->counter = 0;
            this->prevCounter = 0 - this->prevCounter;
        }

        // get the time diff since we last updated
        int64_t diff_us = absolute_time_diff_us(
            this->lastUpdateStamp,
            get_absolute_time()
        );
        float diffInSecs = diff_us / 1000000.0; 

        float rads_per_sec = ticksToRadians(ticks) / diffInSecs;
        this->_setAngularVel(rads_per_sec);
    };

    float getAngularVel()
    {
        float returned = 0.0;
        critical_section_enter_blocking(&this->criticalSection);
        returned = this->angularVel;
        critical_section_exit(&this->criticalSection);

        return returned;
    };

    void addToCounter(int delta)
    {
        critical_section_enter_blocking(&this->criticalSection);
        this->counter += delta;
        critical_section_exit(&this->criticalSection);
    }

protected:

private: // FUNCTIONS

    void _setAngularVel(float new_vel)
    {
        critical_section_enter_blocking(&this->criticalSection);
        this->angularVel = new_vel;
        critical_section_exit(&this->criticalSection);
        
        this->lastUpdateStamp = get_absolute_time();
    };

private: // MEMBERS

    /* Critical section to protect read/write to data fields below. */
    critical_section_t criticalSection;
    /* Number of ticks since last read. */
    volatile int32_t counter = 0;
    volatile int32_t prevCounter = 0;

    /* Current rotational velocity of this encoder in rads/sec. */
    float angularVel = 0.0;
};

using EncoderList = std::vector<std::shared_ptr<Encoder>>;

/**
 * @brief Initialize encoders.
 * 
 */
void init_encoders(
    EncoderList* encoders,
    struct repeating_timer* timer);

/**
 * @brief IRQ for reading ticks from AMT102V quad encoders.
 * 
 * @param gpio The GPIO pin that fired the IRQ.
 * @param event_mask Bitmask of events that caused this IRQ to fire.
 */
static void AMT102V_callback(
    uint gpio, 
    [[maybe_unused]] uint32_t event_mask);

/**
 * @brief Callback for reading encoder values to determine speed.
 * 
 * @param t The timer struct containing information about the encoders.
 * @return true If timer should schedule itself to fire again
 * @return false if timer should stop scheduling itself
 */
bool encoder_timer_callback(
    struct repeating_timer *t);

#endif // ENCODERS_HPP