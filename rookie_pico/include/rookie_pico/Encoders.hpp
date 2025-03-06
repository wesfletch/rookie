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

static const std::string ENCODER_NAME_AMT102V = "AMT-102V";
static const uint LEFT_CHANNEL_A_GPIO = 21;
static const uint LEFT_CHANNEL_B_GPIO = 20;
static const uint RIGHT_CHANNEL_A_GPIO = 15;
static const uint RIGHT_CHANNEL_B_GPIO = 14;

static constexpr uint PULSES_PER_REVOLUTION = 2048;
static constexpr uint COUNTS_PER_PULSE = 4;
static constexpr uint COUNTS_PER_REVOLUTION = COUNTS_PER_PULSE * PULSES_PER_REVOLUTION;

/**
 * @brief Convert encoder pulses to velocity in radians/s
 * 
 * @param pulses Number of pulses
 * @return float equivalent radians for provided pulses.
 */
static inline float
pulsesToRadians(int pulses)
{
    return (pulses * (2 * M_PI)) / PULSES_PER_REVOLUTION; 
}

/**
 * @brief Convert a rads/sec velocity to RPM
 * 
 * @param radians_per_sec 
 * @return float equivalent RPM to radians_per_sec
 */
static inline float
radiansToRPM(float radians_per_sec)
{
    return radians_per_sec * (60 / (2 * M_PI));
}

/**
 * @brief Represents a single quadrature encoder.
 */
class Encoder 
{
public:

    /**
     * @brief Construct a new Encoder object
     * 
     * @param channelA GPIO pin for pulses from encoder channel A
     * @param channelB GPIO pin for pulses from encoder channel B
     * @param invert If true, reverse readings for this encoder; default false
     */
    Encoder(
        uint channelA,
        uint channelB,
        bool invert = false)
        :
        channelAPin(channelA),
        channelBPin(channelB),
        invert(invert) {};

    /**
     * @brief Initialize this Encoder object.
     * 
     * Initialize the assigned ChannelA and ChannelB pins, 
     * the critical section, and the GPIO IRQ that we'll use to 
     * track pulses, plus any other odds and ends.
     * 
     * After this function is called, the Encoder is ready to use.
     */
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

    /**
     * @brief Update reported speed by measuring pulses since last update().
     * 
     */
    void update()
    {   
        // If this is the first time updating, set the vel to 0.0 to
        // avoid spurious readings.
        if (is_nil_time(this->lastUpdateStamp)) {
            this->_setAngularVel(0.0f);
            return; 
        }

        // Get the diff of pulses between now and the last time update was called
        int pulses = this->counter - this->prevCounter;
        this->prevCounter = this->counter;

        // We're capturing both A and B channel pulses, so we'll 
        // have twice as many as we would expect,
        // e.g., we'll read 4096 pulses/rev instead of 2048
        pulses = pulses / 2;

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
        float diff_in_secs = diff_us / 1000000.0; 

        float rads_per_sec = pulsesToRadians(pulses) / diff_in_secs;
        this->_setAngularVel(rads_per_sec);
    };

    /**
     * @brief Get the current reported angular velocity in rads/sec
     * 
     * @return float angular velocity in rads/sec
     */
    float getAngularVel()
    {
        float returned = 0.0;
        critical_section_enter_blocking(&this->criticalSection);
        returned = this->angularVel;
        critical_section_exit(&this->criticalSection);

        return returned;
    };

    /**
     * @brief Update the counter of this encoder.
     * 
     * Intended to be used within the GPIO IRQ that the encoder
     * is hooked to.
     * 
     * @param delta Amount to add to (or subtract from) the counter
     */
    void addToCounter(int delta)
    {
        critical_section_enter_blocking(&this->criticalSection);
        this->counter += delta * (this->invert ? -1 : 1);
        critical_section_exit(&this->criticalSection);
    };

    /**
     * @brief Get the current number of pulses counted by the encoder
     * 
     * @return int32_t number of pulses
     */
    int32_t getCounter()
    {
        int returned;
        critical_section_enter_blocking(&this->criticalSection);
        returned = this->counter;
        critical_section_exit(&this->criticalSection);

        return returned;
    };

    /* GPIO pin of channel A. */
    const uint channelAPin;
    /* GPIO pin of channel B. */
    const uint channelBPin;
    
    /* Whether to invert the speed read by this encoder. */
    const bool invert;

    /* The last time this encoders velocity was updated. */
    absolute_time_t lastUpdateStamp = nil_time;

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
    /* Number of pulses since last read. */
    volatile int32_t counter = 0;
    volatile int32_t prevCounter = 0;

    /*
     * Current rotational velocity of this encoder in rads/sec.
     * CCW rotation is +, CW rotation is -
     */
    float angularVel = 0.0;
};

using EncoderList = std::vector<std::shared_ptr<Encoder>>;

/**
 * @brief Initialize encoders.
 * 
 */
void init_encoders(
    EncoderList* encoders);

/**
 * @brief IRQ for reading pulses from AMT102V quad encoders.
 * 
 * @param gpio The GPIO pin that fired the IRQ.
 * @param event_mask Bitmask of events that caused this IRQ to fire.
 */
static void AMT102V_callback(
    uint gpio, 
    [[maybe_unused]] uint32_t event_mask);


#endif // ENCODERS_HPP