#ifndef PINS_HPP
#define PINS_HPP

#include <stdint.h>

namespace pins
{

namespace motors
{
static constexpr uint8_t MDD10A_PWM_1_PIN = 16;
static constexpr uint8_t MDD10A_PWM_2_PIN = 17;
static constexpr uint8_t MDD10A_DIR_1_PIN = 18;
static constexpr uint8_t MDD10A_DIR_2_PIN = 19;
} // namespace motors

namespace encoders
{
static constexpr uint8_t LEFT_CHANNEL_A_GPIO = 21;
static constexpr uint8_t LEFT_CHANNEL_B_GPIO = 20;
static constexpr uint8_t RIGHT_CHANNEL_A_GPIO = 15;
static constexpr uint8_t RIGHT_CHANNEL_B_GPIO = 14;
} // namespace encoders

} // namespace pins

#endif // PINS_HPP