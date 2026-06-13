#ifndef PINS_HPP
#define PINS_HPP

#include <stdint.h>

// Maps Pico LOGICAL pins to various hardware interfaces.
// NOT the same as PHYSICAL pins, which are etched on the silkscreen.
namespace pins
{

namespace motors
{
static constexpr uint8_t MDD10A_PWM_1_PIN = 19;
static constexpr uint8_t MDD10A_PWM_2_PIN = 18;
static constexpr uint8_t MDD10A_DIR_1_PIN = 17;
static constexpr uint8_t MDD10A_DIR_2_PIN = 16;
} // namespace motors

namespace encoders
{
static constexpr uint8_t LEFT_CHANNEL_A_GPIO = 21;
static constexpr uint8_t LEFT_CHANNEL_B_GPIO = 20;
static constexpr uint8_t RIGHT_CHANNEL_A_GPIO = 26;
static constexpr uint8_t RIGHT_CHANNEL_B_GPIO = 22;
} // namespace encoders

namespace imu
{
static constexpr uint8_t SPI_MISO_PIN = 8; // SDO
static constexpr uint8_t SPI_CS_PIN = 9;
static constexpr uint8_t SPI_SCK_PIN = 10;
static constexpr uint8_t SPI_MOSI_PIN = 11; // SDI
static constexpr uint8_t INT1_PIN = 12;     // data-ready
} // namespace imu

} // namespace pins

#endif // PINS_HPP