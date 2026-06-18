#ifndef IMU_HPP
#define IMU_HPP

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdio.h>

#include <pico/stdlib.h>
#include <pico/binary_info.h>
#include <hardware/spi.h>

#include <rookie_pico/Config.hpp>
#include <rookie_pico/Pins.hpp>

namespace rookie::imu
{

// This is a guess, the LSM6DSV16 doesn't have a listed baudrate
static constexpr uint IMU_BAUDRATE = 500 * 1000; // 500kHz

// Acceleration sensitivity, which is a function of the FS value selected,
// NOTE: given in mg/LSB, so we're converting to g here
static constexpr float ACCEL_SENSITIVITY = 0.488f / 1000.0f;

// Gyro sensitivity;  a function of the FS value we select
// NOTE: given in mdps/LSB, so we're converting to dps here
static constexpr float GYRO_SENSITIVITY = 17.50f / 1000.0f;

// Unit conversions
static constexpr float DEG_TO_RAD = 3.14159265f / 180.0f;
static constexpr float GRAVITY_MS2 = 9.80665f;

// Converting from sensor output dps/LSB -> rad/s per LSB,  g/LSB -> m/s^2 per LSB
static constexpr float GYRO_RADPS_PER_LSB = GYRO_SENSITIVITY * DEG_TO_RAD;
static constexpr float ACCEL_MS2_PER_LSB = ACCEL_SENSITIVITY * GRAVITY_MS2;

// At 0 LSB, temperature is actually 25 degrees C
static constexpr uint8_t TEMPERATURE_OFFSET_C = 25;

// The RW bit picks the operation on a given address.
// RW is always bit 7 of our "address" byte
static constexpr uint8_t READ_BIT = 1 << 7;  // MSB = 1 -> READ
static constexpr uint8_t WRITE_BIT = 0 << 7; // MSB = 0 -> WRITE

static constexpr uint8_t DATA_SIZE = 8;

static constexpr uint8_t MAX_TRANSFER_BYTES = 32;

// NOTE: this is actually set by register CTRL2, we're just
// keeping track of it here...
static constexpr uint8_t IMU_FREQUENCY = 60;

// Register layout for the LSM6DSVBR
enum class Register : uint8_t
{
    // Control the INT1 interrupt
    INT1_CTRL = 0x0D,
    // The value of WHO_AM_I is fixed to 0x71
    WHO_AM_I = 0x0F,
    // Accelerometer control
    CTRL1 = 0x10,
    // Gyroscope control
    CTRL2 = 0x11,
    // General control register, e.g., software reset, memory clear, auto-increment register
    // for multibyte reads (IF_INC)...
    CTRL3 = 0x12,
    // Gyro LPF bandwidth and FS
    CTRL6 = 0x15,

    // Accel. filter config, dual-channel mode enable, FS
    CTRL8 = 0x17,

    // Unread sensor data amount
    FIFO_STATUS_1 = 0x1B,
    // Status register bitmask; these status events are also
    // what trigger INT1, if that's configured
    FIFO_STATUS_2 = 0x1C,
    // Accel, gyro, temp data available
    STATUS_REG = 0x1E,

    // SENSOR OUTPUTS

    // Temperature output data. 16-bit word, twos complement.
    OUT_TEMP_L = 0x20,
    OUT_TEMP_H = 0x21,
    // Angular X (pitch) output data from gyro. 16-bit word, twos complement.
    OUTX_L_G = 0x22,
    OUTX_H_G = 0x23,
    // Angular Y (roll) output data from gyro. 16-bit word, twos complement.
    OUTY_L_G = 0x24,
    OUTY_H_G = 0x25,
    // Angular Z (yaw) output data from gyro. 16-bit word, twos complement.
    OUTZ_L_G = 0x26,
    OUTZ_H_G = 0x27,
    // Linear accel. Z-axis output from accel. 16-bit word, twos complement.
    OUTZ_L_A = 0x28,
    OUTZ_H_A = 0x29,
    // Linear accel. Y-axis output from accel. 16-bit word, twos complement.
    OUTY_L_A = 0x2A,
    OUTY_H_A = 0x2B,
    // Linear acce. X-axis output from accel. 16-bit word, twos complement.
    OUTX_L_A = 0x2C,
    OUTX_H_A = 0x2D,

    // Timestamp first data output register. 32-bit word, bit resolution is 21.75us.
    TIMESTAMP0 = 0x40,
    TIMESTAMP1 = 0x41,
    TIMESTAMP2 = 0x42,
    TIMESTAMP3 = 0x43,

    // Tells us what sensor generated the FIFO bytes in FIFO_DATA_OUT_BYTE_*
    FIFO_DATA_OUT_TAG = 0x78,
    // FIFO provides 6 bytes of data through these 6 registers;
    // data type of each byte is tagged by corresponding FIFO_DATA_OUT_TAG bit
    FIFO_DATA_OUT_BYTE_0 = 0x79,
    FIFO_DATA_OUT_BYTE_1 = 0x7A,
    FIFO_DATA_OUT_BYTE_2 = 0x7B,
    FIFO_DATA_OUT_BYTE_3 = 0x7C,
    FIFO_DATA_OUT_BYTE_4 = 0x7D,
    FIFO_DATA_OUT_BYTE_5 = 0x7E,
};

// The value of register WHO_AM_I is constant, so we can
// use it to make sure our device is up
static constexpr uint8_t WHO_AM_I_VALUE = 0x71;

struct Vector3
{
    float x;
    float y;
    float z;
};

struct ImuReading
{
    // Gyroscope readings, rad/s
    Vector3 angular_vel;
    // Accelerometer readings, m/s2
    Vector3 linear_accel;
    // Temperature sensor reading in degrees C
    float temp_c;
};

class Imu
{
public:

    Imu(spi_inst_t* spi) : _spi(spi){};

    [[nodiscard]] config_error_t configure();

    void read_imu(ImuReading& out);

    void read_regs(const Register& reg, uint8_t* dst, size_t n);
    void read_reg(const Register& reg, uint8_t* dst);

    void write_regs(const Register& reg, const uint8_t* src, size_t n);
    void write_reg(const Register& reg, const uint8_t* src);

    bool
    is_ready() const
    {
        return _is_ready;
    };

    bool capture_gyro_bias(const uint samples);

    Vector3
    gyro_bias() const
    {
        return this->_gyro_bias;
    }

private:

    void _configure_spi();

    void _configure_imu();

    bool _who_am_i();

    void _read_imu_data_raw(ImuReading& imu);

    // SPI instance the IMU is connected to.
    spi_inst_t* _spi;

    Vector3 _gyro_bias = { 0.0f, 0.0f, 0.0f };

    // Goes true only after setup is complete
    bool _is_ready = false;

}; // class Imu

// TODO: for later, when we move to continuous FIFO reads on INT1
enum class READ_STATE : uint8_t
{
    IDLE = 0,
    START = 1,
    SHIFT = 2,
    END = 3,
};

} // namespace rookie::imu

#endif // IMU_HPP