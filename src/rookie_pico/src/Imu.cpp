#include <cstdlib>

#include <rookie_pico/Config.hpp>
#include <rookie_pico/Imu.hpp>

namespace rookie::imu
{

static volatile bool imu_data_ready = false;
void
imu_data_ready_handler(uint gpio, uint32_t /*events*/)
{
    if (gpio != pins::imu::INT1_PIN)
    {
        return;
    }
    imu_data_ready = true;
}

void
Imu::_configure_spi()
{

    spi_init(_spi, IMU_BAUDRATE);
    spi_set_format(_spi, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);

    gpio_set_function(pins::imu::SPI_MISO_PIN, GPIO_FUNC_SPI);
    gpio_set_function(pins::imu::SPI_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(pins::imu::SPI_MOSI_PIN, GPIO_FUNC_SPI);

    // CS for this device is active-low, idle high
    gpio_init(pins::imu::SPI_CS_PIN);
    gpio_set_dir(pins::imu::SPI_CS_PIN, GPIO_OUT);
    gpio_put(pins::imu::SPI_CS_PIN, 1);

    // // TODO: not sure about this one
    // gpio_init(pins::imu::INT1_PIN);
    // gpio_set_dir(pins::imu::INT1_PIN, GPIO_IN);
    // gpio_set_irq_enabled_with_callback(
    //     pins::imu::INT1_PIN, GPIO_IRQ_EDGE_RISE, true, &imu_data_ready_handler);
}

void
Imu::read_regs(const Register& reg, uint8_t* dst, size_t n)
{
    // First byte: RW bit + 7-bit addr
    uint8_t a = static_cast<uint8_t>(reg) | READ_BIT;

    // We need to trigger CS by going low
    gpio_put(pins::imu::SPI_CS_PIN, false);

    // Send the read bit and addr
    spi_write_blocking(_spi, &a, 1);
    spi_read_blocking(_spi, 0x00, dst, n);

    // Set CS high to end
    gpio_put(pins::imu::SPI_CS_PIN, true);
}

void
Imu::read_reg(const Register& reg, uint8_t* dst)
{
    read_regs(reg, dst, 1);
}

void
Imu::write_regs(const Register& reg, const uint8_t* src, size_t n)
{
    if (n > MAX_TRANSFER_BYTES)
    {
        n = MAX_TRANSFER_BYTES;
    }

    uint8_t tx[1 + MAX_TRANSFER_BYTES];
    tx[0] = static_cast<uint8_t>(reg) & 0x7F;
    memcpy(&tx[1], src, n);

    // Drive CS low for transmission
    gpio_put(pins::imu::SPI_CS_PIN, 0);

    spi_write_blocking(_spi, tx, 1 + n);

    // CS back high for idle
    gpio_put(pins::imu::SPI_CS_PIN, 1);
}

void
Imu::write_reg(const Register& reg, const uint8_t* src)
{
    write_regs(reg, src, 1);
}

/**
 * Confirm that this is the right device. 
 */
bool
Imu::_who_am_i()
{
    uint8_t who = 0;
    read_reg(rookie::imu::Register::WHO_AM_I, &who);
    return (who == WHO_AM_I_VALUE);
}

void
Imu::_configure_imu()
{
    // Enable IF_INC, so that multi-byte reads on a register will auto-increment
    uint8_t ctrl3 = 0x04;
    // Set block data update (BDU) to 1, so that the output registers
    // only update after we've read both the low and the high, to avoid "tearing"
    ctrl3 |= 1 << 6;
    write_reg(rookie::imu::Register::CTRL3, &ctrl3);

    // // Enable "data ready" interrupts on INT1
    // uint8_t int1_ctrl = 0x02;
    // rookie::imu::write_reg(rookie::imu::Register::INT1_CTRL, &int1_ctrl);

    // Enable the accelerometer
    // Default is high-perf mode, use ODR bit to set frequency
    uint8_t ctrl1 = 0x04;
    write_reg(rookie::imu::Register::CTRL1, &ctrl1);

    // Enable the gyro
    // Default is high-perf mode, use the ODR bit to set the frequency
    uint8_t ctrl2 = 0x04; // e.g. high-perf, ~60 Hz
    write_reg(rookie::imu::Register::CTRL2, &ctrl2);

    // Set the Full-Scale for gyro measurements.
    // WARN: CHANGING THIS VALUE CHANGES THE INTERPRETATION OF THE DATA.
    uint8_t ctrl6 = 0x02; // +/- 500dps
    write_reg(rookie::imu::Register::CTRL6, &ctrl6);

    // Set FS for accel measurements.
    // WARN: CHANGING THIS VALUE CHANGES THE INTERPRETATION OF THE DATA.
    uint8_t ctrl8 = 0x01; // +/-4g
    write_reg(rookie::imu::Register::CTRL8, &ctrl8);
}

config_error_t
Imu::configure()
{
    _configure_spi();

    // Before we bother to setup the IMU, let's make sure it's actually connected.
    if (!_who_am_i())
    {
        // printf("WHO_AM_I check failed.\n");
        return E_CONFIG_FAILURE;
    }

    // Once we've gotten a positive ID, we can go ahead and configure the IMU proper
    _configure_imu();

    // And finally, we need to do our gyro calibration
    if (!this->capture_gyro_bias(200))
    {
        // printf("gyro calibration failed.\n");
        return E_CONFIG_FAILURE;
    }
    _is_ready = true;

    return E_CONFIG_SUCCESS;
}

void
Imu::_read_imu_data_raw(ImuReading& imu)
{
    // Sensor data is contiguous, starting at TEMP, so we can read it all at once
    uint8_t out[14];
    read_regs(rookie::imu::Register::OUT_TEMP_L, out, 14);

    // Temperature (TEMP_L, TEMP_H)
    int16_t temp_raw = static_cast<int16_t>((out[1] << 8) | out[0]);
    imu.temp_c = static_cast<float>(temp_raw) / 256.0f + TEMPERATURE_OFFSET_C;

    // GYROSCOPE
    int16_t gx_raw = static_cast<int16_t>((out[3] << 8) | out[2]); // OUTX_G
    int16_t gy_raw = static_cast<int16_t>((out[5] << 8) | out[4]); // OUTY_G
    int16_t gz_raw = static_cast<int16_t>((out[7] << 8) | out[6]); // OUTZ_G
    imu.angular_vel.x = static_cast<float>(gx_raw) * GYRO_RADPS_PER_LSB;
    imu.angular_vel.y = static_cast<float>(gy_raw) * GYRO_RADPS_PER_LSB;
    imu.angular_vel.z = static_cast<float>(gz_raw) * GYRO_RADPS_PER_LSB;

    // ACCELEROMETER
    int16_t az_raw = static_cast<int16_t>((out[9] << 8) | out[8]);   // OUTZ_A
    int16_t ay_raw = static_cast<int16_t>((out[11] << 8) | out[10]); // OUTY_A
    int16_t ax_raw = static_cast<int16_t>((out[13] << 8) | out[12]); // OUTX_A
    imu.linear_accel.x = static_cast<float>(ax_raw) * ACCEL_MS2_PER_LSB;
    imu.linear_accel.y = static_cast<float>(ay_raw) * ACCEL_MS2_PER_LSB;
    imu.linear_accel.z = static_cast<float>(az_raw) * ACCEL_MS2_PER_LSB;
}

void
Imu::read_imu(ImuReading& out)
{
    _read_imu_data_raw(out);

    // Subtract out the bias from our earlier calibration
    out.angular_vel.x -= _gyro_bias.x;
    out.angular_vel.y -= _gyro_bias.y;
    out.angular_vel.z -= _gyro_bias.z;
}

// TODO: periodically perform zero velocity updates for the gyro
bool
Imu::capture_gyro_bias(const uint samples)
{
    // The movement threshold, this is WAY above the noise for
    // this particular value of FS
    constexpr float STILL_THRESH_DPS = 1.0f * DEG_TO_RAD;

    float minv[3] = { +1e9f, +1e9f, +1e9f };
    float maxv[3] = { -1e9f, -1e9f, -1e9f };
    float sumv[3] = { 0, 0, 0 };

    for (uint i = 0; i < samples; i++)
    {
        ImuReading imu;
        _read_imu_data_raw(imu);

        float g[3] = { imu.angular_vel.x, imu.angular_vel.y, imu.angular_vel.z };
        for (int a = 0; a < 3; a++)
        {
            sumv[a] += g[a];
            if (g[a] < minv[a])
            {
                minv[a] = g[a];
            }
            if (g[a] > maxv[a])
            {
                maxv[a] = g[a];
            }
        }
        sleep_ms(static_cast<uint>(1000.0f / IMU_FREQUENCY));
    }

    // Make sure none of the axes moved during our sampling time
    for (int a = 0; a < 3; a++)
    {
        if (maxv[a] - minv[a] > STILL_THRESH_DPS)
        {
            return false;
        }
    }

    const float f_samples = static_cast<float>(samples);
    _gyro_bias.x = sumv[0] / f_samples;
    _gyro_bias.y = sumv[1] / f_samples;
    _gyro_bias.z = sumv[2] / f_samples;

    return true;
}

} // namespace rookie::imu

int
main()
{
    stdio_init_all();
    // stdio_set_translate_crlf(&stdio_usb, false);

    rookie::imu::Imu imu(spi1);
    if (imu.configure() != E_CONFIG_SUCCESS)
    {
        printf("Configure failed\n");
        return EXIT_FAILURE;
    }
    printf(
        "gyro bias: p=%.3f r=%.3f y=%.3f\n",
        imu.gyro_bias().x,
        imu.gyro_bias().y,
        imu.gyro_bias().z);

    while (true)
    {
        rookie::imu::ImuReading i;
        imu.read_imu(i);

        printf(
            "T=%.1fC | gyro[rad/s] p=%7.2f r=%7.2f y=%7.2f | accel[m/s^2] x=%6.3f y=%6.3f "
            "z=%6.3f\n",
            i.temp_c,
            i.angular_vel.x,
            i.angular_vel.y,
            i.angular_vel.z,
            i.linear_accel.x,
            i.linear_accel.y,
            i.linear_accel.z);
        sleep_ms(20);
    }
}