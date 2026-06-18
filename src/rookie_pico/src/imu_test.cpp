
#include <pico/stdio.h>

#include <rookie_pico/Clock.hpp>
#include <rookie_pico/Imu.hpp>

int
main()
{
    stdio_init_all();
    // stdio_set_translate_crlf(&stdio_usb, false);

    rookie::clock::SystemTimeSource time;

    rookie::imu::Imu imu(spi1, time);
    if (imu.configure() != E_CONFIG_SUCCESS)
    {
        printf("Configure failed\n");
        return EXIT_FAILURE;
    }
    printf(
        "gyro bias: x=%.3f y=%.3f z=%.3f\n",
        imu.gyro_bias().x,
        imu.gyro_bias().y,
        imu.gyro_bias().z);

    while (true)
    {
        rookie::imu::ImuReading i;
        imu.read_imu(i);

        printf(
            "T=%.1fC | gyro[rad/s] x=%7.2f y=%7.2f z=%7.2f | accel[m/s^2] x=%6.3f y=%6.3f "
            "z=%6.3f | time: %d\n",
            i.temp_c,
            i.angular_vel.x,
            i.angular_vel.y,
            i.angular_vel.z,
            i.linear_accel.x,
            i.linear_accel.y,
            i.linear_accel.z,
            i.timestamp_us);
        sleep_ms(20);
    }
}