// general includes
#include <stdio.h>
#include <stdlib.h>

// Pico includes
#include "pico/stdlib.h"
#include <pico/multicore.h>

#include <rookie_pico/Encoders.hpp>


int main()
{
    stdio_init_all();

    // Configure our encoders...
    std::shared_ptr<Encoder> leftEncoder = std::make_shared<Encoder>(
        Encoder(LEFT_CHANNEL_A_GPIO, LEFT_CHANNEL_B_GPIO, false /* invert*/));
    std::shared_ptr<Encoder> rightEncoder = std::make_shared<Encoder>(
        Encoder(RIGHT_CHANNEL_A_GPIO, RIGHT_CHANNEL_B_GPIO, false /* invert*/));

    EncoderList encoders = {leftEncoder, rightEncoder};
    init_encoders(&encoders);

    while (1)
    {
        leftEncoder->update();
        rightEncoder->update();

        printf("LEFT: %.3f\n", radiansToRPM(leftEncoder->getAngularVel()));
        printf("RIGHT: %.3f\n", radiansToRPM(rightEncoder->getAngularVel()));

        sleep_ms(20);
    }


    return EXIT_SUCCESS;


}