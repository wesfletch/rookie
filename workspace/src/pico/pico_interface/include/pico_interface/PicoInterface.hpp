#ifndef PICO_INTERFACE_HPP
#define PICO_INTERFACE_HPP


#include <string>


// Message: Motor Control (Command)
// Directly command the motors of the base with
// PWM and direction values.
static const std::string MSG_MOTORS_CMD = "$MTR.C";
typedef struct MOTORS_CMD {

    uint8_t motor_1_direction;
    uint8_t motor_1_pwm;

    uint8_t motor_2_direction;
    uint8_t motor_2_pwm;

} MOTORS_CMD;


static const std::string MSG_MOTORS_STATUS = "$MTR.S";
typedef struct MOTORS_STATUS {

    uint8_t motor_1_direction;
    uint8_t motor_1_pwm;

    uint8_t motor_2_direction;
    uint8_t motor_2_pwm;

} MOTORS_STATUS;



#endif // PICO_INTERFACE_HPP