#ifndef ROBOT_CONFIG_HPP
#define ROBOT_CONFIG_HPP

#include <math.h>

namespace robot
{

namespace wheels
{
static constexpr float WHEEL_RADIUS_M = 0.049213f; // 3 7/8"
static constexpr float WHEELBASE_M = 0.15f;
} // namespace wheels

namespace motor
{
// Max RPM of the specific worm-gear motors I'm using at nominal voltage 12V
static constexpr int MOTOR_MAX_RPM = 250;
// Theoretical max velocity, in rads, of the motors at 12V
static constexpr float MOTOR_MAX_VEL_RADS = (MOTOR_MAX_RPM / 60.0f) * (2 * M_PI);
} // namespace motor

} // namespace robot

#endif // ROBOT_CONFIG_HPP