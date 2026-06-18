#ifndef TRANSFORMS_HPP
#define TRANSFORMS_HPP

#include <cstdint>

#include <rookie_pico/Types.hpp>

namespace rookie::frames
{

enum class Axis : uint8_t
{
    POS_X,
    NEG_X,
    POS_Y,
    NEG_Y,
    POS_Z,
    NEG_Z
};

constexpr int
axis_index(Axis a)
{
    switch (a)
    {
        case Axis::POS_X:
            [[fallthrough]];
        case Axis::NEG_X:
            return 0;
        case Axis::POS_Y:
            [[fallthrough]];
        case Axis::NEG_Y:
            return 1;
        case Axis::POS_Z:
            [[fallthrough]];
        case Axis::NEG_Z:
            return 2;
    }
    return -1; // unreachable: switch covers every enumerator
}

constexpr int
axis_sign(Axis a)
{
    if (a == Axis::POS_X || a == Axis::POS_Y || a == Axis::POS_Z)
    {
        return +1;
    }
    return -1;
}

// The mounting determinant tells us if the mounting {bx,by,bz} is actually physically possible.
constexpr int
mounting_det(Axis bx, Axis by, Axis bz)
{
    const Axis body[3] = { bx, by, bz };
    int M[3][3] = {};
    for (int i = 0; i < 3; ++i)
    {
        M[i][axis_index(body[i])] = axis_sign(body[i]);
    }
    return M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) -
           M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) +
           M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
}

// BX, BY, BZ are BODY axes, that is, they're in the frame of the robot itself:
// REP-103 says: X+ is forward, Y+ is leftward, and Z+ is up
// If a sensor or other component has it's axes oriented some other way, align
// the BODY axis with whatever component axis points in that direction, e.g.,
// if Y+ is forward and X+ is right on the sensor, you would setup the mounting frame like:
//      SensorMounting = MountingFrame<Axis::POS_Y, Axis::NEG_X, Axis::POS_Z>
template <Axis BX, Axis BY, Axis BZ> struct MountingFrame
{
    // If the mounting isn't physically possible, no point in even letting it compile.
    static_assert(
        mounting_det(BX, BY, BZ) == 1,
        "Mounting is not a proper rotation: each body axis must map to a distinct "
        "sensor axis, and the orientation must be right-handed (det == +1).");

    static constexpr Vector3
    to_body(const Vector3& s)
    {
        const float in[3] = { s.x, s.y, s.z };
        return {
            axis_sign(BX) * in[axis_index(BX)],
            axis_sign(BY) * in[axis_index(BY)],
            axis_sign(BZ) * in[axis_index(BZ)],
        };
    }
};

} // namespace rookie::frames

#endif // TRANSFORMS_HPP