#ifndef TYPES_HPP
#define TYPES_HPP

#include <pico_interface/protos/Vector3.pb.hpp>

namespace rookie
{

struct Vector3
{
    float x;
    float y;
    float z;

    static ::Vector3
    to_proto(const Vector3& in)
    {
        ::Vector3 vec = Vector3_init_zero;
        vec.x = in.x;
        vec.y = in.y;
        vec.z = in.z;

        return vec;
    }
};

} // namespace rookie

#endif // TYPES_HPP