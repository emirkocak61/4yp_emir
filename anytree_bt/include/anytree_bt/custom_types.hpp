#pragma once

#include <behaviortree_cpp/behavior_tree.h>

//Custom types
struct Position3D {
    double x, y, z;
};

struct Quaternion {
    double x, y, z, w;
};

struct Orientation3D {
    double R,P,Y;
};

namespace BT {
template <> inline Position3D convertFromString(StringView str){
    //real number separated by space
    auto parts = splitString(str, ' ');
    if (parts.size() != 3) {
        throw RuntimeError("invalid input");
    } else {
        Position3D output;
        output.x = convertFromString<double>(parts[0]);
        output.y = convertFromString<double>(parts[1]);
        output.z = convertFromString<double>(parts[2]);
        return output;
    }
}

template <> inline Quaternion convertFromString(StringView str) {
    //Real numbers separated by space
    auto parts = splitString(str, ' ');
    if (parts.size() != 4) {
        throw RuntimeError("invalid input");
    } else {
        Quaternion output;
        output.x = convertFromString<double>(parts[0]);
        output.y = convertFromString<double>(parts[1]);
        output.z = convertFromString<double>(parts[2]);
        output.w = convertFromString<double>(parts[3]);
        return output;
    }

}

template <> inline Orientation3D convertFromString(StringView str) {
    //Real numbers separated by space
    auto parts = splitString(str, ' ');
    if (parts.size() != 3) {
        throw RuntimeError("invalid input (expected 3)");
    } else {
        Orientation3D output;
        output.R = convertFromString<double>(parts[0]);
        output.P = convertFromString<double>(parts[1]);
        output.Y = convertFromString<double>(parts[2]);
        return output;
    }
}

} //end namespace BT
