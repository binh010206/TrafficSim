// Coordinates: meters, Y up. Route rotations operate in the XZ ground plane.
#pragma once
#include <algorithm>
#include <cmath>
constexpr double PI = 3.141592653589793;
struct V {
    double x = 0, y = 0, z = 0;
    V operator+(V b) const { return {x + b.x, y + b.y, z + b.z}; }
    V operator-(V b) const { return {x - b.x, y - b.y, z - b.z}; }
    V operator*(double s) const { return {x * s, y * s, z * s}; }
};
inline double dot(V a, V b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline V cross(V a, V b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline double length(V a) { return std::sqrt(dot(a, a)); }
inline V unit(V a) {
    double l = length(a);
    return l > 1e-10 ? a * (1 / l) : V{0, 1, 0};
}
inline V bezier(V a, V b, V c, V d, double t) {
    auto mix = [t](V p, V q) { return p * (1 - t) + q * t; };
    return mix(mix(mix(a, b), mix(b, c)), mix(mix(b, c), mix(c, d)));
}
