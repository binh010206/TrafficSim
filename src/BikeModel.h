#pragma once
#include "Geometry.h"
#include <array>
#include <map>

// Stock, upright motorcycles; meters, +X forward, Y up and +Z rider's right.
enum class BikeType { Wave, Vision };
struct Tire {
    double widthMm, aspect, rimInches;
    double rimRadius() const { return .0127 * rimInches; }
    double radius() const { return rimRadius() + widthMm * aspect / 100000.; }
};
struct BikeSpec {
    const char *name;
    double length, width, height, wheelbase, seatHeight;
    Tire front, rear;
};
const BikeSpec &bikeSpec(BikeType type);
inline BikeType typeForBike(int id) { return id % 2 ? BikeType::Wave : BikeType::Vision; }

enum class Joint { Frame, Fork, FrontWheel, RearWheel };
enum class Finish {
    Paint,
    Graphite,
    Rubber,
    Seat,
    DarkMetal,
    Alloy,
    Bright,
    Lens,
    Reflector,
    Amber,
    Tail,
    Mirror,
    Accent,
    Decal
};
struct BikeModel {
    BikeType type{};
    bool distant = false;
    // Vertices are stored in the local coordinates of each joint.
    std::map<std::pair<Joint, Finish>, Mesh> batches;
    std::vector<V> seatControls;
    void build(BikeType kind, bool distantDetail = false);
    void draw(double distance, double steeringDegrees, bool braking, bool normals,
              bool controls) const;
    void verify() const;
};
