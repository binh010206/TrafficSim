// Fixed-step traffic state, independent of OpenGL and frame rate.
#pragma once
#include "Math.h"
#include <array>
#include <random>
#include <string>
#include <vector>
struct Route {
    std::vector<V> points;
    std::vector<double> arc;
    double stop = 0, entry = 0, leave = 0, total = 0;
    int side = 0, turn = 0;
    V position(double s) const;
    V tangent(double s) const;
};
struct Bike {
    int id = 0, route = 0;
    double s = 0, v = 0, a = 0, wait = 0;
    bool entered = false, reserved = false, yellowCommitted = false;
    double requestSince = -1;
    std::string reason = "CRUISE";
};
enum class SignalColor { Red, Yellow, Green };
class Simulation {
  public:
    std::vector<Route> routes;
    std::vector<Bike> bikes;
    double time = 0, density = 0.60, spawnClock = 0, totalWait = 0;
    int passed = 0, nextId = 1;
    double maxWait = 0;
    Simulation();
    void reset();
    void step(double dt);
    int phase() const;
    double remaining() const;
    bool green(int side) const;
    SignalColor signal(int side) const;
    const char *phaseName() const;
    int activeReservations() const;
    int queueLength(int side) const;
    bool routesConflict(int a, int b) const { return conflicts[a][b]; }

  private:
    std::mt19937 rng{42};
    std::array<std::array<bool, 12>, 12> conflicts{};
    int phaseIndex = 0;
    double phaseElapsed = 0;
    void spawn();
    void updateSignal(double dt);
};
