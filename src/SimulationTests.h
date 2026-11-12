#pragma once
#include "Simulation.h"
#include <iostream>
#include <stdexcept>
#include <unordered_map>

inline void runSimulationTests() {
    auto check = [](bool value, const char *message) {
        if (!value)
            throw std::runtime_error(std::string("Traffic test failed: ") + message);
    };
    auto advance = [](Simulation &s, double seconds) {
        for (int i = 0; i < std::lround(seconds * 60); ++i)
            s.step(1. / 60);
    };
    auto place = [](Simulation &s, int route, double distance, double speed, bool reserved = false,
                    bool entered = false) {
        Bike b;
        b.id = s.nextId++;
        b.route = route;
        b.s = distance;
        b.v = speed;
        b.reserved = reserved;
        b.entered = entered;
        s.bikes.push_back(b);
        return b.id;
    };
    {
        Simulation s;
        s.density = 0;
        for (const auto &r : s.routes) {
            V front = r.position(r.stop) + r.tangent(r.stop) * .892;
            V bar = r.position(r.stop + 1.05);
            check(dot(front - bar, r.tangent(r.stop)) < -.15, "front tire behind stop bar");
        }
        int previous = 0, transitions = 0;
        double changedAt = 0;
        const double expected[] = {18, 3, 2, 18, 3, 2};
        for (int i = 0; i < 60 * 47; ++i) {
            s.step(1. / 60);
            check(!(s.green(0) && s.green(1)), "cross directions green together");
            check(s.signal(0) == s.signal(2) && s.signal(1) == s.signal(3),
                  "opposite signal consistency");
            if (s.phase() != previous) {
                check(s.phase() == (previous + 1) % 6, "signal phase order");
                check(std::abs(s.time - changedAt - expected[previous]) < .02,
                      "unoccupied signal duration");
                changedAt = s.time;
                previous = s.phase();
                ++transitions;
            }
        }
        check(transitions == 6, "full signal cycle");
    }
    {
        Simulation s;
        s.density = 0;
        for (int k = 0; k < 4; ++k)
            place(s, 3, s.routes[3].stop - 9 - 3.4 * k, 4);
        advance(s, 12);
        double previous = 1e9;
        for (const auto &b : s.bikes) {
            check(!b.entered && b.s <= s.routes[b.route].stop + 1e-8 && b.v < .05,
                  "red queue must stop behind line");
            check(previous - b.s >= 2.69, "ordered queue spacing");
            previous = b.s;
        }
    }
    {
        Simulation s;
        s.density = 0;
        advance(s, 17.8);
        place(s, 0, s.routes[0].stop - 5.5, 2, true);
        advance(s, 3.1);
        check(!s.bikes[0].entered && s.bikes[0].v < .05, "yellow with room must stop");
    }
    {
        Simulation s;
        s.density = 0;
        advance(s, 17.95);
        place(s, 0, s.routes[0].stop - .65, 5, true);
        bool enteredOnYellow = false;
        for (int i = 0; i < 60 * 2; ++i) {
            bool wasEntered = s.bikes[0].entered;
            s.step(1. / 60);
            if (!wasEntered && s.bikes[0].entered)
                enteredOnYellow = s.signal(0) == SignalColor::Yellow && s.bikes[0].yellowCommitted;
        }
        check(enteredOnYellow, "committed close vehicle clears on yellow");
    }
    {
        Simulation s;
        s.density = 0;
        advance(s, 21.1);
        place(s, 2, s.routes[2].entry + 1, 0, true, true);
        advance(s, 2.5);
        check(s.phase() == 2 && s.remaining() == 0 && s.activeReservations() > 0,
              "all-red must extend for occupied junction");
        for (int i = 0; i < 60 * 20 && s.phase() == 2; ++i)
            s.step(1. / 60);
        check(s.phase() == 3 && s.activeReservations() == 0, "release after junction clears");
    }
    {
        Simulation s;
        s.density = 0;
        place(s, 0, s.routes[0].stop - .4, 3);
        place(s, 6, s.routes[6].stop - .4, 3);
        place(s, 0, s.routes[0].stop - 4.5, 3);
        s.step(1. / 60);
        check(s.activeReservations() == 3, "disjoint streams and following platoon may proceed");
        advance(s, 3);
        for (const auto &b : s.bikes)
            check(b.entered, "green must discharge queue");
    }
    {
        Simulation s;
        s.density = 0;
        place(s, 2, s.routes[2].stop - .4, 3);
        place(s, 6, s.routes[6].stop - .4, 3);
        s.step(1. / 60);
        check(s.routesConflict(2, 6) && s.activeReservations() == 1,
              "left turn yields to an already reserved crossing stream");
    }
    std::cout << "TRAFFIC SCENARIOS PASS: signal order / red queue / yellow stop + clear / "
                 "extended all-red / concurrent streams / crossing yield\n";

    for (double density : {.25, .7, 1.5}) {
        Simulation s;
        s.density = density;
        double minGap = 1e9;
        int maxConcurrent = 0;
        for (int i = 0; i < 60 * 1800; ++i) {
            std::unordered_map<int, bool> entered;
            for (const auto &b : s.bikes)
                entered[b.id] = b.entered;
            s.step(1. / 60);
            maxConcurrent = std::max(maxConcurrent, s.activeReservations());
            check(!(s.green(0) && s.green(1)), "incompatible green signals");
            for (size_t a = 0; a < s.bikes.size(); ++a) {
                const auto &b = s.bikes[a];
                auto color = s.signal(s.routes[b.route].side);
                check(std::isfinite(b.s) && std::isfinite(b.v) && b.v >= 0, "finite vehicle state");
                check(b.entered || b.s <= s.routes[b.route].stop + .00001, "stop-line overrun");
                if (b.entered && !entered[b.id])
                    check(b.reserved && (color == SignalColor::Green ||
                                         (color == SignalColor::Yellow && b.yellowCommitted)),
                          "unpermitted entry");
                for (size_t j = a + 1; j < s.bikes.size(); ++j) {
                    const auto &q = s.bikes[j];
                    check(!(b.reserved && q.reserved && s.routesConflict(b.route, q.route)),
                          "conflicting crossing reservations");
                    double gap =
                        length(s.routes[b.route].position(b.s) - s.routes[q.route].position(q.s));
                    minGap = std::min(minGap, gap);
                    check(gap >= 2.24, "vehicle overlap");
                }
            }
        }
        check(s.passed > 100 && maxConcurrent >= 2, "traffic flow must make progress");
        std::cout << "PASS density=" << density << " passed=" << s.passed
                  << " active=" << s.bikes.size() << " minGap=" << minGap
                  << " maxWait=" << s.maxWait << " concurrent=" << maxConcurrent << "\n";
    }
}
