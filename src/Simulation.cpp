// Routes combine straight segments and cubic Bezier turns. Arc-length tables map meters to
// positions.
#include "Simulation.h"
#include "Matrix.h"
#include <limits>
#include <stdexcept>
V Route::position(double s) const {
    if (s <= 0)
        return points.front();
    if (s >= total)
        return points.back();
    auto it = std::upper_bound(arc.begin(), arc.end(), s);
    size_t i = it - arc.begin();
    double t = (s - arc[i - 1]) / (arc[i] - arc[i - 1]);
    return points[i - 1] * (1 - t) + points[i] * t;
}
V Route::tangent(double s) const {
    return unit(position(std::min(total, s + .04)) - position(std::max(0., s - .04)));
}
Simulation::Simulation() {
    for (int side = 0; side < 4; ++side)
        for (int turn = 0; turn < 3; ++turn) {
            Route r;
            r.side = side;
            r.turn = turn;
            const Matrix4x4 routeRotation = rotationYMatrix(-side * 90.);
            V p0{2, 0, 9}, p3; // right-hand traffic; approach from south
            if (turn == 0)
                p3 = {2, 0, -9};
            else if (turn == 1)
                p3 = {9, 0, 2};
            else
                p3 = {-9, 0, -2};
            V endDir = turn == 0 ? V{0, 0, -1} : (turn == 1 ? V{1, 0, 0} : V{-1, 0, 0});
            V p1 = p0 + V{0, 0, -1} * (turn == 2 ? 11. : 5.),
              p2 = p3 - endDir * (turn == 2 ? 11. : 5.);
            auto add = [&](V p) {
                p = transformPoint(routeRotation, p);
                if (!r.points.empty())
                    r.total += length(p - r.points.back());
                r.points.push_back(p);
                r.arc.push_back(r.total);
            };
            for (int i = 0; i <= 100; i++) {
                add({2, 0, 44 - 35. * i / 100});
            }
            r.entry = r.total;
            // Stop bar is 13 m from the junction centre, before the crosswalk.
            // The bike's centre stops 1.05 m before it: its front tire stays behind the bar.
            r.stop = 44. - 13. - 1.05;
            for (int i = 1; i <= 200; i++) {
                add(bezier(p0, p1, p2, p3, i / 200.));
            }
            r.leave = r.total;
            for (int i = 1; i <= 100; i++) {
                add(p3 + endDir * (35. * i / 100));
            }
            routes.push_back(r);
        }
    // Precompute which entire crossing paths overlap. Disjoint streams may use the
    // intersection together. A platoon on the same approach is controlled by headway.
    for (int a = 0; a < 12; ++a)
        for (int b = a + 1; b < 12; ++b) {
            if (routes[a].side == routes[b].side)
                continue;
            bool conflict = false;
            for (double x = routes[a].stop; x < routes[a].leave + 3 && !conflict; x += .35)
                for (double y = routes[b].stop; y < routes[b].leave + 3; y += .35)
                    if (length(routes[a].position(x) - routes[b].position(y)) < 3.1) {
                        conflict = true;
                        break;
                    }
            conflicts[a][b] = conflicts[b][a] = conflict;
        }
    reset();
}
void Simulation::reset() {
    bikes.clear();
    time = spawnClock = totalWait = maxWait = 0;
    passed = 0;
    nextId = 1;
    phaseIndex = 0;
    phaseElapsed = 0;
    rng.seed(42);
}
int Simulation::phase() const { return phaseIndex; }
double Simulation::remaining() const {
    const double duration[] = {18, 3, 2, 18, 3, 2};
    return std::max(0., duration[phaseIndex] - phaseElapsed);
}
bool Simulation::green(int side) const { return phase() == ((side % 2) ? 3 : 0); }
SignalColor Simulation::signal(int side) const {
    if (green(side))
        return SignalColor::Green;
    if (phaseIndex == ((side % 2) ? 4 : 1))
        return SignalColor::Yellow;
    return SignalColor::Red;
}
const char *Simulation::phaseName() const {
    const char *names[] = {"BAC-NAM XANH",  "BAC-NAM VANG",  "DO TOAN BO / DON NUT",
                           "DONG-TAY XANH", "DONG-TAY VANG", "DO TOAN BO / DON NUT"};
    return names[phaseIndex];
}
int Simulation::activeReservations() const {
    return std::count_if(bikes.begin(), bikes.end(), [](const Bike &b) { return b.reserved; });
}
int Simulation::queueLength(int side) const {
    return std::count_if(bikes.begin(), bikes.end(), [&](const Bike &b) {
        return routes[b.route].side == side && !b.entered && b.v < .15;
    });
}
void Simulation::updateSignal(double dt) {
    phaseElapsed += dt;
    if (remaining() > 1e-9)
        return;
    bool clearance = phaseIndex == 2 || phaseIndex == 5;
    // Never start a new green across vehicles still finishing the previous phase.
    if (clearance && activeReservations() > 0)
        return;
    phaseIndex = (phaseIndex + 1) % 6;
    phaseElapsed = 0;
}
void Simulation::spawn() {
    int side = rng() % 4;
    double u = double(rng() % 1000) / 1000;
    int turn = u < .55 ? 0 : (u < .8 ? 1 : 2), ri = side * 3 + turn;
    V p = routes[ri].position(0);
    for (auto &b : bikes)
        if (length(routes[b.route].position(b.s) - p) < 4.5)
            return;
    Bike b;
    b.id = nextId++;
    b.route = ri;
    bikes.push_back(b);
}
void Simulation::step(double dt) {
    if (!std::isfinite(dt) || dt <= 0 || dt > .1)
        throw std::invalid_argument("Simulation requires a positive timestep <= 0.1 s");
    time += dt;
    for (auto &b : bikes)
        if (b.s > routes[b.route].leave + 3)
            b.reserved = false;
    updateSignal(dt);
    spawnClock += dt;
    double interval = density > 0 ? 1 / density : std::numeric_limits<double>::infinity();
    while (spawnClock >= interval) {
        spawnClock -= interval;
        spawn();
    }
    for (auto &b : bikes) {
        auto &r = routes[b.route];
        if (b.entered)
            continue;
        if (b.requestSince < 0 && b.s >= r.stop - 6)
            b.requestSince = time;
        if (signal(r.side) == SignalColor::Yellow && b.reserved && !b.yellowCommitted) {
            // Yellow dilemma zone: react for 0.45 s, then brake at 2.5 m/s^2.
            double comfortableStop = .45 * b.v + b.v * b.v / (2 * 2.5);
            b.yellowCommitted = b.v > .5 && r.stop - b.s <= comfortableStop;
            if (!b.yellowCommitted)
                b.reserved = false;
        }
        if (signal(r.side) == SignalColor::Red) {
            b.reserved = false;
            b.yellowCommitted = false;
        }
    }
    std::vector<Bike *> requests;
    for (auto &b : bikes)
        if (!b.entered && !b.reserved && b.requestSince >= 0 && green(routes[b.route].side))
            requests.push_back(&b);
    std::sort(requests.begin(), requests.end(), [](const Bike *a, const Bike *b) {
        return a->requestSince == b->requestSince ? a->id < b->id
                                                  : a->requestSince < b->requestSince;
    });
    for (auto *b : requests) {
        const auto &r = routes[b->route];
        bool allowed = true;
        for (const auto &q : bikes) {
            if (q.id == b->id)
                continue;
            // Do not pass an unadmitted leader or reserve an intersecting route.
            if ((!q.entered && !q.reserved && routes[q.route].side == r.side && q.s > b->s) ||
                (q.reserved && routesConflict(b->route, q.route)))
                allowed = false;
            // Keep the junction clear if its receiving lane is backed up.
            if (q.entered && q.v < .3 &&
                length(routes[q.route].position(q.s) - r.position(r.leave + 2)) < 2.7)
                allowed = false;
        }
        if (allowed) {
            b->reserved = true;
            b->yellowCommitted = false;
        }
    }
    std::sort(bikes.begin(), bikes.end(), [](const Bike &a, const Bike &b) { return a.s > b.s; });
    for (auto &b : bikes) {
        auto &r = routes[b.route];
        V p = r.position(b.s), dir = r.tangent(b.s);
        double free = 1e9;
        std::string reason = "CRUISE";
        if (!b.entered && !b.reserved) {
            free = std::max(0., r.stop - b.s);
            reason = green(r.side)                           ? "YIELD / CONFLICT"
                     : signal(r.side) == SignalColor::Yellow ? "YELLOW / BRAKE"
                                                             : "RED / STOP";
        } else if (b.yellowCommitted) {
            reason = "CLEAR ON YELLOW";
        } else if (b.entered && !green(r.side)) {
            reason = "CLEAR JUNCTION";
        }
        for (auto &q : bikes)
            if (q.id != b.id) {
                const auto &qr = routes[q.route];
                bool sameLane =
                    b.route == q.route || (r.side == qr.side && b.s < r.entry && q.s < qr.entry);
                if (sameLane && q.s > b.s && q.s - b.s - 2.7 < free) {
                    free = std::max(0., q.s - b.s - 2.7);
                    reason = "FOLLOW / QUEUE";
                }
                V d = qr.position(q.s) - p;
                double ahead = dot(d, dir), lateral = length(d - dir * ahead);
                if (ahead > 0 && lateral < 1.5 && ahead - 2.7 < free &&
                    dot(dir, qr.tangent(q.s)) > .3) {
                    free = std::max(0., ahead - 2.7);
                    reason = "FOLLOW / QUEUE";
                }
            }
        double target = std::min(7., sqrt(2 * 2.5 * free));
        if (b.s > r.entry - 3 && b.s < r.leave + 3)
            target = std::min(target, r.turn == 0 ? 5. : 3.2);
        double old = b.v;
        b.a = std::clamp((target - b.v) * 2., -4., 2.);
        b.v = std::max(0., b.v + b.a * dt);
        double ds = std::min(b.v * dt, free);
        V candidate = r.position(b.s + ds);
        for (auto &q : bikes)
            if (q.id != b.id && length(candidate - routes[q.route].position(q.s)) < 2.25) {
                ds = 0;
                b.v = 0;
                reason = "SAFETY GAP";
                break;
            }
        b.v = std::min(b.v, ds / dt);
        b.a = (b.v - old) / dt;
        b.s += ds;
        if (b.s > r.stop + .00001)
            b.entered = true;
        b.reason = b.v < 6.9 ? reason : "CRUISE";
        if (b.v < .15) {
            b.wait += dt;
            maxWait = std::max(maxWait, b.wait);
        }
    }
    bikes.erase(std::remove_if(bikes.begin(), bikes.end(),
                               [&](const Bike &b) {
                                   if (b.s >= routes[b.route].total) {
                                       passed++;
                                       totalWait += b.wait;
                                       return true;
                                   }
                                   return false;
                               }),
                bikes.end());
}
