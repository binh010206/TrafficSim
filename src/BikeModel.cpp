#include "BikeModel.h"
#include "Transform.h"
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>

// Honda VN product leaflets, Oct/Nov 2024. Body stations are photo estimates.
// See README for the exact editions and dimension conventions.
const BikeSpec &bikeSpec(BikeType type) {
    static const BikeSpec wave{"Honda Wave Alpha 110", 1.913,       .689, 1.076, 1.224, .770,
                               {70, 90, 17},           {80, 90, 17}};
    static const BikeSpec vision{
        "Honda Vision Standard", 1.871, .686, 1.101, 1.255, .761, {80, 90, 14}, {90, 90, 14}};
    return type == BikeType::Wave ? wave : vision;
}
namespace {
using F = Finish;
using J = Joint;
struct Station {
    double x, low, high, halfWidth;
};
using Stations = std::vector<Station>;
using Surface = std::function<V(double, double)>;
V axle(const BikeSpec &s, bool front) {
    return {s.wheelbase * (front ? .5 : -.5), (front ? s.front : s.rear).radius(), 0};
}
V origin(const BikeSpec &s, J j) { return j == J::Frame ? V{} : axle(s, j != J::RearWheel); }
double signedPower(double x, double power) {
    return std::copysign(std::pow(std::abs(x), power), x);
}

// Piecewise Hermite interpolation with bounded slopes. Passes through the actual
// authored stations, preserving peaks such as the seat reference height.
double interpolate(const Stations &a, int i, double t, double Station::*field) {
    auto slope = [&](int k) { return (a[k + 1].*field - a[k].*field) / (a[k + 1].x - a[k].x); };
    auto tangent = [&](int k) {
        if (k == 0)
            return slope(0);
        if (k == int(a.size()) - 1)
            return slope(k - 1);
        double l = slope(k - 1), r = slope(k);
        return l * r <= 0 ? 0. : 2 * l * r / (l + r);
    };
    double d = a[i + 1].x - a[i].x, q = 1 - t;
    return q * q * (1 + 2 * t) * (a[i].*field) + t * t * (3 - 2 * t) * (a[i + 1].*field) +
           d * t * q * (q * tangent(i) - t * tangent(i + 1));
}
// Invert the superellipse section to put a detail ON the authored skin.
// Flat polygons intersect a curved fairing and produce broken, jagged decals.
V skinSide(const Stations &stations, double power, double x, double y, int sign,
           double clearance = .002) {
    size_t i = 0;
    while (i + 2 < stations.size() && stations[i + 1].x < x)
        ++i;
    double t = (x - stations[i].x) / (stations[i + 1].x - stations[i].x);
    double lo = interpolate(stations, i, t, &Station::low),
           hi = interpolate(stations, i, t, &Station::high);
    double w = interpolate(stations, i, t, &Station::halfWidth);
    double cosine = std::pow(std::min(1., std::abs((2 * y - hi - lo) / (hi - lo))), 1. / power);
    double z = w * std::pow(std::max(0., 1 - cosine * cosine), power / 2);
    return {x, y, sign * (z + clearance)};
}
// Quadratic Bezier corner fillets; retain the outline rather than using an ellipse
// for every lamp and mirror. q(t) = (1-t)^2 A + 2t(1-t) B + t^2 C.
std::vector<V> roundedOutline(const std::vector<V> &corners, double radius, int samples) {
    std::vector<V> result;
    for (size_t i = 0; i < corners.size(); ++i) {
        V p = corners[i], a = p + (corners[(i + corners.size() - 1) % corners.size()] - p) * radius,
          c = p + (corners[(i + 1) % corners.size()] - p) * radius;
        for (int k = 0; k <= samples; ++k) {
            double t = double(k) / samples;
            result.push_back(a * ((1 - t) * (1 - t)) + p * (2 * t * (1 - t)) + c * (t * t));
        }
    }
    return result;
}
class Builder {
    BikeModel &model;
    const BikeSpec &spec;
    Mesh &mesh(F f) { return model.batches[{joint, f}]; }
    Vertex vertex(V p, V n) const { return {p - origin(spec, joint), n}; }

  public:
    bool distant() const { return model.distant; }
    J joint = J::Frame;
    Builder(BikeModel &m) : model(m), spec(bikeSpec(m.type)) {}
    void tri(F f, V a, V b, V c, V facing) {
        V n = cross(b - a, c - a);
        if (length(n) < 1e-12)
            return;
        if (dot(n, facing) < 0) {
            std::swap(b, c);
            n = n * (-1);
        }
        n = unit(n);
        auto &out = mesh(f).vertices;
        for (auto p : {a, b, c})
            out.push_back(vertex(p, n));
    }
    void grid(F f, int nu, int nv, const Surface &surface, bool flip = false) {
        std::vector<Vertex> samples((nu + 1) * (nv + 1));
        for (int i = 0; i <= nu; i++)
            for (int k = 0; k <= nv; k++) {
                double u = double(i) / nu, v = double(k) / nv, e = 1e-5;
                V du = surface(std::min(1., u + e), v) - surface(std::max(0., u - e), v);
                // v is also used for open patches; one-sided differences at their edges.
                V dv = surface(u, std::min(1., v + e)) - surface(u, std::max(0., v - e));
                V n = cross(du, dv);
                double l = length(n);
                samples[i * (nv + 1) + k] =
                    vertex(surface(u, v), l > 1e-20 ? n * ((flip ? -1. : 1.) / l) : V{0, 1, 0});
            }
        auto &out = mesh(f).vertices;
        for (int i = 0; i < nu; i++)
            for (int k = 0; k < nv; k++) {
                int a = i * (nv + 1) + k, b = a + nv + 1, c = b + 1, d = a + 1;
                std::array<int, 6> indices = flip ? std::array<int, 6>{a, c, b, a, d, c}
                                                  : std::array<int, 6>{a, b, c, a, c, d};
                for (int index : indices)
                    out.push_back(samples[index]);
            }
    }
    void tube(F f, V a, V b, double ra, double rb = 0, int slices = 12) {
        if (distant())
            slices = std::min(slices, 8);
        if (rb <= 0)
            rb = ra;
        V axis = unit(b - a),
          e = unit(cross(axis, std::abs(axis.y) < .9 ? V{0, 1, 0} : V{1, 0, 0})),
          g = cross(axis, e);
        auto radial = [&](double v) { return e * cos(2 * PI * v) + g * sin(2 * PI * v); };
        grid(
            f, 1, slices,
            [&](double u, double v) { return a + (b - a) * u + radial(v) * (ra + (rb - ra) * u); },
            true);
        for (int k = 0; k < slices; k++) {
            V p = radial(double(k) / slices), q = radial(double(k + 1) / slices);
            tri(f, a, a + p * ra, a + q * ra, axis * (-1));
            tri(f, b, b + p * rb, b + q * rb, axis);
        }
    }
    // Rounded longitudinal sections for seat cushions, side fairings and casings.
    void skin(F f, const Stations &s, double power = .72, double zCenter = 0,
              bool markSeat = false) {
        int n = int(s.size()) - 1;
        auto sample = [&](double u, double v) {
            double at = u * n;
            int i = std::min(n - 1, int(at));
            double t = at - i;
            double low = interpolate(s, i, t, &Station::low),
                   high = interpolate(s, i, t, &Station::high);
            double width = interpolate(s, i, t, &Station::halfWidth), ang = 2 * PI * v;
            return V{s[i].x + (s[i + 1].x - s[i].x) * t,
                     (high + low) * .5 + (high - low) * .5 * signedPower(cos(ang), power),
                     zCenter + width * signedPower(sin(ang), power)};
        };
        int ringSamples = distant() ? 12 : 32;
        grid(f, n * (distant() ? 2 : 6), ringSamples, sample, true);
        for (int end : {0, 1})
            for (int k = 0; k < ringSamples; k++) {
                V p = sample(end, double(k) / ringSamples),
                  q = sample(end, double(k + 1) / ringSamples);
                auto st = end ? s.back() : s.front();
                tri(f, {st.x, (st.low + st.high) * .5, zCenter}, p, q, {end ? 1. : -1., 0, 0});
            }
        if (markSeat) {
            for (auto st : s)
                model.seatControls.push_back({st.x, st.high, zCenter});
        }
    }
    // A pressed panel described by a perimeter, a bevel and a crowned face.
    // The loop must be star-shaped about its centre. All coordinates are in meters.
    void panel(F f, const std::vector<V> &outline, V normal, double bevel) {
        V center{};
        for (auto p : outline)
            center = center + p;
        center = center * (1. / outline.size());
        normal = unit(normal);
        std::vector<V> prev = outline;
        for (auto layer : {std::pair<double, double>{.92, .62}, {.70, .92}, {.40, 1.0}}) {
            std::vector<V> next;
            for (V p : outline)
                next.push_back(center + (p - center) * layer.first +
                               normal * (bevel * layer.second));
            for (size_t i = 0; i < prev.size(); i++) {
                size_t k = (i + 1) % prev.size();
                tri(f, prev[i], prev[k], next[i], normal);
                tri(f, prev[k], next[k], next[i], normal);
            }
            prev = next;
        }
        for (size_t i = 0; i < prev.size(); i++)
            tri(f, prev[i], prev[(i + 1) % prev.size()], center + normal * bevel, normal);
    }
    // Optical unit with a shaped black gasket, metal reflector surround and separate lens.
    void optical(const std::vector<V> &perimeter, V normal, F lens, double depth) {
        V mid{};
        for (V p : perimeter)
            mid = mid + p;
        mid = mid * (1. / perimeter.size());
        normal = unit(normal);
        auto layer = [&](double scale, double step, F f, double crown) {
            std::vector<V> ring;
            for (V p : perimeter)
                ring.push_back(mid + (p - mid) * scale + normal * step);
            panel(f, ring, normal, crown);
        };
        for (size_t i = 0; i < perimeter.size(); ++i) {
            V a = perimeter[i], b = perimeter[(i + 1) % perimeter.size()],
              delta = normal * (depth * 1.7);
            V facing = (a + b) * .5 - mid;
            tri(F::Graphite, a, b, b - delta, facing);
            tri(F::Graphite, a, b - delta, a - delta, facing);
        }
        layer(1., 0, F::Graphite, depth * .28);
        layer(.92, depth * .32, F::Bright, depth * .25);
        layer(.82, depth * .63, lens, depth * .32);
        // Prismatic reflector flutes across the clear lens.
        V across = unit(cross({0, 1, 0}, normal));
        V up = unit(cross(normal, across));
        double w = 0, h = 0;
        for (V p : perimeter) {
            w = std::max(w, std::abs(dot(p - mid, across)));
            h = std::max(h, std::abs(dot(p - mid, up)));
        }
        if (!distant() && lens == F::Lens)
            for (int i = -3; i <= 3; i++) {
                V base = mid + normal * depth + across * (i * w * .15);
                tube(F::Reflector, base - up * h * .32, base + up * h * .35, .0009, 0, 6);
            }
    }
    // Extruded curved alloy spoke (not a solid wheel disk).
    void blade(V center, double angle, double hub, double rimR) {
        std::vector<V> sides;
        for (auto rw : {std::pair<double, double>{hub, -.22},
                        {rimR * .68, -.04},
                        {rimR, .26},
                        {rimR, .38},
                        {rimR * .60, .17},
                        {hub, .35}})
            sides.push_back(center + V{rw.first * cos(angle + rw.second),
                                       rw.first * sin(angle + rw.second), 0});
        auto face = sides;
        for (auto &p : face)
            p.z += .009;
        panel(F::Alloy, face, {0, 0, 1}, .004);
        auto back = sides;
        for (auto &p : back)
            p.z -= .009;
        panel(F::Alloy, back, {0, 0, -1}, .004);
        for (size_t i = 0; i < sides.size(); ++i) {
            auto k = (i + 1) % sides.size();
            V n = cross(face[k] - face[i], back[i] - face[i]);
            tri(F::DarkMetal, face[i], face[k], back[i], n);
            tri(F::DarkMetal, face[k], back[k], back[i], n);
        }
    }
};
// Subdivide a decal in the side-view plane BEFORE mapping it onto the fairing.
// Subdivision prevents the interior of each triangle cutting through the body.
void bodyDecal(Builder &b, F finish, const Stations &stations, double power, int sign,
               const std::vector<V> &outline) {
    V centre{};
    for (auto p : outline)
        centre = centre + p;
    centre = centre * (1. / outline.size());
    auto mapped = [&](V p) { return skinSide(stations, power, p.x, p.y, sign); };
    int n = b.distant() ? 5 : 14;
    for (size_t k = 0; k < outline.size(); ++k) {
        V a = outline[k], c = outline[(k + 1) % outline.size()];
        auto point = [&](int i, int j) {
            return mapped(centre + (a - centre) * (double(i) / n) + (c - centre) * (double(j) / n));
        };
        for (int i = 0; i < n; ++i)
            for (int j = 0; j < n - i; ++j) {
                b.tri(finish, point(i, j), point(i + 1, j), point(i, j + 1), {0, 0, double(sign)});
                if (i + j + 1 < n)
                    b.tri(finish, point(i + 1, j), point(i + 1, j + 1), point(i, j + 1),
                          {0, 0, double(sign)});
            }
    }
}
void wheel(Builder &b, const BikeSpec &s, bool front, bool wave) {
    b.joint = front ? J::FrontWheel : J::RearWheel;
    Tire tire = front ? s.front : s.rear;
    V c = axle(s, front);
    double rim = tire.rimRadius(), outer = tire.radius(), width = tire.widthMm / 1000.;
    // Pneumatic tire: outer radius and widest section follow the nominal marking.
    double minor = (outer - rim) / 2, major = (outer + rim) / 2;
    auto tirePoint = [&](double a, double t) {
        double r = major + minor * cos(t);
        return c + V{r * cos(a), r * sin(a), width * .5 * sin(t)};
    };
    b.grid(F::Rubber, b.distant() ? 32 : 64, b.distant() ? 12 : 20,
           [&](double u, double v) { return tirePoint(2 * PI * u, 2 * PI * v); });
    for (int side : {-1, 1})
        for (int i = 0; i < (b.distant() ? 0 : 48); i++) {
            double a = 2 * PI * i / 48;
            b.grid(
                F::Graphite, 3, 1,
                [&](double u, double v) {
                    double az = a + .052 * u + .010 * v;
                    return tirePoint(az, side * (.12 + .97 * u)) +
                           V{.0003 * cos(az), .0003 * sin(az), 0};
                },
                side > 0);
        }
    auto ring = [&](F f, double r, double radial, double axial, double z) {
        b.grid(f, b.distant() ? 24 : 48, b.distant() ? 6 : 10, [&](double u, double v) {
            double a = 2 * PI * u, t = 2 * PI * v;
            return c + V{(r + radial * cos(t)) * cos(a), (r + radial * cos(t)) * sin(a),
                         z + axial * sin(t)};
        });
    };
    ring(wave ? F::Bright : F::Alloy, rim - .007, .011, width * .30, 0);
    for (int sign : {-1, 1})
        ring(F::Bright, rim - .003, .004, .003, sign * width * .29);
    double hub = wave ? .059 : .038;
    b.tube(F::Alloy, c + V{0, 0, -.042}, c + V{0, 0, .042}, hub, 0, 30);
    if (wave) {
        for (int k = 0; k < 36; k++) {
            double a = k * 2 * PI / 36, sgn = k % 2 ? 1 : -1, shift = (k % 4 < 2 ? .45 : -.45);
            b.tube(F::Bright, c + V{hub * cos(a + shift), hub * sin(a + shift), sgn * .039},
                   c + V{(rim - .012) * cos(a), (rim - .012) * sin(a), sgn * .009}, .0017, 0, 5);
        }
    } else {
        for (int k = 0; k < 8; k++)
            b.blade(c, 2 * PI * k / 8, hub, rim - .01);
        if (front) {
            // Annular brake rotor with openings between bridges and outer braking track.
            ring(F::Bright, .094, .009, .0025, -.041);
            for (int k = 0; k < 8; k++) {
                double a = k * PI / 4;
                b.tube(F::DarkMetal, c + V{.032 * cos(a), .032 * sin(a), -.041},
                       c + V{.093 * cos(a + .27), .093 * sin(a + .27), -.041}, .007, 0, 6);
            }
        }
    }
    b.joint = J::Fork;
    if (!wave && front)
        b.tube(F::DarkMetal, c + V{-.063, .038, -.044}, c + V{-.090, .080, -.044}, .018);
}
// A fender is a thin curved shell around the top of the wheel, with rolled edges.
void fender(Builder &b, V axlePos, double radius, double halfWidth, F f, double start, double end) {
    auto top = [&](double u, double v) {
        double a = (start + (end - start) * u) * PI / 180, t = (v - .5) * PI;
        double span = .80 + .20 * sin(PI * u),
               r = radius + .032 + .025 * cos(t) - .014 * (1 - cos(t)) * sin(PI * u);
        return axlePos + V{r * cos(a), r * sin(a), halfWidth * span * sin(t)};
    };
    b.grid(f, 36, 22, top);
    for (int side : {0, 1})
        b.grid(
            f, 36, 3,
            [&](double u, double v) {
                V p = top(u, side);
                return p + V{0, -v * (.004 + .021 * sin(PI * u)), (side ? 1 : -1) * .002 * v};
            },
            side == 1);
}
void shock(Builder &b, V a, V end) {
    b.tube(F::Bright, a, end, .014);
    if (b.distant()) {
        b.tube(F::DarkMetal, a + (end - a) * .18, a + (end - a) * .88, .022);
        return;
    }
    V dir = unit(end - a), e = unit(cross(dir, {0, 0, 1})), n = cross(dir, e);
    V last{};
    for (int i = 0; i <= 72; i++) {
        double u = i / 72., t = 2 * PI * 8 * u;
        V p = a + (end - a) * (.15 + .73 * u) + (e * cos(t) + n * sin(t)) * .021;
        if (i)
            b.tube(F::DarkMetal, last, p, .004, 0, 6);
        last = p;
    }
}

// A section across the leg shield at a given elevation (rear skin, nose and half-width).
// Reuse the station interpolator vertically: x = height, low/high = back/nose.
using Shield = Stations;
V shieldPoint(const Shield &rows, double u, double theta) {
    double t = u * (rows.size() - 1);
    int i = std::min(int(rows.size()) - 2, int(t));
    t -= i;
    double back = interpolate(rows, i, t, &Station::low),
           nose = interpolate(rows, i, t, &Station::high);
    double width = interpolate(rows, i, t, &Station::halfWidth);
    return {(back + nose) * .5 + (nose - back) * .5 * signedPower(cos(theta), .80),
            rows[i].x * (1 - t) + rows[i + 1].x * t, width * signedPower(sin(theta), .80)};
}
void legShield(Builder &b, const Shield &rows, bool indicators) {
    for (int k = 0; k < 4; k++)
        b.grid(k == 0 || k == 3 ? F::Paint : F::Graphite, (rows.size() - 1) * 5, 16,
               [&](double u, double v) { return shieldPoint(rows, u, (k + v) * PI / 2); });
    for (int end : {0, 1})
        for (int k = 0; k < 32; k++) {
            V a = shieldPoint(rows, end, k * PI / 16),
              c = shieldPoint(rows, end, (k + 1) * PI / 16);
            auto r = end ? rows.back() : rows.front();
            b.tri(F::Graphite, {(r.high + r.low) / 2, r.x, 0}, a, c, {0, end ? 1. : -1., 0});
        }
    if (indicators)
        for (int sign : {-1, 1}) {
            // Map every lens vertex onto the same curved apron. An inset scaled in
            // Cartesian space can otherwise disappear inside the convex panel.
            auto strip = [&](F finish, double lo, double hi, double width, double offset) {
                b.grid(
                    finish, 22, 8,
                    [&](double u, double v) {
                        double angle = .29 + .66 * u + width * (2 * v - 1) * (.17 + sin(PI * u));
                        V p = shieldPoint(rows, lo + (hi - lo) * u, sign * angle);
                        return p + V{offset, 0, sign * offset * .60};
                    },
                    sign < 0);
            };
            strip(F::Graphite, .47, .91, .29, .004);
            strip(F::Bright, .485, .895, .25, .008);
            strip(F::Lens, .502, .882, .207, .012);
            strip(F::Amber, .675, .729, .103, .017);
        }
    else
        for (int sign : {-1, 1}) {
            // Lower side opening of the underbone's leg shield, with three moulded ribs.
            auto vent = [&](double u, double v) {
                double angle = 1.12 + (.08 + .40 * sin(PI * u)) * (v - .5);
                return shieldPoint(rows, .31 + .24 * u, sign * angle) + V{.0025, 0, sign * .003};
            };
            b.grid(F::Graphite, b.distant() ? 10 : 24, 8, vent, sign < 0);
            for (int rib = 1; rib <= 3; ++rib) {
                V previous = vent(.20 * rib, .10);
                for (int k = 1; k <= 8; ++k) {
                    V p = vent(.20 * rib, .10 + .8 * k / 8);
                    b.tube(F::Paint, previous, p, .0035, 0, 6);
                    previous = p;
                }
            }
            if (!b.distant())
                for (double u : {.41, .78}) {
                    V p = shieldPoint(rows, u, sign * 1.48);
                    b.tube(F::DarkMetal, p, p + V{.001, 0, sign * .004}, .0045, 0, 8);
                }
        }
}
void stitchedSeat(Builder &b, const Stations &steps) {
    Stations base = steps;
    for (auto &st : base) {
        double bottom = st.low;
        st.high = bottom + .013;
        st.low = bottom - .075;
        st.halfWidth *= .94;
    }
    b.skin(F::Graphite, base, .54);
    b.skin(F::Seat, steps, .63, 0, true);
    if (b.distant())
        return;
    for (int sign : {-1, 1}) {
        V previous{};
        int count = (int(steps.size()) - 1) * 6;
        for (int k = 0; k <= count; k++) {
            int i = std::min(int(steps.size()) - 2, k / 6);
            double t = k / 6. - i;
            double low = interpolate(steps, i, t, &Station::low),
                   hi = interpolate(steps, i, t, &Station::high);
            V p{steps[i].x * (1 - t) + steps[i + 1].x * t, low + .64 * (hi - low),
                sign * interpolate(steps, i, t, &Station::halfWidth) * .98};
            if (k)
                b.tube(F::Graphite, previous, p, .0017, 0, 6);
            previous = p;
        }
    }
}
void tail(Builder &b, const BikeSpec &s, bool wave) {
    // Tire is the foremost point at straight ahead. The lower rear mud flap
    // fixes nominal overall length instead of stretching an entire bike mesh.
    double xEnd = axle(s, true).x + s.front.radius() - s.length;
    double x = xEnd + (wave ? .13 : .095), y = wave ? .757 : .768;
    b.skin(F::Graphite,
           {{xEnd, .377, .395, .045},
            {xEnd + .035, .383, .485, .07},
            {x + .05, .60, y, .085},
            {x + .13, .65, y + .015, .045}},
           .6);
    b.panel(F::Reflector,
            {{xEnd + .009, .421, -.086},
             {xEnd + .009, .421, .086},
             {xEnd + .029, .490, .086},
             {xEnd + .029, .490, -.086}},
            {-1, .28, 0}, .002);
    b.optical({{x - .012, y + .036, -.108},
               {x - .025, y + .032, 0},
               {x - .012, y + .036, .108},
               {x + .018, y - .031, .087},
               {x - .008, y - .042, 0},
               {x + .018, y - .031, -.087}},
              {-1, .16, 0}, F::Tail, .010);
    for (int sign : {-1, 1}) {
        b.tube(F::Alloy, {x + .27, y + .038, sign * .124}, {x + .089, y + .099, sign * .132}, .011);
        b.tube(F::Alloy, {x + .089, y + .099, sign * .132}, {x + .045, y + .11, sign * .08}, .011);
    }
    b.tube(F::Alloy, {x + .045, y + .11, -.08}, {x + .045, y + .11, .08}, .011);
}
void exhaust(Builder &b, bool wave) {
    double z = wave ? .134 : .172;
    std::vector<V> header = {
        {.09, .273, .045}, {.14, .205, .083}, {.10, .158, z}, {-.11, .168, z}, {-.30, .221, z}};
    for (size_t i = 1; i < header.size(); i++)
        b.tube(F::DarkMetal, header[i - 1], header[i], .016);
    Stations pipe = {{-.88, .344, .420, .025},
                     {-.83, .310, .438, .050},
                     {-.52, .237, .360, .063},
                     {-.28, .197, .279, .043},
                     {-.23, .205, .24, .020}};
    b.skin(F::DarkMetal, pipe, .65, z);
    b.skin(wave ? F::Bright : F::Graphite,
           {{-.86, .365, .417, .009},
            {-.78, .337, .427, .019},
            {-.53, .273, .370, .022},
            {-.34, .242, .300, .016},
            {-.32, .253, .269, .005}},
           .55, z + .047);
    for (V p : {V{-.77, .379, z + .066}, V{-.43, .295, z + .067}})
        b.tube(F::Alloy, p, p + V{0, 0, .004}, .006, 0, 6);
    b.tube(F::Rubber, {-.877, .389, z}, {-.886, .39, z}, .017, 0, 16);
}
// The badge follows the outer skin rather than a flat sign next to the bike.
void sideBadge(Builder &b, const Stations &stations, double power, bool wave) {
    if (b.distant())
        return;
    const std::string word = wave ? "WAVE" : "VISION";
    auto glyph = [](char c) -> std::vector<std::vector<V>> {
        switch (c) {
        case 'W':
            return {{{0, 1}, {.12, 0}, {.5, .53}, {.73, 0}, {1, 1}}};
        case 'A':
            return {{{0, 0}, {.46, 1}, {1, 0}}, {{.2, .4}, {.75, .4}}};
        case 'V':
            return {{{0, 1}, {.52, 0}, {1, 1}}};
        case 'E':
            return {{{1, 1}, {0, 1}, {0, 0}, {1, 0}}, {{0, .52}, {.76, .52}}};
        case 'I':
            return {{{.48, 0}, {.48, 1}}};
        case 'S':
            return {{{1, .90},
                     {.75, 1},
                     {.13, 1},
                     {0, .72},
                     {.15, .56},
                     {.84, .44},
                     {1, .28},
                     {.85, 0},
                     {.15, 0},
                     {0, .10}}};
        case 'O':
            return {{{.20, 1},
                     {.80, 1},
                     {1, .78},
                     {1, .20},
                     {.80, 0},
                     {.20, 0},
                     {0, .20},
                     {0, .78},
                     {.20, 1}}};
        default:
            return {{{0, 0}, {0, 1}, {1, 0}, {1, 1}}};
        }
    };
    for (int sign : {-1, 1}) {
        auto point = [&](size_t letter, V g) {
            const double advance = wave ? .039 : .028, baseX = wave ? -.745 : -.735;
            double distance = advance * (letter + .77 * g.x + .14 * g.y);
            double x = baseX + (sign > 0 ? distance : advance * word.size() - distance);
            double y = (wave ? .755 : .724) - .23 * (x - baseX) + .017 * g.y;
            return skinSide(stations, power, x, y, sign, .003);
        };
        for (size_t i = 0; i < word.size(); ++i)
            for (const auto &stroke : glyph(word[i]))
                for (size_t k = 1; k < stroke.size(); ++k)
                    b.tube(wave ? F::Decal : F::Bright, point(i, stroke[k - 1]),
                           point(i, stroke[k]), .0015, 0, 6);
    }
}
void waveBody(Builder &b, const BikeSpec &s) {
    tail(b, s, true);
    // Black step-through frame, narrow upper side covers and horizontal cylinder.
    b.skin(F::Graphite,
           {{-.89, .715, .776, .052},
            {-.71, .655, .801, .112},
            {-.46, .516, .765, .133},
            {-.27, .445, .712, .124},
            {-.10, .39, .582, .094},
            {.04, .39, .49, .066},
            {.28, .50, .61, .045}},
           .65);
    const Stations sideCover = {{-.895, .746, .794, .062}, {-.78, .706, .812, .126},
                                {-.64, .660, .793, .145},  {-.42, .558, .755, .141},
                                {-.26, .530, .706, .126},  {-.09, .493, .571, .092},
                                {-.048, .505, .523, .062}};
    b.skin(F::Paint, sideCover, .60);
    sideBadge(b, sideCover, .60, true);
    stitchedSeat(b, {{-.846, .794, .801, .055},
                     {-.78, .792, .832, .118},
                     {-.63, .770, .821, .146},
                     {-.40, .725, .791, .145},
                     {-.215, .689, s.seatHeight, .124},
                     {-.108, .676, .752, .099},
                     {-.078, .682, .713, .051}});
    legShield(b,
              {{.184, .183, .224, .055},
               {.308, .202, .300, .083},
               {.480, .226, .393, .133},
               {.664, .289, .511, .152},
               {.802, .331, .564, .155},
               {.909, .315, .476, .102},
               {.944, .304, .411, .055}},
              false);
    b.skin(F::Graphite,
           {{-.22, .29, .414, .075},
            {-.04, .287, .438, .088},
            {.148, .319, .404, .062},
            {.241, .196, .311, .064}},
           .7);
    // Cast crankcase, finned cylinder, gear lever, right-side brake pedal and rider pegs.
    b.tube(F::DarkMetal, {-.044, .299, -.115}, {-.044, .299, .110}, .091, 0, 32);
    b.tube(F::Alloy, {-.044, .299, .109}, {-.044, .299, .119}, .075, 0, 32);
    for (int i = 0; i < 6; i++) {
        double a = i * PI / 3;
        b.tube(F::Bright, {-.044 + .078 * cos(a), .299 + .078 * sin(a), .107},
               {-.044 + .078 * cos(a), .299 + .078 * sin(a), .117}, .005, 0, 6);
    }
    for (int k = 0; k < 7; k++)
        b.tube(F::Alloy, {.055 + k * .014, .300 + k * .002, 0},
               {.060 + k * .014, .301 + k * .002, 0}, .060, 0, 12);
    b.tube(F::DarkMetal, {-.10, .218, -.203}, {-.10, .218, .203}, .012);
    for (int sign : {-1, 1}) {
        b.tube(F::Rubber, {-.10, .219, sign * .118}, {-.10, .219, sign * .202}, .019);
        b.tube(F::Alloy, {-.45, .315, sign * .074}, {-.425, .380, sign * .152}, .012);
        b.tube(F::Rubber, {-.425, .38, sign * .15}, {-.425, .38, sign * .222}, .013);
        b.tube(F::DarkMetal, {-.625, .29, sign * .059}, {-.14, .281, sign * .060}, .020);
        shock(b, {-.588, .307, sign * .076}, {-.487, .585, sign * .088});
        // Tapered printed graphics conform to the side panel, including their interior.
        bodyDecal(b, F::Decal, sideCover, .60, sign,
                  {{-.748, .752}, {-.665, .740}, {-.216, .559}, {-.477, .637}});
        bodyDecal(b, F::Accent, sideCover, .60, sign,
                  {{-.700, .778}, {-.641, .757}, {-.300, .611}, {-.554, .703}});
    }
    b.skin(F::DarkMetal,
           {{-.671, .243, .316, .017},
            {-.609, .235, .336, .027},
            {-.27, .247, .307, .024},
            {-.166, .26, .30, .015}},
           .60, -.079);
    b.tube(F::Alloy, {-.115, .23, -.16}, {.052, .26, -.17}, .008);
    b.tube(F::Rubber, {.052, .26, -.168}, {.052, .26, -.213}, .010);
    // Right crank cover has two cast lobes; boltheads and kick lever sit proud.
    b.tube(F::DarkMetal, {-.094, .286, .103}, {-.094, .286, .119}, .047, 0, 24);
    b.tube(F::Alloy, {-.043, .298, .117}, {-.043, .298, .123}, .050, 0, 28);
    b.tube(F::Bright, {-.109, .286, .125}, {-.158, .396, .138}, .008);
    b.tube(F::Rubber, {-.158, .396, .132}, {-.158, .396, .194}, .010);
    b.tube(F::Alloy, {-.102, .212, .165}, {.075, .210, .17}, .008);
    b.tube(F::DarkMetal, {.072, .208, .143}, {.072, .208, .196}, .010);
    exhaust(b, true);
}
void visionBody(Builder &b, const BikeSpec &s) {
    tail(b, s, false);
    b.skin(F::Graphite,
           {{-.89, .660, .770, .057},
            {-.76, .547, .802, .146},
            {-.55, .461, .79, .169},
            {-.34, .341, .717, .167},
            {-.18, .265, .655, .145},
            {-.085, .243, .500, .118},
            {-.047, .251, .310, .107}},
           .57);
    const Stations sideCover = {{-.9, .721, .789, .058},   {-.79, .686, .807, .144},
                                {-.59, .594, .788, .169},  {-.407, .450, .735, .188},
                                {-.296, .397, .641, .185}, {-.240, .395, .540, .181},
                                {-.224, .425, .490, .177}};
    b.skin(F::Paint, sideCover, .56);
    sideBadge(b, sideCover, .56, false);
    stitchedSeat(b, {{-.827, .786, .804, .060},
                     {-.768, .775, .850, .130},
                     {-.61, .753, .832, .169},
                     {-.44, .719, .799, .165},
                     {-.26, .682, s.seatHeight, .139},
                     {-.165, .674, .76, .106},
                     {-.123, .686, .727, .073}});
    // Lower body sweeps under a continuous treaded footboard.
    b.skin(F::Paint,
           {{-.40, .215, .384, .111},
            {-.27, .177, .321, .157},
            {-.05, .152, .275, .168},
            {.173, .156, .276, .17},
            {.310, .182, .326, .157},
            {.366, .211, .382, .107}},
           .46);
    b.skin(F::Rubber,
           {{-.29, .282, .306, .11},
            {-.205, .253, .286, .150},
            {-.06, .247, .269, .162},
            {.125, .252, .273, .158},
            {.249, .275, .307, .148},
            {.308, .304, .344, .124}},
           .40);
    for (int sign : {-1, 1})
        for (int k = 0; k < 4; k++) {
            double z = sign * (.046 + k * .025);
            b.tube(F::Graphite, {-.14, .275, z}, {.135, .276, z}, .003, 0, 6);
        }
    legShield(b,
              {{.212, .239, .355, .125},
               {.346, .238, .426, .179},
               {.515, .270, .483, .166},
               {.655, .291, .568, .177},
               {.749, .317, .614, .178},
               {.869, .316, .573, .155},
               {.934, .312, .479, .092},
               {.950, .306, .409, .056}},
              true);
    // CVT casing and airbox on the left, silencer on the right; single rear shock.
    b.skin(F::DarkMetal,
           {{-.77, .208, .319, .027},
            {-.713, .184, .365, .055},
            {-.57, .176, .362, .068},
            {-.40, .185, .326, .06},
            {-.235, .191, .35, .059},
            {-.168, .22, .322, .025}},
           .65, -.083);
    b.skin(F::Graphite,
           {{-.76, .345, .388, .025},
            {-.66, .339, .448, .049},
            {-.45, .35, .452, .054},
            {-.31, .36, .410, .025}},
           .48, -.087);
    b.tube(F::Alloy, {-.626, .256, -.126}, {-.626, .256, -.151}, .036, 0, 24);
    shock(b, {-.611, .264, -.064}, {-.479, .587, -.098});
    for (int sign : {-1, 1}) {
        b.tube(F::Alloy, {-.447, .349, sign * .100}, {-.363, .330, sign * .174}, .013);
        // Small crease below the separate chrome nameplate; not a floating flat plate.
        bodyDecal(b, F::Graphite, sideCover, .56, sign,
                  {{-.756, .718}, {-.554, .660}, {-.380, .534}, {-.574, .644}});
    }
    exhaust(b, false);
}

void steering(Builder &b, const BikeSpec &s, bool wave) {
    b.joint = J::Fork;
    V c = axle(s, true);
    double stem = wave ? .317 : .331, head = s.height;
    for (int sign : {-1, 1}) {
        double z = sign * (wave ? .069 : .078);
        V base = c + V{0, 0, z}, knee = c + V{-.054, .207, z}, top = c + V{-.108, .391, z};
        b.tube(wave ? F::Graphite : F::Alloy, base, knee, .020, .024);
        b.tube(F::Bright, knee, top, .0145);
        b.tube(F::Rubber, knee, knee + V{-.006, .026, 0}, .022);
        b.tube(F::Alloy, base, base + V{0, 0, sign * .012}, .014, 0, 6);
    }
    b.tube(F::DarkMetal, c + V{-.12, .383, -.078}, c + V{-.12, .383, .078}, .021);
    b.tube(F::DarkMetal, c + V{-.12, .38, 0}, {stem, head - .075, 0}, .029);
    fender(b, c, s.front.radius(), wave ? .057 : .070, F::Paint, 37, 157);
    // Lower bar enclosure and lacquered handlebar nacelle surround the headlamp.
    b.skin(F::Graphite,
           {{stem - .095, head - .107, head - .056, .105},
            {stem - .049, head - .131, head - .046, .171},
            {stem + .035, head - .137, head - .046, .168},
            {stem + .065, head - .117, head - .058, .144}},
           .59);
    b.skin(F::Paint,
           {{stem - .1, head - .069, head - .054, .088},
            {stem - .063, head - .091, head - .009, .152},
            {stem + .023, head - .104, head, .184},
            {stem + .052, head - .106, head - .025, .172},
            {stem + .070, head - .089, head - .039, .145}},
           .62);
    // Lens vertices follow a sloping front face. Vision's central reflector is
    // narrower; Wave integrates the turn lamps into the handlebar housing.
    double face = stem + .155, w = wave ? .125 : .104;
    auto lens = [&](double z, double y) {
        return V{face - .50 * y - .022 * std::abs(z / w), head - .077 + y, z};
    };
    std::vector<V> headlightRim =
        roundedOutline({lens(-w * .96, .054), lens(-w * .43, .063), lens(w * .43, .063),
                        lens(w * .96, .054), lens(w * .89, -.019), lens(w * .53, -.061),
                        lens(0, -.067), lens(-w * .53, -.061), lens(-w * .89, -.019)},
                       .24, b.distant() ? 1 : 4);
    V rimCenter{};
    for (V p : headlightRim)
        rimCenter = rimCenter + p;
    rimCenter = rimCenter * (1. / headlightRim.size());
    for (size_t i = 0; i < headlightRim.size(); ++i) {
        V a = headlightRim[i], b2 = headlightRim[(i + 1) % headlightRim.size()];
        auto back = [&](V p) {
            return V{stem + .038, rimCenter.y + (p.y - rimCenter.y) * .80, p.z * 1.12};
        };
        auto trim = (a.y + b2.y) * .5 < rimCenter.y - .013 ? F::Graphite : F::Paint;
        V outward = (a + b2) * .5 - rimCenter;
        b.tri(trim, a, b2, back(a), outward);
        b.tri(trim, b2, back(b2), back(a), outward);
    }
    b.optical(headlightRim, {1, .50, 0}, F::Lens, .009);
    for (int sign : {-1, 1}) {
        if (wave) {
            auto indicator = roundedOutline({lens(sign * .124, .042), lens(sign * .158, .032),
                                             lens(sign * .182, .009), lens(sign * .167, -.027),
                                             lens(sign * .123, -.033)},
                                            .26, b.distant() ? 1 : 3);
            for (size_t k = 0; k < indicator.size(); ++k) {
                V a = indicator[k], c2 = indicator[(k + 1) % indicator.size()];
                V backA{stem + .045, a.y, a.z}, backC{stem + .045, c2.y, c2.z};
                V facing = (a + c2) * .5 - lens(sign * .148, .003);
                b.tri(F::Paint, a, c2, backA, facing);
                b.tri(F::Paint, c2, backC, backA, facing);
            }
            b.optical(indicator, {1, .28, sign * .42}, F::Amber, .006);
        }
        double edge = s.width * .5;
        V grip1{stem - .044, head - .066, sign * .212},
            grip2{stem - .061, head - .072, sign * (edge - .005)};
        b.tube(F::DarkMetal, {stem, head - .075, sign * .125}, grip1, .019);
        b.tube(F::Graphite, grip1, grip1 + V{0, 0, sign * .026}, .028);
        b.tube(F::Rubber, grip1 + V{0, 0, sign * .025}, grip2, .0175);
        for (int k = 0; k < (b.distant() ? 0 : 7); k++) {
            double t = .29 + .092 * k;
            V pos = grip1 + (grip2 - grip1) * t;
            b.tube(F::Graphite, pos, pos + V{0, 0, sign * .002}, .0182, 0, 10);
        }
        b.tube(F::DarkMetal, {grip2.x, grip2.y, grip2.z}, {grip2.x, grip2.y, sign * edge}, .017);
        if (!wave || sign > 0) {
            V hinge{stem, head - .072, sign * .189}, bend{stem + .010, head - .073, sign * .235};
            b.tube(F::Alloy, hinge, bend, .007);
            b.tube(F::Alloy, bend, {stem - .018, head - .081, sign * (edge - .005)}, .006, .0035);
        }
        // Back shell, silvered glass and stem are independent parts.
        V base{stem - .035, head - .023, sign * .177}, neck{stem - .050, head + .080, sign * .225};
        V mid{stem - .062, head + .142, sign * .261};
        b.tube(F::Graphite, base, base + V{0, .031, sign * .008}, .010);
        b.tube(F::DarkMetal, base, neck, .006);
        b.tube(F::DarkMetal, neck, mid, .006);
        std::vector<V> outline;
        for (auto zy : {std::pair<double, double>{-.061, -.017},
                        {-.043, -.040},
                        {.034, -.025},
                        {.065, -.006},
                        {.065, .020},
                        {.038, .044},
                        {-.025, .041},
                        {-.062, .015}})
            outline.push_back(mid + V{.010 * zy.first / .065, zy.second, sign * zy.first});
        outline = roundedOutline(outline, .37, b.distant() ? 1 : 4);
        b.panel(F::Graphite, outline, {1, .15, sign * .16}, .020);
        b.panel(F::Graphite, outline, {-1, .1, -sign * .16}, .006);
        for (auto &p : outline)
            p = mid + (p - mid) * .86 + V{-.009, 0, 0};
        b.panel(F::Mirror, outline, {-1, .15, -sign * .16}, .003);
    }
    // Recessed analog cluster on the rider-facing sloped surface.
    V mid{stem - .078, head - .027, 0}, across{0, 0, 1}, up{.034, .019, 0};
    std::vector<V> dial;
    for (int k = 0; k < 32; k++) {
        double a = 2 * PI * k / 32;
        dial.push_back(mid + across * (.086 * cos(a)) + up * sin(a));
    }
    b.panel(F::DarkMetal, dial, {-1, .73, 0}, .005);
    for (int k = 0; k < 11; k++) {
        double a = PI * k / 10;
        V p = mid + V{-.006, .005, 0} + across * (.07 * cos(a)) + up * (.79 * sin(a));
        b.tube(F::Reflector, p, p - across * (.005 * cos(a)) - up * (.11 * sin(a)), .0014, 0, 5);
    }
    b.tube(F::Amber, mid + V{-.01, .007, 0}, mid + V{-.01, .007, -.052}, .0016, 0, 6);
}
V pigment(F f, bool wave, bool braking) {
    switch (f) {
    case F::Paint:
        return wave ? V{.034, .22, .52} : V{.93, .91, .84};
    case F::Graphite:
        return {.068, .078, .091};
    case F::Rubber:
        return {.031, .035, .040};
    case F::Seat:
        return {.070, .065, .068};
    case F::DarkMetal:
        return {.174, .189, .206};
    case F::Alloy:
        return {.52, .57, .62};
    case F::Bright:
        return {.78, .81, .84};
    case F::Lens:
        return {.80, .85, .86};
    case F::Reflector:
        return {.92, .95, .95};
    case F::Amber:
        return {1, .35, .055};
    case F::Tail:
        return braking ? V{1, .065, .028} : V{.57, .028, .022};
    case F::Mirror:
        return {.58, .75, .84};
    case F::Accent:
        return {.90, .42, .08};
    case F::Decal:
        return {.84, .89, .90};
    }
    return {};
}
} // namespace
void BikeModel::build(BikeType kind, bool distantDetail) {
    distant = distantDetail;
    type = kind;
    Builder b(*this);
    const auto &s = bikeSpec(type);
    bool wave = type == BikeType::Wave;
    if (wave)
        waveBody(b, s);
    else
        visionBody(b, s);
    steering(b, s, wave);
    wheel(b, s, true, wave);
    wheel(b, s, false, wave);
    verify();
}
void BikeModel::draw(double distance, double steer, bool braking, bool normals,
                     bool controls) const {
    const auto &s = bikeSpec(type);
    bool wave = type == BikeType::Wave;
    // Inclined steering axis through the front hub; fork and wheel inherit yaw.
    // Negative roll moves the tire contact point backwards in the bike's frame.
    double rake = wave ? 25. : 26.;
    Matrix4x4 steeringRotation = multipleMatrix(
        rotationZMatrix(rake), multipleMatrix(rotationYMatrix(steer), rotationZMatrix(-rake)));
    for (J j : {J::Frame, J::Fork, J::FrontWheel, J::RearWheel}) {
        transforms.push();
        if (j != J::Frame) {
            V o = origin(s, j);
            auto local = translationMatrix(o.x, o.y, o.z);
            if (j != J::RearWheel)
                local = multipleMatrix(local, steeringRotation);
            if (j != J::Fork) {
                double r = (j == J::FrontWheel ? s.front : s.rear).radius();
                local = multipleMatrix(local, rotationZMatrix(-distance / r * 180 / PI));
            }
            transforms.apply(local);
        }
        for (auto it = batches.lower_bound({j, F::Paint});
             it != batches.end() && it->first.first == j; ++it) {
            auto f = it->first.second;
            double shine = (f == F::Rubber || f == F::Seat) ? 9. : (f == F::Graphite ? 22. : 78.);
            material(pigment(f, wave, braking), shine);
            float highlight = (f == F::Rubber     ? .045f
                               : f == F::Seat     ? .085f
                               : f == F::Graphite ? .15f
                                                  : .49f);
            const GLfloat specular[] = {highlight, highlight, highlight, 1};
            glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular);
            it->second.draw(normals);
        }
        transforms.pop();
    }
    if (controls) {
        glDisable(GL_LIGHTING);
        glColor3f(1, .58, .05);
        glLineWidth(2);
        glPointSize(6);
        for (auto mode : {GL_POINTS, GL_LINE_STRIP}) {
            glBegin(mode);
            for (V p : seatControls)
                glVertex3d(p.x, p.y + .006, p.z);
            glEnd();
        }
        glEnable(GL_LIGHTING);
    }
}
void BikeModel::verify() const {
    const auto &s = bikeSpec(type);
    V low{10, 10, 10}, high{-10, -10, -10};
    size_t triangles = 0;
    for (const auto &batch : batches) {
        V o = origin(s, batch.first.first);
        for (const auto &vertex : batch.second.vertices) {
            V p = vertex.p + o;
            if (!std::isfinite(dot(p, p)) || fabs(length(vertex.n) - 1) > .00001)
                throw std::runtime_error("Invalid bike mesh");
            low = {std::min(low.x, p.x), std::min(low.y, p.y), std::min(low.z, p.z)};
            high = {std::max(high.x, p.x), std::max(high.y, p.y), std::max(high.z, p.z)};
        }
        triangles += batch.second.vertices.size() / 3;
    }
    auto check = [&](bool valid, const char *name) {
        if (!valid)
            throw std::runtime_error(std::string(s.name) + " geometry: " + name);
    };
    double contactCenter[2]{};
    for (J j : {J::FrontWheel, J::RearWheel}) {
        const auto &vertices = batches.at({j, F::Rubber}).vertices;
        V o = origin(s, j);
        double ground = 10, radius = 0, minX = 10, maxX = -10;
        for (const auto &v : vertices) {
            ground = std::min(ground, v.p.y + o.y);
            minX = std::min(minX, v.p.x + o.x);
            maxX = std::max(maxX, v.p.x + o.x);
            radius = std::max(radius, hypot(v.p.x, v.p.y));
        }
        contactCenter[j == J::FrontWheel ? 1 : 0] = (maxX + minX) / 2;
        check(fabs(ground) < 1e-7 && fabs(radius - o.y) < 1e-7,
              "nominal tire radius / ground contact");
    }
    check(fabs(high.x - low.x - s.length) < .002, "overall length");
    check(fabs(high.z - low.z - s.width) < .002, "overall width");
    check(fabs(contactCenter[1] - contactCenter[0] - s.wheelbase) < 1e-8, "wheelbase");
    double seatX = type == BikeType::Wave ? -.215 : -.26, riderSeat = -1;
    for (const auto &v : batches.at({J::Frame, F::Seat}).vertices)
        if (fabs(v.p.x - seatX) < 1e-7)
            riderSeat = std::max(riderSeat, v.p.y);
    check(fabs(riderSeat - s.seatHeight) < 1e-6, "seat surface at rider station");
    std::cout << s.name << (distant ? " [traffic]" : " [studio]") << ": mesh (mm, mirrors fitted) "
              << std::lround(1000 * (high.x - low.x)) << " x "
              << std::lround(1000 * (high.z - low.z)) << " x "
              << std::lround(1000 * (high.y - low.y)) << ", rider seat "
              << std::lround(1000 * riderSeat) << ", wheelbase " << std::lround(1000 * s.wheelbase)
              << ", " << triangles << " triangles\n";
}
