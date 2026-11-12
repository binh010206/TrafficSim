// Compatibility OpenGL geometry with GLSL 1.20 per-fragment lighting; fixed-function Gouraud
// comparison.
#include "Renderer.h"
#include "Font8x8.h"
#include "Shader.h"
#include "Transform.h"
#include <iomanip>
#include <sstream>
static void text(double x, double y, std::string s, double scale = 1.5) {
    glBegin(GL_QUADS);
    for (unsigned char c : s) {
        if (c < 128)
            for (int r = 0; r < 8; r++)
                for (int b = 0; b < 8; b++)
                    if (font8x8_basic[c][r] & (1 << b)) {
                        double X = x + b * scale, Y = y + r * scale;
                        glVertex2d(X, Y);
                        glVertex2d(X + scale, Y);
                        glVertex2d(X + scale, Y + scale);
                        glVertex2d(X, Y + scale);
                    }
        x += 9 * scale;
    }
    glEnd();
}
void Renderer::init() {
    models[0].build(BikeType::Wave);
    models[1].build(BikeType::Vision);
    distantModels[0].build(BikeType::Wave, true);
    distantModels[1].build(BikeType::Vision, true);
    const char *vs = "#version 120\nvarying vec3 N,P;void main(){vec4 "
                     "p=gl_ModelViewMatrix*gl_Vertex;P=p.xyz;N=gl_NormalMatrix*gl_Normal;gl_"
                     "FrontColor=gl_Color;gl_Position=gl_ProjectionMatrix*p;}";
    const char *fs =
        "#version 120\nvarying vec3 N,P;void main(){vec3 "
        "n=normalize(N),l=normalize(gl_LightSource[0].position.xyz),v=normalize(-P);float "
        "d=max(dot(n,l),0.);float "
        "s=d>0.?pow(max(dot(n,normalize(l+v)),0.),max(1.,gl_FrontMaterial.shininess)):0.;gl_"
        "FragColor=vec4(gl_Color.rgb*(.28+.72*d)+gl_FrontMaterial.specular.rgb*s,1.);}";
    GLuint a = glCreateShader(GL_VERTEX_SHADER), b = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(a, 1, &vs, nullptr);
    glCompileShader(a);
    glShaderSource(b, 1, &fs, nullptr);
    glCompileShader(b);
    program = glCreateProgram();
    glAttachShader(program, a);
    glAttachShader(program, b);
    glLinkProgram(program);
    GLint ok = 0;
    glGetProgramiv(program, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[2048];
        glGetProgramInfoLog(program, 2048, nullptr, log);
        throw std::runtime_error(log);
    }
    glDeleteShader(a);
    glDeleteShader(b);
}
static void block(V p, V size, V color) {
    material(color);
    transforms.push();
    transforms.apply(translationMatrix(p.x, p.y, p.z));
    box(size.x, size.y, size.z);
    transforms.pop();
}
static void dimensionGuides(BikeType type) {
    const auto &s = bikeSpec(type);
    // Anchors use the SAME metre-valued specification as the physical model.
    double front = s.wheelbase * .5, back = -front;
    double maxX = front + s.front.radius(), minX = maxX - s.length;
    auto line = [](V a, V b) {
        glVertex3d(a.x, a.y, a.z);
        glVertex3d(b.x, b.y, b.z);
    };
    auto dim = [&](V a, V b, V tick) {
        line(a, b);
        line(a - tick, a + tick);
        line(b - tick, b + tick);
    };
    glLineWidth(2);
    glBegin(GL_LINES);
    glColor3d(.96, .67, .12);
    dim({back, .075, .43}, {front, .075, .43}, {0, .025, 0});
    line({back, s.rear.radius(), 0}, {back, .075, .43});
    line({front, s.front.radius(), 0}, {front, .075, .43});
    glColor3d(.20, .83, .92);
    dim({minX, .025, .61}, {maxX, .025, .61}, {0, .025, 0});
    line({minX, .025, 0}, {minX, .025, .63});
    line({maxX, .025, 0}, {maxX, .025, .63});
    glColor3d(.91, .43, .34);
    double seatX = type == BikeType::Wave ? -.215 : -.26;
    dim({seatX, 0, -.46}, {seatX, s.seatHeight, -.46}, {.025, 0, 0});
    line({seatX, s.seatHeight, 0}, {seatX, s.seatHeight, -.48});
    glEnd();
    glLineWidth(1);
}
void Renderer::draw(const Simulation &sim, View &v, int w, int h) {
    glViewport(0, 0, w, h);
    glClearColor(.63, .73, .79, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_NORMALIZE);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glShadeModel(GL_SMOOTH);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    size_t selectedIndex = 0;
    for (size_t i = 0; i < sim.bikes.size(); i++)
        if (sim.bikes[i].id == v.selected)
            selectedIndex = i;
    if (!sim.bikes.empty())
        v.selected = sim.bikes[selectedIndex].id;
    V target{0, 0, 0};
    if (v.mode == 2)
        target = {-.045, .615, 0};
    if (v.mode == 3 && !sim.bikes.empty())
        target = sim.routes[sim.bikes[selectedIndex].route].position(sim.bikes[selectedIndex].s) +
                 V{0, .6, 0};
    double dist = v.mode == 2 ? v.showroomDist : v.mode == 3 ? 7. : v.distance;
    if (v.orthographic) {
        // Match the perspective scale in the target plane when changing projection.
        double halfHeight = dist * tan(22.5 * PI / 180), halfWidth = halfHeight * w / h;
        loadProjectionMatrix(
            orthographicMatrix(-halfWidth, halfWidth, -halfHeight, halfHeight, .1, 250));
    } else {
        loadProjectionMatrix(perspectiveMatrix(45., double(w) / h, .1, 250));
    }
    double pitch = (v.mode == 1   ? 89.5
                    : v.mode == 2 ? v.showroomPitch
                                  : v.pitch) *
                   PI / 180,
           yaw = (v.mode == 2 ? v.showroomYaw : v.yaw) * PI / 180;
    V eye =
        target + V{dist * cos(pitch) * sin(yaw), dist * sin(pitch), dist * cos(pitch) * cos(yaw)};
    transforms.setView(lookAtMatrix(eye, target));
    GLfloat light[] = {.65f, 1.f, .80f, 0}, amb[] = {.28f, .28f, .28f, 1};
    glLightfv(GL_LIGHT0, GL_POSITION, light);
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, amb);
    glUseProgram(v.phong ? program : 0);
    glPolygonMode(GL_FRONT_AND_BACK, v.wire ? GL_LINE : GL_FILL);
    if (v.mode == 2) {
        block({0, -.06, 0}, {70, .058, 70}, {.62, .68, .70});
        // Soft contact patches on the studio floor, anchored to the two wheel centers.
        glUseProgram(0);
        glDisable(GL_LIGHTING);
        glDisable(GL_CULL_FACE);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDepthMask(GL_FALSE);
        auto shadow = [](double x, double rx, double rz, double alpha) {
            glBegin(GL_TRIANGLE_FAN);
            glColor4d(.07, .10, .13, alpha);
            glVertex3d(x, .0002, 0);
            for (int k = 0; k <= 48; k++) {
                double a = 2 * PI * k / 48;
                glColor4d(.07, .10, .13, 0);
                glVertex3d(x + rx * cos(a), .0002, rz * sin(a));
            }
            glEnd();
        };
        shadow(-.035, 1.10, .36, .32);
        for (int side : {-1, 1})
            shadow(side * bikeSpec(v.showroom).wheelbase * .5, .19, .10, .68);
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
        glEnable(GL_CULL_FACE);
        glEnable(GL_LIGHTING);
        glUseProgram(v.phong ? program : 0);
        models[int(v.showroom)].draw(0., 0., false, v.normals, v.controls);
        if (v.dimensions) {
            glUseProgram(0);
            glDisable(GL_LIGHTING);
            dimensionGuides(v.showroom);
        }
    } else {
        block({0, -.16, 0}, {110, .1, 110}, {.28, .40, .31});
        block({0, -.06, 0}, {16, .06, 96}, {.14, .16, .18});
        block({0, -.059, 0}, {96, .06, 16}, {.14, .16, .18});
        for (int side = 0; side < 4; side++) {
            transforms.push();
            transforms.apply(rotationYMatrix(-side * 90));
            block({11, 0, 27}, {6, .15, 38}, {.59, .60, .56});
            for (int i = 0; i < 6; i++) {
                block({-6.5 + i * 2.6, .008, 10.8}, {1.3, .015, 2.4}, {.85, .86, .79});
            }
            block({3.8, .01, 13}, {7.4, .02, .26}, {.94, .94, .84});
            for (int z = 15; z < 45; z += 5) {
                block({0, .005, double(z)}, {.12, .015, 2.5}, {.91, .73, .25});
                block({4.5, .005, double(z)}, {.09, .015, 2.5}, {.70, .72, .68});
            }
            material({.16, .19, .21});
            rod({8.7, 0, 13}, {8.7, 5.8, 13}, .085);
            rod({8.7, 5.8, 13}, {2., 5.8, 13}, .065);
            block({2, 4.05, 13}, {.70, 1.65, .38}, {.035, .045, .055});
            int lit = static_cast<int>(sim.signal(side));
            glUseProgram(0);
            glDisable(GL_LIGHTING);
            for (int k = 0; k < 3; k++) {
                transforms.push();
                transforms.apply(translationMatrix(2., 5.40 - k * .51, 13.22));
                material(k == lit ? (k == 0   ? V{1, .08, .03}
                                     : k == 1 ? V{1, .65, .03}
                                              : V{.05, 1, .3})
                                  : V{.12, .14, .13});
                static Mesh lens = ellipsoid(.23, .23, .065);
                lens.draw();
                transforms.pop();
            }
            glEnable(GL_LIGHTING);
            glUseProgram(v.phong ? program : 0);
            transforms.pop();
        }
        // Architectural context kept secondary to the intersection.
        for (int x : {-1, 1})
            for (int z : {-1, 1})
                for (int i = 0; i < 3; i++) {
                    double height = 4 + (i % 3) * 2;
                    V p{double(x * (21 + i * 9)), 0, double(z * 24)};
                    block(p, {7, height, 10}, {.58 + i * .045, .61 + i * .025, .60});
                    for (int k = 1; k < height; k += 2)
                        block({p.x, k * 1., p.z + z * 5.01}, {5, .7, .02}, {.24, .39, .44});
                }
        if (v.paths) {
            glUseProgram(0);
            glDisable(GL_LIGHTING);
            glLineWidth(2);
            for (auto &r : sim.routes) {
                glColor3f(r.turn == 0 ? .3 : 1, r.turn == 1 ? 1 : .5, .8);
                glBegin(GL_LINE_STRIP);
                for (V p : r.points)
                    glVertex3d(p.x, .045, p.z);
                glEnd();
            }
            glColor3f(1, .3, .1);
            glBegin(GL_LINE_LOOP);
            for (V p : {V{-8, .05, -8}, V{8, .05, -8}, V{8, .05, 8}, V{-8, .05, 8}})
                glVertex3d(p.x, p.y, p.z);
            glEnd();
            glEnable(GL_LIGHTING);
            glUseProgram(v.phong ? program : 0);
        }
        for (auto &b : sim.bikes) {
            auto &r = sim.routes[b.route];
            V p = r.position(b.s), t = r.tangent(b.s), t2 = r.tangent(b.s + .3);
            const auto type = typeForBike(b.id);
            const double measuredArc = std::min(.3, r.total - b.s);
            double curvature =
                measuredArc > 1e-6 ? -atan2(t.x * t2.z - t.z * t2.x, dot(t, t2)) / measuredArc : 0.;
            // Kinematic estimate for the visual steering angle; curvature is in 1/m.
            double steer =
                std::clamp(atan(bikeSpec(type).wheelbase * curvature) * 180 / PI, -25., 25.);
            transforms.push();
            // M_bike = T(position) * Ry(heading); local X points along the route.
            Matrix4x4 bikeModel = multipleMatrix(translationMatrix(p.x, .002, p.z),
                                                 rotationYMatrix(-atan2(t.z, t.x) * 180 / PI));
            transforms.apply(bikeModel);
            auto &fleet = length(eye - p) < 12.0 ? models : distantModels;
            fleet[int(type)].draw(b.s, steer, b.a < -.4 || b.v < .1, v.normals, false);
            transforms.pop();
        }
    }
    glUseProgram(0);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glDisable(GL_CULL_FACE);
    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);
    loadProjectionMatrix(orthographicMatrix(0, w, h, 0, -1, 1));
    transforms.setView(identityMatrix());
    if (v.cleanPreview)
        return;
    glColor3f(.035, .065, .09);
    glBegin(GL_QUADS);
    glVertex2d(0, 0);
    glVertex2d(w, 0);
    glVertex2d(w, v.mode == 2 ? 108 : 132);
    glVertex2d(0, v.mode == 2 ? 108 : 132);
    glVertex2d(15, h - 87);
    glVertex2d(w - 15, h - 87);
    glVertex2d(w - 15, h - 15);
    glVertex2d(15, h - 15);
    glEnd();
    glColor3f(.4, .86, .82);
    const bool studio = v.mode == 2;
    text(24, 16, studio ? bikeSpec(v.showroom).name : "MOTO / INTERSECTION LAB", 2);
    glColor3f(.88, .92, .93);
    std::ostringstream s;
    if (studio) {
        const auto &m = bikeSpec(v.showroom);
        s << "WHEELBASE " << std::lround(m.wheelbase * 1000) << " mm   SEAT "
          << std::lround(m.seatHeight * 1000) << " mm   RIMS " << m.front.rimInches << " / "
          << m.rear.rimInches << " in";
        text(24, 48, s.str());
        text(24, 75,
             std::string("M MODEL   O PROJECTION: ") +
                 (v.orthographic ? "ORTHOGRAPHIC" : "PERSPECTIVE") +
                 "   G DIMENSIONS   C SEAT STATIONS",
             1.25);
        s.str("");
        s << "LENGTH (CYAN) " << std::lround(m.length * 1000)
          << " mm  /  WHEELBASE (GOLD)  /  SEAT HEIGHT (RED)";
        if (v.dimensions)
            text(24, 93, s.str(), 1.1);
    } else {
        s << std::fixed << std::setprecision(1) << "TIME " << sim.time << "s   ACTIVE "
          << sim.bikes.size() << "   PASSED " << sim.passed << "   INFLOW " << sim.density
          << "/s   " << (v.paused ? "PAUSED" : "RUNNING");
        text(24, 70, s.str());
        s.str("");
        s << sim.phaseName() << "  " << std::setprecision(1) << sim.remaining() << "s   RESERVED "
          << sim.activeReservations() << "   QUEUES S/W/N/E " << sim.queueLength(0) << "/"
          << sim.queueLength(1) << "/" << sim.queueLength(2) << "/" << sim.queueLength(3);
        text(24, 90, s.str(), 1.3);
        for (int axis = 0; axis < 2; ++axis) {
            auto color = sim.signal(axis);
            V rgb = color == SignalColor::Green    ? V{.20, 1., .48}
                    : color == SignalColor::Yellow ? V{1., .76, .16}
                                                   : V{1., .30, .25};
            glColor3d(rgb.x, rgb.y, rgb.z);
            std::string name = color == SignalColor::Green    ? "XANH"
                               : color == SignalColor::Yellow ? "VANG"
                                                              : "DO";
            text(24 + axis * 350, 44, (axis ? "DONG-TAY: " : "BAC-NAM: ") + name, 1.8);
        }
        glColor3f(.88, .92, .93);
        if (!sim.bikes.empty()) {
            auto &b = sim.bikes[selectedIndex];
            s.str("");
            s << bikeSpec(typeForBike(b.id)).name << " #" << b.id << "   v=" << b.v
              << " m/s   a=" << b.a << "   wait=" << b.wait << "s   " << b.reason;
            text(24, 111, s.str(), 1.25);
        }
    }
    text(28, h - 73, "1 OVERVIEW  2 TOP  3 MODELS   4 FOLLOW | SPACE PAUSE  . STEP  R RESET", 1.4);
    text(28, h - 51, "W MESH  N NORMALS  C CONTROLS  P PATHS  H SHADING  TAB BIKE  +/- INFLOW",
         1.4);
    text(28, h - 29,
         "ARROWS ORBIT   SCROLL ZOOM   ESC EXIT | FIXED 60 HZ | QUEUES / SIGNALS / ROUTE CONFLICTS",
         1.25);
}
