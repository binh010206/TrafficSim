// Application input, fixed 60 Hz simulation, rendering and deterministic regression checks.
#include "MatrixTests.h"
#include "Renderer.h"
#include "Shader.h"
#include "SimulationTests.h"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <unordered_map>
static View view;
static Simulation sim;
static void saveFrame(GLFWwindow *window, const std::filesystem::path &path) {
    int width, height;
    glfwGetFramebufferSize(window, &width, &height);
    std::vector<unsigned char> pixels(width * height * 3);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());
    if (glGetError() != GL_NO_ERROR)
        throw std::runtime_error("Failed to capture framebuffer");
    std::ofstream out(path, std::ios::binary);
    out.exceptions(std::ios::badbit | std::ios::failbit);
    out << "P6\n" << width << " " << height << "\n255\n";
    for (int y = height - 1; y >= 0; --y)
        out.write(reinterpret_cast<char *>(pixels.data() + 3 * y * width), 3 * width);
}
static void key(GLFWwindow *w, int k, int, int action, int) {
    if (action != GLFW_PRESS)
        return;
    switch (k) {
    case GLFW_KEY_ESCAPE:
        glfwSetWindowShouldClose(w, 1);
        break;
    case GLFW_KEY_SPACE:
        view.paused = !view.paused;
        break;
    case GLFW_KEY_PERIOD:
        if (view.paused)
            sim.step(1. / 60);
        break;
    case GLFW_KEY_R:
        sim.reset();
        break;
    case GLFW_KEY_M:
        if (view.mode == 2)
            view.showroom = view.showroom == BikeType::Wave ? BikeType::Vision : BikeType::Wave;
        break;
    case GLFW_KEY_W:
        view.wire = !view.wire;
        break;
    case GLFW_KEY_N:
        view.normals = !view.normals;
        break;
    case GLFW_KEY_C:
        view.controls = !view.controls;
        break;
    case GLFW_KEY_P:
        view.paths = !view.paths;
        break;
    case GLFW_KEY_H:
        view.phong = !view.phong;
        break;
    case GLFW_KEY_O:
        view.orthographic = !view.orthographic;
        break;
    case GLFW_KEY_G:
        view.dimensions = !view.dimensions;
        break;
    case GLFW_KEY_TAB: {
        std::vector<int> ids;
        for (auto &b : sim.bikes)
            ids.push_back(b.id);
        std::sort(ids.begin(), ids.end());
        auto it = std::upper_bound(ids.begin(), ids.end(), view.selected);
        if (!ids.empty())
            view.selected = it == ids.end() ? ids.front() : *it;
        break;
    }
    case GLFW_KEY_EQUAL:
    case GLFW_KEY_KP_ADD:
        sim.density = std::min(2., sim.density + .1);
        break;
    case GLFW_KEY_MINUS:
        sim.density = std::max(.1, sim.density - .1);
        break;
    default:
        if (k >= GLFW_KEY_1 && k <= GLFW_KEY_4)
            view.mode = k - GLFW_KEY_1;
    }
}
int main(int argc, char **argv) {
    try {
        if (argc > 1 && std::string(argv[1]) == "--model-test") {
            for (auto type : {BikeType::Wave, BikeType::Vision}) {
                BikeModel model;
                model.build(type);
                BikeModel distant;
                distant.build(type, true);
            }
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "--matrix-test") {
            runMatrixTests();
            return 0;
        }
        bool test = argc > 1 && std::string(argv[1]) == "--self-test";
        if (test) {
            runMatrixTests();
            runSimulationTests();
            return 0;
        }
        bool preview = argc > 1 && std::string(argv[1]) == "--model-preview";
        bool trafficPreview = argc > 1 && std::string(argv[1]) == "--traffic-preview";
        bool trafficDemo = argc > 1 && std::string(argv[1]) == "--traffic-demo";
        if (argc > 1 && std::string(argv[1]) == "--showroom")
            view.mode = 2;
        if (!glfwInit()) {
            throw std::runtime_error("GLFW init failed");
        }
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
        glfwWindowHint(GLFW_SAMPLES, 4);
        if (preview || trafficPreview)
            glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
        GLFWwindow *w =
            glfwCreateWindow(1440, 900, "Moto Intersection | C++ / OpenGL", nullptr, nullptr);
        if (!w)
            throw std::runtime_error("OpenGL window failed");
        glfwMakeContextCurrent(w);
        loadShaders();
        glfwSwapInterval(1);
        glfwSetKeyCallback(w, key);
        glfwSetScrollCallback(w, [](GLFWwindow *, double, double y) {
            if (view.mode == 2)
                view.showroomDist = std::clamp(view.showroomDist - y * .14, 1.65, 5.);
            else
                view.distance = std::clamp(view.distance - y * 3, 15., 120.);
        });
        Renderer renderer;
        renderer.init();
        std::cout << "OpenGL " << glGetString(GL_VERSION) << "\n";
        if (preview) {
            int width, height;
            glfwGetFramebufferSize(w, &width, &height);
            std::vector<unsigned char> pixels(width * height * 3);
            glPixelStorei(GL_PACK_ALIGNMENT, 1);
            view.mode = 2;
            view.cleanPreview = true;
            view.showroomDist = 2.65;
            struct ModelAngle {
                const char *name;
                double yaw, pitch;
            };
            for (BikeType type : {BikeType::Wave, BikeType::Vision})
                for (auto angle : {ModelAngle{"side", 0, 3},
                                   {"front", 32, 13},
                                   {"rear", -144, 17},
                                   {"head", 90, 9},
                                   {"dimensions", 24, 18}}) {
                    view.showroom = type;
                    view.showroomYaw = angle.yaw;
                    view.showroomPitch = angle.pitch;
                    view.dimensions = std::string(angle.name) == "dimensions";
                    view.orthographic = view.dimensions;
                    view.cleanPreview = !view.dimensions;
                    renderer.draw(sim, view, width, height);
                    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());
                    if (glGetError() != GL_NO_ERROR)
                        throw std::runtime_error("Failed to render model preview");
                    auto name = std::string(type == BikeType::Wave ? "wave-" : "vision-") +
                                angle.name + ".ppm";
                    std::ofstream out(std::filesystem::path(argv[0]).parent_path() / name,
                                      std::ios::binary);
                    out.exceptions(std::ios::badbit | std::ios::failbit);
                    out << "P6\n" << width << " " << height << "\n255\n";
                    for (int y = height - 1; y >= 0; --y)
                        out.write(reinterpret_cast<char *>(pixels.data() + 3 * y * width),
                                  3 * width);
                }
            glfwDestroyWindow(w);
            glfwTerminate();
            return 0;
        }
        if (trafficPreview || trafficDemo) {
            for (int i = 0; i < 60 * 90; ++i)
                sim.step(1. / 60);
            view.yaw = 35;
            view.pitch = 58;
            view.distance = 49;
        }
        if (trafficPreview) {
            int width, height;
            glfwGetFramebufferSize(w, &width, &height);
            auto output = std::filesystem::path(argv[0]).parent_path();
            for (int phase = 0; phase < 6; ++phase) {
                int steps = 0;
                while (sim.phase() != phase || sim.remaining() > (phase % 3 == 0 ? 7. : 1.5)) {
                    sim.step(1. / 60);
                    if (++steps > 60 * 120)
                        throw std::runtime_error("Traffic preview stalled before target phase");
                }
                renderer.draw(sim, view, width, height);
                saveFrame(w, output / ("traffic-" + std::to_string(phase) + ".ppm"));
            }
            // A short genuine simulation sequence spanning green, yellow and clearance.
            while (sim.phase() != 0 || sim.remaining() > 4.)
                sim.step(1. / 60);
            for (int frame = 0; frame < 48; ++frame) {
                for (int i = 0; i < 30; ++i)
                    sim.step(1. / 60);
                renderer.draw(sim, view, width, height);
                saveFrame(w, output / ("flow-" + std::to_string(frame) + ".ppm"));
            }
            glfwDestroyWindow(w);
            glfwTerminate();
            return 0;
        }
        double last = glfwGetTime(), acc = 0;
        int frames = 0;
        double renderStart = last;
        bool smoke = argc > 1 && std::string(argv[1]) == "--smoke";
        if (smoke) {
            for (int i = 0; i < 3600; i++)
                sim.step(1. / 60);
        }
        while (!glfwWindowShouldClose(w)) {
            double now = glfwGetTime(), dt = std::min(.1, now - last);
            last = now;
            glfwPollEvents();
            double &yaw = view.mode == 2 ? view.showroomYaw : view.yaw;
            double &pitch = view.mode == 2 ? view.showroomPitch : view.pitch;
            if (glfwGetKey(w, GLFW_KEY_LEFT))
                yaw -= dt * 50;
            if (glfwGetKey(w, GLFW_KEY_RIGHT))
                yaw += dt * 50;
            if (glfwGetKey(w, GLFW_KEY_UP))
                pitch = std::min(85., pitch + dt * 40);
            if (glfwGetKey(w, GLFW_KEY_DOWN))
                pitch = std::max(1., pitch - dt * 40);
            if (!view.paused)
                acc += dt;
            else
                acc = 0;
            while (acc >= 1. / 60) {
                sim.step(1. / 60);
                acc -= 1. / 60;
            }
            int width, height;
            glfwGetFramebufferSize(w, &width, &height);
            if (width > 0 && height > 0) {
                if (smoke) {
                    int stage = int(now - renderStart);
                    view.mode = (stage / 2) % 4;
                    view.normals = stage == 5;
                    view.controls = stage == 4;
                    view.showroom = stage == 5 ? BikeType::Vision : BikeType::Wave;
                    view.orthographic = stage == 2 || stage == 5;
                    view.dimensions = stage == 4 || stage == 5;
                    view.paths = stage == 2;
                    view.wire = stage == 6;
                    view.phong = stage != 7;
                }
                renderer.draw(sim, view, width, height);
                frames++;
                GLenum e = glGetError();
                if (e != GL_NO_ERROR)
                    throw std::runtime_error("OpenGL error " + std::to_string(e));
                glfwSwapBuffers(w);
            }
            if (smoke && now - renderStart > 8)
                glfwSetWindowShouldClose(w, 1);
        }
        if (smoke)
            std::cout << "SMOKE PASS: all cameras / perspective + orthographic / dimensions / "
                         "wireframe / normals / paths / control curve / shading; average FPS="
                      << frames / (glfwGetTime() - renderStart) << "\n";
        glfwDestroyWindow(w);
        glfwTerminate();
        return 0;
    } catch (const std::exception &e) {
        std::cerr << "ERROR: " << e.what() << "\n";
        return 1;
    }
}
