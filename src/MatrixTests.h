#pragma once
#include "Matrix.h"
#include <iostream>

inline void runMatrixTests() {
    auto point = [](V actual, V expected, const char *label) {
        if (length(actual - expected) > 1e-9)
            throw std::runtime_error(std::string("Matrix test failed: ") + label);
    };
    auto equal = [](const Matrix4x4 &a, const Matrix4x4 &b, const char *label) {
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c)
                if (fabs(a.m[r][c] - b.m[r][c]) > 1e-9)
                    throw std::runtime_error(std::string("Matrix test failed: ") + label);
    };
    V p{1, 2, 3};
    auto t = translationMatrix(4, -2, 7);
    auto s = scalingMatrix(2, 3, 4);
    auto r = rotationYMatrix(90);
    point(transformPoint(identityMatrix(), p), p, "identity");
    point(transformPoint(t, p), {5, 0, 10}, "translation");
    point(transformPoint(s, p), {2, 6, 12}, "nonuniform scale");
    point(transformPoint(rotationXMatrix(90), {0, 1, 0}), {0, 0, 1}, "rotation X");
    point(transformPoint(r, {1, 0, 0}), {0, 0, -1}, "rotation Y");
    point(transformPoint(rotationZMatrix(90), {1, 0, 0}), {0, 1, 0}, "rotation Z");
    point(transformPoint(multipleMatrix(t, s), p), {6, 4, 19}, "T*S order");
    point(transformPoint(multipleMatrix(s, t), p), {10, 0, 40}, "S*T order");
    equal(multipleMatrix(multipleMatrix(t, r), s), multipleMatrix(t, multipleMatrix(r, s)),
          "associativity");
    equal(multipleMatrix(r, rotationYMatrix(-90)), identityMatrix(), "rotation inverse");
    V pivot{4, 2, -3};
    point(transformPoint(rotationAroundPointMatrix(pivot, r), pivot), pivot, "fixed pivot");
    point(transformPoint(rotationAroundPointMatrix(pivot, r), pivot + V{1, 0, 0}),
          pivot + V{0, 0, -1}, "rotate around pivot");

    // Real hierarchy: bike placement -> wheel attachment -> wheel spin.
    auto bike = multipleMatrix(translationMatrix(10, 0, 20), r);
    auto wheel =
        multipleMatrix(bike, multipleMatrix(translationMatrix(.73, .32, 0), rotationZMatrix(90)));
    point(transformPoint(wheel, {0, 0, 0}), {10, .32, 19.27}, "wheel center follows bike");
    point(transformPoint(wheel, {.32, 0, 0}), {10, .64, 19.27}, "wheel rotates locally");

    auto view = lookAtMatrix({0, 0, 5}, {0, 0, 0});
    point(transformPoint(view, {0, 0, 5}), {0, 0, 0}, "eye maps to origin");
    point(transformPoint(view, {0, 0, 0}), {0, 0, -5}, "target in front of camera");
    auto projection = perspectiveMatrix(60, 1.6, .1, 250);
    point(transformPoint(projection, {0, 0, -.1}), {0, 0, -1}, "perspective near plane");
    point(transformPoint(projection, {0, 0, -250}), {0, 0, 1}, "perspective far plane");
    double nearX = transformPoint(projection, {1, 0, -2}).x;
    double farX = transformPoint(projection, {1, 0, -4}).x;
    if (fabs(nearX - 2 * farX) > 1e-9)
        throw std::runtime_error("Perspective must shrink distant objects");
    auto ortho = orthographicMatrix(0, 1440, 900, 0, -1, 1);
    point(transformPoint(ortho, {0, 0, 0}), {-1, 1, 0}, "HUD top-left");
    point(transformPoint(ortho, {1440, 900, 0}), {1, -1, 0}, "HUD bottom-right");
    auto packed = openGLMatrixData(t);
    for (int row = 0; row < 4; ++row)
        for (int col = 0; col < 4; ++col)
            if (packed[col * 4 + row] != t.m[row][col])
                throw std::runtime_error("Incorrect OpenGL memory order");
    // Preserve the previous XZ route rotation convention on all four approaches.
    for (int side = 0; side < 4; ++side) {
        double a = side * PI / 2;
        point(transformPoint(rotationYMatrix(-side * 90.), p),
              {p.x * cos(a) - p.z * sin(a), p.y, p.x * sin(a) + p.z * cos(a)},
              "route rotation regression");
    }
    std::cout
        << "MATRIX PASS: multiplication / composition order / axes / pivot / wheel hierarchy / "
           "camera / projections / OpenGL layout / route regression\n";
}
