#pragma once
#include "Math.h"
#include <array>
#include <stdexcept>

// Column vectors: p' = M * p. Storage is m[row][column].
// A * B applies B first, then A. All rotation angles below are in DEGREES.
struct Matrix4x4 {
    double m[4][4]{};
};

inline Matrix4x4 identityMatrix() {
    Matrix4x4 result;
    for (int i = 0; i < 4; ++i)
        result.m[i][i] = 1;
    return result;
}

// Actual matrix multiplication, implemented on the CPU (not delegated to OpenGL).
inline Matrix4x4 multipleMatrix(const Matrix4x4 &a, const Matrix4x4 &b) {
    Matrix4x4 result;
    for (int row = 0; row < 4; ++row)
        for (int col = 0; col < 4; ++col)
            for (int k = 0; k < 4; ++k)
                result.m[row][col] += a.m[row][k] * b.m[k][col];
    return result;
}

inline Matrix4x4 translationMatrix(double tx, double ty, double tz) {
    Matrix4x4 result = identityMatrix();
    result.m[0][3] = tx;
    result.m[1][3] = ty;
    result.m[2][3] = tz;
    return result;
}

inline Matrix4x4 scalingMatrix(double sx, double sy, double sz) {
    Matrix4x4 result = identityMatrix();
    result.m[0][0] = sx;
    result.m[1][1] = sy;
    result.m[2][2] = sz;
    return result;
}

inline Matrix4x4 rotationXMatrix(double degrees) {
    double a = degrees * PI / 180, c = cos(a), s = sin(a);
    Matrix4x4 result = identityMatrix();
    result.m[1][1] = result.m[2][2] = c;
    result.m[1][2] = -s;
    result.m[2][1] = s;
    return result;
}

inline Matrix4x4 rotationYMatrix(double degrees) {
    double a = degrees * PI / 180, c = cos(a), s = sin(a);
    Matrix4x4 result = identityMatrix();
    result.m[0][0] = result.m[2][2] = c;
    result.m[0][2] = s;
    result.m[2][0] = -s;
    return result;
}

inline Matrix4x4 rotationZMatrix(double degrees) {
    double a = degrees * PI / 180, c = cos(a), s = sin(a);
    Matrix4x4 result = identityMatrix();
    result.m[0][0] = result.m[1][1] = c;
    result.m[0][1] = -s;
    result.m[1][0] = s;
    return result;
}

inline Matrix4x4 rotationAroundPointMatrix(V pivot, const Matrix4x4 &rotation) {
    // Move pivot to origin, rotate, move back: T(pivot) * R * T(-pivot).
    return multipleMatrix(
        translationMatrix(pivot.x, pivot.y, pivot.z),
        multipleMatrix(rotation, translationMatrix(-pivot.x, -pivot.y, -pivot.z)));
}

inline V transformPoint(const Matrix4x4 &matrix, V p) {
    double input[] = {p.x, p.y, p.z, 1}, output[4]{};
    for (int row = 0; row < 4; ++row)
        for (int col = 0; col < 4; ++col)
            output[row] += matrix.m[row][col] * input[col];
    if (fabs(output[3]) < 1e-12)
        throw std::runtime_error("Cannot divide homogeneous point by zero w");
    return {output[0] / output[3], output[1] / output[3], output[2] / output[3]};
}

inline Matrix4x4 lookAtMatrix(V eye, V target) {
    V f = unit(target - eye), right = unit(cross(f, {0, 1, 0})), up = cross(right, f);
    Matrix4x4 orientation = identityMatrix();
    orientation.m[0][0] = right.x;
    orientation.m[0][1] = right.y;
    orientation.m[0][2] = right.z;
    orientation.m[1][0] = up.x;
    orientation.m[1][1] = up.y;
    orientation.m[1][2] = up.z;
    orientation.m[2][0] = -f.x;
    orientation.m[2][1] = -f.y;
    orientation.m[2][2] = -f.z;
    return multipleMatrix(orientation, translationMatrix(-eye.x, -eye.y, -eye.z));
}

inline Matrix4x4 perspectiveMatrix(double fovDegrees, double aspect, double near, double far) {
    Matrix4x4 result;
    double f = 1 / tan(fovDegrees * PI / 360);
    result.m[0][0] = f / aspect;
    result.m[1][1] = f;
    result.m[2][2] = -(far + near) / (far - near);
    result.m[2][3] = -2 * far * near / (far - near);
    result.m[3][2] = -1;
    return result;
}

inline Matrix4x4 orthographicMatrix(double left, double right, double bottom, double top,
                                    double near, double far) {
    Matrix4x4 result = identityMatrix();
    result.m[0][0] = 2 / (right - left);
    result.m[1][1] = 2 / (top - bottom);
    result.m[2][2] = -2 / (far - near);
    result.m[0][3] = -(right + left) / (right - left);
    result.m[1][3] = -(top + bottom) / (top - bottom);
    result.m[2][3] = -(far + near) / (far - near);
    return result;
}

// OpenGL expects column-major memory; this only packs the result, it does not transpose M.
inline std::array<double, 16> openGLMatrixData(const Matrix4x4 &matrix) {
    std::array<double, 16> packed{};
    for (int row = 0; row < 4; ++row)
        for (int col = 0; col < 4; ++col)
            packed[col * 4 + row] = matrix.m[row][col];
    return packed;
}
