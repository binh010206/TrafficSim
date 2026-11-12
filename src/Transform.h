#pragma once
#include "Matrix.h"
#include <GLFW/glfw3.h>
#include <vector>

// CPU-side model hierarchy. OpenGL only receives completed matrices via glLoadMatrixd.
class TransformStack {
    Matrix4x4 view = identityMatrix();
    std::vector<Matrix4x4> models{identityMatrix()};

    void upload() const {
        auto data = openGLMatrixData(multipleMatrix(view, models.back()));
        glMatrixMode(GL_MODELVIEW);
        glLoadMatrixd(data.data());
    }

  public:
    void setView(const Matrix4x4 &matrix) {
        if (models.size() != 1)
            throw std::runtime_error("Unbalanced model transform stack");
        view = matrix;
        models.back() = identityMatrix();
        upload();
    }
    void push() { models.push_back(models.back()); }
    void pop() {
        if (models.size() <= 1)
            throw std::runtime_error("Model transform stack underflow");
        models.pop_back();
        upload();
    }
    void apply(const Matrix4x4 &local) {
        models.back() = multipleMatrix(models.back(), local);
        upload();
    }
};

inline TransformStack transforms;

inline void loadProjectionMatrix(const Matrix4x4 &projection) {
    auto data = openGLMatrixData(projection);
    glMatrixMode(GL_PROJECTION);
    glLoadMatrixd(data.data());
    glMatrixMode(GL_MODELVIEW);
}
