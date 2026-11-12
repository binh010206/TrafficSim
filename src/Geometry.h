#pragma once
#include "Math.h"
#include <GLFW/glfw3.h>
#include <vector>
struct Vertex {
    V p, n;
};
struct Mesh {
    mutable GLuint displayList = 0;
    std::vector<Vertex> vertices;
    void draw(bool normals = false) const;
};
Mesh ellipsoid(double x, double y, double z);
void material(V c, double shine = 30);
void box(double x, double y, double z);
void rod(V a, V b, double radius);
