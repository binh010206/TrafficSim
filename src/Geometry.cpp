#include "Geometry.h"
#include <functional>
static Mesh surface(std::function<V(double, double)> f, int nu = 40, int nv = 24,
                    bool reverse = true) {
    Mesh m;
    auto v = [&](double u, double t) {
        V p = f(u, t);
        double e = 1e-5;
        V n = unit(cross(f(u, t + e) - f(u, t - e), f(u + e, t) - f(u - e, t))) *
              (reverse ? 1. : -1.);
        return Vertex{p, n};
    };
    for (int i = 0; i < nu; i++)
        for (int j = 0; j < nv; j++) {
            double u = double(i) / nu, U = double(i + 1) / nu, t = double(j) / nv,
                   T = double(j + 1) / nv;
            std::vector<Vertex> quad = {v(u, t), v(U, t), v(U, T), v(u, t), v(U, T), v(u, T)};
            if (reverse) {
                std::swap(quad[1], quad[2]);
                std::swap(quad[4], quad[5]);
            }
            for (auto p : quad)
                m.vertices.push_back(p);
        }
    return m;
}
Mesh ellipsoid(double x, double y, double z) {
    return surface(
        [=](double u, double v) {
            double a = u * PI, b = v * 2 * PI;
            return V{x * sin(a) * cos(b), y * cos(a), z * sin(a) * sin(b)};
        },
        24, 24);
}
void Mesh::draw(bool normals) const {
    GLint compiling = 0;
    glGetIntegerv(GL_LIST_INDEX, &compiling);
    if (displayList) {
        glCallList(displayList);
    } else {
        if (!compiling) {
            displayList = glGenLists(1);
            glNewList(displayList, GL_COMPILE);
        }
        glBegin(GL_TRIANGLES);
        for (auto &v : vertices) {
            glNormal3d(v.n.x, v.n.y, v.n.z);
            glVertex3d(v.p.x, v.p.y, v.p.z);
        }
        glEnd();
        if (!compiling) {
            glEndList();
            glCallList(displayList);
        }
    }
    if (normals) {
        glDisable(GL_LIGHTING);
        glColor3f(.2, 1, .8);
        glBegin(GL_LINES);
        for (size_t i = 0; i < vertices.size(); i += 24) {
            auto v = vertices[i];
            glVertex3d(v.p.x, v.p.y, v.p.z);
            V q = v.p + v.n * .1;
            glVertex3d(q.x, q.y, q.z);
        }
        glEnd();
        glEnable(GL_LIGHTING);
    }
}
void material(V c, double shine) {
    GLfloat diffuse[] = {(float)c.x, (float)c.y, (float)c.z, 1}, spec[] = {.45, .45, .45, 1};
    glColor3f(c.x, c.y, c.z);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, diffuse);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shine);
}
void box(double x, double y, double z) {
    V p[8] = {{-x / 2, 0, -z / 2}, {x / 2, 0, -z / 2}, {x / 2, y, -z / 2}, {-x / 2, y, -z / 2},
              {-x / 2, 0, z / 2},  {x / 2, 0, z / 2},  {x / 2, y, z / 2},  {-x / 2, y, z / 2}};
    int faces[6][4] = {{0, 3, 2, 1}, {4, 5, 6, 7}, {0, 4, 7, 3},
                       {1, 2, 6, 5}, {3, 7, 6, 2}, {0, 1, 5, 4}};
    glBegin(GL_QUADS);
    for (auto &f : faces) {
        V n = unit(cross(p[f[1]] - p[f[0]], p[f[2]] - p[f[0]]));
        glNormal3d(n.x, n.y, n.z);
        for (int i : f)
            glVertex3d(p[i].x, p[i].y, p[i].z);
    }
    glEnd();
}
void rod(V a, V b, double radius) {
    V axis = unit(b - a), side = unit(cross(axis, fabs(axis.y) < .9 ? V{0, 1, 0} : V{1, 0, 0})),
      up = cross(axis, side);
    glBegin(GL_QUAD_STRIP);
    for (int i = 0; i <= 12; i++) {
        double t = i * 2 * PI / 12;
        V n = side * cos(t) + up * sin(t);
        glNormal3d(n.x, n.y, n.z);
        V p = a + n * radius, q = b + n * radius;
        glVertex3d(q.x, q.y, q.z);
        glVertex3d(p.x, p.y, p.z);
    }
    glEnd();
}
