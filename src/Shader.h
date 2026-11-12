#pragma once
#include <GL/glext.h>
#include <GLFW/glfw3.h>
#include <stdexcept>
#define SHADER_FUNCTIONS(X)                                                                        \
    X(PFNGLCREATESHADERPROC, glCreateShader)                                                       \
    X(PFNGLSHADERSOURCEPROC, glShaderSource)                                                       \
    X(PFNGLCOMPILESHADERPROC, glCompileShader)                                                     \
    X(PFNGLCREATEPROGRAMPROC, glCreateProgram)                                                     \
    X(PFNGLATTACHSHADERPROC, glAttachShader)                                                       \
    X(PFNGLLINKPROGRAMPROC, glLinkProgram)                                                         \
    X(PFNGLGETPROGRAMIVPROC, glGetProgramiv)                                                       \
    X(PFNGLGETPROGRAMINFOLOGPROC, glGetProgramInfoLog)                                             \
    X(PFNGLDELETESHADERPROC, glDeleteShader)                                                       \
    X(PFNGLUSEPROGRAMPROC, glUseProgram)
#define DECLARE(T, N) inline T N = nullptr;
SHADER_FUNCTIONS(DECLARE)
#undef DECLARE
inline void loadShaders() {
#define LOAD(T, N)                                                                                 \
    N = reinterpret_cast<T>(glfwGetProcAddress(#N));                                               \
    if (!N)                                                                                        \
        throw std::runtime_error("Missing OpenGL function: " #N);
    SHADER_FUNCTIONS(LOAD)
#undef LOAD
}
