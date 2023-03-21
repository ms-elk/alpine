#include <alpine.h>

#include <GLFW/glfw3.h>
#include <GL/gl.h>

#define _USE_MATH_DEFINES
#include <math.h>

int main(int argc, char* argv[])
{
    if (!glfwInit())
    {
        return 1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

    const int width = 512;
    const int height = 512;
    auto* window = glfwCreateWindow(width, height, "alpine viewer", nullptr, nullptr);
    if (!window)
    {
        glfwTerminate();
        return 1;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    const int maxDepth = 8;
    alpine::initialize(width, height, maxDepth);

    const float eye[] = { 0.0f, 0.0f, -3.0f };
    const float at[] = { 0.0f, 0.0f, 0.0f };
    const float up[] = { 0.0f, 1.0f, 0.0f };
    alpine::addDebugScene();

    constexpr float fovy = float(M_PI) / 2.0f;
    float aspect = float(width) / float(height);
    alpine::setCamera(eye, at, up, fovy, aspect);

    const void* pixels = alpine::getFrameBuffer();

    while (!glfwWindowShouldClose(window))
    {
        glfwPollEvents();

        alpine::render(1);

        glRasterPos2i(-1, 1);
        glPixelZoom(1.0f, -1.0f);
        glDrawPixels(width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels);

        glfwSwapBuffers(window);
    }

    // cleanup
    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}