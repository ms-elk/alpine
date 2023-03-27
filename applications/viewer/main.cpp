#include <alpine.h>

#include <GLFW/glfw3.h>
#include <GL/gl.h>

static constexpr float PI = 3.14159265358979323846f;

static constexpr int WIDTH = 512;
static constexpr int HEIGHT = 512;

bool gIsMousePressed = false;
float gMousePos[2] = { 0.0f };

void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods)
{
    if (button == GLFW_MOUSE_BUTTON_LEFT)
    {
        if (action == GLFW_PRESS)
        {
            gIsMousePressed = true;

            double cx, cy;
            glfwGetCursorPos(window, &cx, &cy);
            gMousePos[0] = static_cast<float>(cx);
            gMousePos[1] = static_cast<float>(cy);
        }
        else
        {
            gIsMousePressed = false;
        }
    }
}

void cursorPositionCallback(GLFWwindow* window, double xpos, double ypos)
{
    if (gIsMousePressed)
    {
        float deltaX = static_cast<float>(xpos) - gMousePos[0];
        float deltaY = static_cast<float>(ypos) - gMousePos[1];
        float theta = deltaX / static_cast<float>(WIDTH) * PI;
        float phi = deltaY / static_cast<float>(HEIGHT) * PI;
        alpine::rotateCamera(theta, phi);
        alpine::resetAccumulation();
    }
}

int main(int argc, char* argv[])
{
    if (!glfwInit())
    {
        return 1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

    auto* window = glfwCreateWindow(WIDTH, HEIGHT, "alpine viewer", nullptr, nullptr);
    if (!window)
    {
        glfwTerminate();
        return 1;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    glfwSetMouseButtonCallback(window, mouseButtonCallback);
    glfwSetCursorPosCallback(window, cursorPositionCallback);

    const int maxDepth = 8;
    alpine::initialize(WIDTH, HEIGHT, maxDepth);

    const float eye[] = { 0.0f, 0.0f, -5.0f };
    const float at[] = { 0.0f, 0.0f, 0.0f };
    const float up[] = { 0.0f, 1.0f, 0.0f };
    alpine::addDebugScene();

    const float fovy = PI / 2.0f;
    float aspect = float(WIDTH) / float(HEIGHT);
    alpine::setCamera(eye, at, up, fovy, aspect);

    const void* pixels = alpine::getFrameBuffer();

    while (!glfwWindowShouldClose(window))
    {
        glfwPollEvents();

        alpine::render(1);

        glRasterPos2i(-1, 1);
        glPixelZoom(1.0f, -1.0f);
        glDrawPixels(WIDTH, HEIGHT, GL_RGB, GL_UNSIGNED_BYTE, pixels);

        glfwSwapBuffers(window);
    }

    // cleanup
    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}