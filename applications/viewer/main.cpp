#include <alpine/alpine.h>

#include <GLFW/glfw3.h>
#include <GL/gl.h>
#include <numbers>

static constexpr uint32_t MEMORY_ARENA_SIZE = 1024 * 1024 * 1024;

static constexpr uint32_t WIDTH = 512;
static constexpr uint32_t HEIGHT = 512;
static constexpr uint32_t MAX_DEPTH = 8;

static constexpr float PAN_SPEED = 0.01f;
static constexpr float ZOOM_SPEED = 0.5f;

static constexpr float DELTA_TIME = 1.0f / 60.0f;

enum class MouseAction
{
    Released,
    LeftPressed,
    MiddlePressed,
} gMouseAction = MouseAction::Released;
float gMousePos[2] = { 0.0f };

alpine::api::Camera* gCamera = nullptr;

float gTime = 0.0f;

void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods)
{
    if (action == GLFW_PRESS)
    {
        double cx, cy;
        glfwGetCursorPos(window, &cx, &cy);
        gMousePos[0] = static_cast<float>(cx);
        gMousePos[1] = static_cast<float>(cy);

        switch (button)
        {
        case GLFW_MOUSE_BUTTON_LEFT:
            gMouseAction = MouseAction::LeftPressed;
            break;
        case GLFW_MOUSE_BUTTON_MIDDLE:
            gMouseAction = MouseAction::MiddlePressed;
            break;
        default:
            gMouseAction = MouseAction::Released;
            break;
        }
    }
    else
    {
        gMouseAction = MouseAction::Released;
    }
}

void cursorPositionCallback(GLFWwindow* window, double xpos, double ypos)
{
    if (gMouseAction != MouseAction::Released)
    {
        float deltaX = static_cast<float>(xpos) - gMousePos[0];
        float deltaY = static_cast<float>(ypos) - gMousePos[1];

        if (gMouseAction == MouseAction::LeftPressed)
        {
            float theta = deltaX / static_cast<float>(WIDTH) * std::numbers::pi_v<float>;
            float phi = deltaY / static_cast<float>(HEIGHT) * std::numbers::pi_v<float>;
            gCamera->orbit(theta, phi);
        }
        else if (gMouseAction == MouseAction::MiddlePressed)
        {
            gCamera->pan(deltaX * PAN_SPEED, deltaY * PAN_SPEED);
        }

        alpine::resetAccumulation();
    }
}

void scrollCallback(GLFWwindow* window, double xoffset, double yoffset)
{
    float zoom = -static_cast<float>(yoffset) * ZOOM_SPEED;
    gCamera->zoom(zoom);
    alpine::resetAccumulation();
}

void updateScene(GLFWwindow* window)
{
    if (!alpine::isDynamicScene())
    {
        return;
    }

    bool isRightPressed = glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS;
    bool isLeftPressed = glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS;

    if (isRightPressed || isLeftPressed)
    {
        gTime = isRightPressed ? gTime + DELTA_TIME : std::max(gTime - DELTA_TIME, 0.0f);
        alpine::updateScene(gTime);
        alpine::resetAccumulation();
    }
}

int main(int argc, char* argv[])
{
    if (argc < 2)
    {
        printf("ERROR: invalid input parameters\n");
        return 1;
    }

    if (!glfwInit())
    {
        printf("ERROR: failed to initialize GLFW\n");
        return 1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

    auto* window = glfwCreateWindow(
        WIDTH, HEIGHT, "alpine viewer", nullptr, nullptr);
    if (!window)
    {
        glfwTerminate();
        return 1;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    glfwSetMouseButtonCallback(window, mouseButtonCallback);
    glfwSetCursorPosCallback(window, cursorPositionCallback);
    glfwSetScrollCallback(window, scrollCallback);

    alpine::initialize(
        MEMORY_ARENA_SIZE, WIDTH, HEIGHT, MAX_DEPTH, alpine::AcceleratorType::Bvh);

    const float eye[] = { 0.0f, 0.0f, -3.0f };
    const float target[] = { 0.0f, 0.0f, 0.0f };
    const float up[] = { 0.0f, 1.0f, 0.0f };

    bool loaded = alpine::load(argv[1], alpine::FileType::Gltf);
    if (!loaded)
    {
        printf("ERROR: failed to load %s\n", argv[1]);
        return 1;
    }

    alpine::buildAccelerator();

    alpine::buildLightSampler(alpine::LightSamplerType::Bvh);

    const float fovy = std::numbers::pi_v<float> / 2.0f;
    float aspect = float(WIDTH) / float(HEIGHT);
    gCamera = alpine::getCamera();
    gCamera->setLookAt(eye, target, up, fovy, aspect);

    const void* pixels = alpine::getFrameBuffer();

    while (!glfwWindowShouldClose(window))
    {
        glfwPollEvents();

        updateScene(window);

        alpine::render(1);
        alpine::resolve(false);

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
