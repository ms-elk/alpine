#include <alpine.h>

#include <GLFW/glfw3.h>
#include <GL/gl.h>

#include <string>
#include <vector>

#include <chrono>

static constexpr float PI = 3.14159265358979323846f;

static constexpr uint32_t WIDTH = 256;// 960;
static constexpr uint32_t HEIGHT = 256;// 540;

static constexpr float PAN_SPEED = 0.01f;
static constexpr float ZOOM_SPEED =0.5f;

enum class MouseAction
{
    Released,
    LeftPressed,
    MiddlePressed,
} gMouseAction = MouseAction::Released;
float gMousePos[2] = { 0.0f };

alpine::ICamera* gCamera = nullptr;

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
            float theta = deltaX / static_cast<float>(WIDTH) * PI;
            float phi = deltaY / static_cast<float>(HEIGHT) * PI;
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
    float zoom = static_cast<float>(yoffset) * ZOOM_SPEED;
    gCamera->zoom(zoom);
    alpine::resetAccumulation();
}

int main(int argc, char* argv[])
{
    using clock = std::chrono::high_resolution_clock;
    const clock::time_point appStartTp = clock::now();

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
    glfwSetScrollCallback(window, scrollCallback);

    const uint32_t maxDepth = 8;
    alpine::initialize(WIDTH, HEIGHT, maxDepth);
    alpine::setBackgroundColor(0.0f, 0.0f, 0.0f);

    //float emisssion[] = { 15.0f, 15.0f, 15.0f };
    //float lightPos[] = { 278.0f, 548.7f, 227.0f };
    //float lightRadius = 100.0f;
    //alpine::setLight(emisssion, lightPos, lightRadius);

    std::vector<alpine::ILight*> lightGroup0;
    {
        float emisssion[] = { 10.0f, 10.0f, 10.0f };
        float lightPos[] = { 0.0f, 2.9f, 0.0f };
        float lightRadius = 1.0f;
        lightGroup0.push_back(alpine::addDiskLight(emisssion, lightPos, lightRadius));
    }

    {
        float emisssion[] = { 30.0f, 20.0f, 5.0f };
        float lightPos[] = { 0.0f, 2.5f, -4.0f };
        float lightRadius = 0.3f;
        lightGroup0.push_back(alpine::addDiskLight(emisssion, lightPos, lightRadius));
    }

#if 0
    {
        float emisssion[] = { 30.0f, 20.0f, 5.0f };
        float lightPos[] = { 0.5f, 1.75f, 2.7f };
        float lightRadius = 0.1f;
        lightGroup0.push_back(alpine::addDiskLight(emisssion, lightPos, lightRadius));
        lightGroup0[2]->enable(false);
    }

    {
        float emisssion[] = { 30.0f, 20.0f, 5.0f };
        float lightPos[] = { -0.5f, 1.75f, 2.7f };
        float lightRadius = 0.1f;
        lightGroup0.push_back(alpine::addDiskLight(emisssion, lightPos, lightRadius));
        lightGroup0[3]->enable(false);
    }
#endif

    std::vector<alpine::ILight*> lightGroup1(3);
    {
        float intensity[] = { 0.0f, 50.0f, 50.0f };
        float lightPos[] = { 1.5f, 2.8f, 1.5f };
        lightGroup1[0] = alpine::addDiskLight(intensity, lightPos, 0.3f);
        lightGroup1[0]->enable(false);
    }

    {
        float intensity[] = { 50.0f, 0.0f, 50.0f };
        float lightPos[] = { 1.5f, 2.8f, -1.5f };
        lightGroup1[1] = alpine::addDiskLight(intensity, lightPos, 0.3f);
        lightGroup1[1]->enable(false);
    }

    {
        float intensity[] = { 50.0f, 50.0f, 0.0f };
        float lightPos[] = { -1.5f, 2.8f, 1.5f };
        lightGroup1[2] = alpine::addDiskLight(intensity, lightPos, 0.3f);
        lightGroup1[2]->enable(false);
    }

    gCamera = alpine::getCamera();

    //const float eye[] = { 1.0f, 1.0f, 0.0f };
    //const float target[] = { 0.0f, 0.0f, 0.0f };
    //const float up[] = { 0.0f, 1.0f, 0.0f };

    //const float eye[] = { 278.0f, 273.0f, -600.0f };
    //const float target[] = { 278.0f, 273.0f, 0.0f };
    //const float up[] = { 0.0f, 1.0f, 0.0f };

    const float eye0[] = { 2.24f, 1.8f, -6.64f };
    const float target0[] = { 3.0f, 0.8f, 0.0f };
    const float up[] = { 0.0f, 1.0f, 0.0f };

    const float eye1[] = { 2.5f, 1.8f, -1.6f };
    const float target1[] = { 0.18f, 1.0f, 0.2f };
    //const float up[] = { 0.0f, 1.0f, 0.0f };

    const char* objFilename = argv[1];
    bool loaded = alpine::load(objFilename, alpine::FileType::GLTF);
    //bool loaded = alpine::load(objFilename, alpine::FileType::OBJ);
    if (!loaded)
    {
        printf("ERROR: failed to load %s\n", objFilename);
        return 1;
    }
    //alpine::addDebugScene();

    const float fovy = PI / 2.0f;
    float aspect = float(WIDTH) / float(HEIGHT);
    gCamera->setLookAt(eye0, target0, up, fovy, aspect);

    const void* pixels = alpine::getFrameBuffer();

    const float deltaLightRot = PI / 18.0f;
    uint32_t frame = 0;
    while (!glfwWindowShouldClose(window))
    {
        const clock::time_point frameStartTp = clock::now();
        glfwPollEvents();

#if 0
        if (frame >= 40)
        {
            lightGroup0[0]->enable(false);
            lightGroup0[1]->enable(false);

            float scale = static_cast<float>(frame - 40) / 3.0f;
            scale = std::min(scale, 1.0f);

            for (uint32_t i = 0; i < lightGroup1.size(); ++i)
            {
                float offset = static_cast<float>(i) * 2.0f * PI / static_cast<float>(lightGroup1.size());
                float theta = static_cast<float>(frame - 40) * deltaLightRot + offset;
                float p[] = { cosf(theta), 2.8f, sinf(theta) };
                lightGroup1[i]->setPosition(p);
                lightGroup1[i]->enable(true);
                lightGroup1[i]->setScale(scale);
            }
        }
        else
        {
            float scale = frame >= 30 ? 1.0f - static_cast<float>(frame - 30) / 9.0f : 1.0f;

            for (auto& l0 : lightGroup0)
            {
                l0->enable(true);
                l0->setScale(scale);
            }

            for (auto& l1 : lightGroup1)
            {
                l1->enable(false);
            }
        }

        float t = static_cast<float>(frame) / 29.0f;
        t = std::min(t, 1.0f);

        float eye[3];
        eye[0] = eye0[0] * (1.0f - t) + eye1[0] * t;
        eye[1] = eye0[1] * (1.0f - t) + eye1[1] * t;
        eye[2] = eye0[2] * (1.0f - t) + eye1[2] * t;

        float target[3];
        target[0] = target0[0] * (1.0f - t) + target1[0] * t;
        target[1] = target0[1] * (1.0f - t) + target1[1] * t;
        target[2] = target0[2] * (1.0f - t) + target1[2] * t;

        gCamera->setLookAt(eye, target, up, fovy, aspect);

        alpine::resetAccumulation();
#endif

        alpine::render(1);
        alpine::resolve(true);

        glRasterPos2i(-1, 1);
        glPixelZoom(1.0f, -1.0f);
        glDrawPixels(WIDTH, HEIGHT, GL_RGB, GL_UNSIGNED_BYTE, pixels);

        glfwSwapBuffers(window);

        if (frame < 80)
        {
            // 起動からの時刻とフレーム時間を計算。
            const clock::time_point now = clock::now();
            const clock::duration frameTime = now - frameStartTp;
            const clock::duration totalTime = now - appStartTp;
            printf(
                "Done: %.3f [ms] (total: %.3f [s])\n",
                std::chrono::duration_cast<std::chrono::microseconds>(frameTime).count() * 1e-3f,
                std::chrono::duration_cast<std::chrono::milliseconds>(totalTime).count() * 1e-3f);
        }

        frame++;
        frame = std::min(frame, 80u);
    }

    // cleanup
    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}