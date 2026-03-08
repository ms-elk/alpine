#include <alpine/alpine.h>

#include <GLFW/glfw3.h>

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <numbers>

static constexpr uint32_t MEMORY_ARENA_SIZE = 512 * 1024 * 1024;

static constexpr uint32_t WINDOW_WIDTH = 800;
static constexpr uint32_t WINDOW_HEIGHT = 512;

static constexpr uint32_t IMAGE_WIDTH = 512;
static constexpr uint32_t IMAGE_HEIGHT = 512;
static constexpr uint32_t MAX_DEPTH = 8;

static constexpr float PAN_SPEED = 0.01f;
static constexpr float ZOOM_SPEED = 0.5f;

static constexpr float DELTA_TIME = 1.0f / 60.0f;

alpine::api::Camera* gCamera = nullptr;

float gTime = 0.0f;

GLuint createRenderTexture()
{
    GLuint rt;
    glGenTextures(1, &rt);
    glBindTexture(GL_TEXTURE_2D, rt);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

    glTexImage2D(
        GL_TEXTURE_2D,
        0,
        GL_RGB8,
        IMAGE_WIDTH,
        IMAGE_HEIGHT,
        0,
        GL_RGB,
        GL_UNSIGNED_BYTE,
        nullptr
    );

    glBindTexture(GL_TEXTURE_2D, 0);

    return rt;
}

void updateRenderTexture(GLuint rt, const void* pixels)
{
    glBindTexture(GL_TEXTURE_2D, rt);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    glTexImage2D(
        GL_TEXTURE_2D,
        0,
        GL_RGB8,
        IMAGE_WIDTH,
        IMAGE_HEIGHT,
        0,
        GL_RGB,
        GL_UNSIGNED_BYTE,
        pixels
    );

    glBindTexture(GL_TEXTURE_2D, 0);
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

void updateCamera()
{
    ImGui::SetItemKeyOwner(ImGuiKey_MouseLeft);
    ImGui::SetItemKeyOwner(ImGuiKey_MouseMiddle);
    ImGui::SetItemKeyOwner(ImGuiKey_MouseWheelY);

    const auto& io = ImGui::GetIO();
    auto delta = io.MouseDelta;

    if (ImGui::IsMouseDown(ImGuiMouseButton_Left))
    {
        float theta = delta.x / static_cast<float>(IMAGE_WIDTH) * std::numbers::pi_v<float>;
        float phi = delta.y / static_cast<float>(IMAGE_HEIGHT) * std::numbers::pi_v<float>;
        gCamera->orbit(theta, phi);
        alpine::resetAccumulation();
    }
    else if (ImGui::IsMouseDown(ImGuiMouseButton_Middle))
    {
        gCamera->pan(delta.x * PAN_SPEED, delta.y * PAN_SPEED);
        alpine::resetAccumulation();
    }

    float wheel = io.MouseWheel;

    if (wheel != 0.0f)
    {
        float zoom = -wheel * ZOOM_SPEED;
        gCamera->zoom(zoom);
        alpine::resetAccumulation();
    }
}

void showRenderTexturePanel(GLuint rt)
{
    ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(IMAGE_WIDTH, IMAGE_HEIGHT), ImGuiCond_Always);

    ImGuiWindowFlags flags =
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoCollapse |
        ImGuiWindowFlags_NoTitleBar;

    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);

    ImGui::Begin("Path Tracer", nullptr, flags);

    ImGui::Image(
        (ImTextureID)(intptr_t)rt,
        ImVec2(IMAGE_WIDTH, IMAGE_HEIGHT),
        ImVec2(0, 0),
        ImVec2(1, 1)
    );

    bool isHovered = ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenBlockedByActiveItem);
    if (isHovered)
    {
        updateCamera();
    }

    ImGui::End();

    ImGui::PopStyleVar(2);
}

void showLoadButton()
{
    ImGui::SetNextWindowPos(ImVec2(IMAGE_WIDTH + 16, 16), ImGuiCond_Once);
    ImGui::SetNextWindowSize(ImVec2(256, 80), ImGuiCond_Once);

    ImGui::Begin("UI");

    static char file[512] = "";
    ImGui::InputText("GLB file", file, sizeof(file));

    if (ImGui::Button("Load"))
    {
        alpine::resetScene(alpine::AcceleratorType::WideBvh);

        bool loaded = alpine::load(file, alpine::FileType::Gltf);
        if (loaded)
        {
            gTime = 0.0f;
            alpine::updateScene(gTime);
            alpine::buildLightSampler(alpine::LightSamplerType::Bvh);
            alpine::resetAccumulation();
        }
    }

    ImGui::End();
}

int main(int argc, char* argv[])
{
    if (!glfwInit())
    {
        printf("ERROR: failed to initialize GLFW\n");
        return 1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    auto* window = glfwCreateWindow(
        WINDOW_WIDTH, WINDOW_HEIGHT, "alpine viewer", nullptr, nullptr);
    if (!window)
    {
        glfwTerminate();
        return 1;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    alpine::initialize(
        MEMORY_ARENA_SIZE, IMAGE_WIDTH, IMAGE_HEIGHT, MAX_DEPTH);

    const float eye[] = { 0.0f, 0.0f, -3.0f };
    const float target[] = { 0.0f, 0.0f, 0.0f };
    const float up[] = { 0.0f, 1.0f, 0.0f };

    const float fovy = std::numbers::pi_v<float> / 2.0f;
    float aspect = float(IMAGE_WIDTH) / float(IMAGE_HEIGHT);
    gCamera = alpine::getCamera();
    gCamera->setLookAt(eye, target, up, fovy, aspect);

    const void* pixels = alpine::getFrameBuffer();

    GLuint rt = createRenderTexture();

    while (!glfwWindowShouldClose(window))
    {
        glfwPollEvents();

        updateScene(window);

        alpine::render(1);
        alpine::resolve(false);

        updateRenderTexture(rt, pixels);

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        showRenderTexturePanel(rt);

        showLoadButton();

        ImGui::Render();

        glViewport(0, 0, IMAGE_WIDTH, IMAGE_HEIGHT);
        glClearColor(0.5f, 0.5f, 0.5f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    glDeleteTextures(1, &rt);

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    // cleanup
    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}
