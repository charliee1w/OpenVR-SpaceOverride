// SPDX-License-Identifier: AGPL-3.0-only

#ifdef _WIN32
#include <Windows.h>
#endif

#include "ImGuiWindow.h"

#include <imgui.h>
#include <backends/imgui_impl_sdl3.h>
#include <backends/imgui_impl_vulkan.h>

#include "imgui_impl_openvr.h"

#include <SDL3/SDL.h>
#include <SDL3/SDL_vulkan.h>

#include <math.h>

#include "EmbeddedFiles.h"

#define IMGUI_NORMALIZED_RGBA(r, g, b, a) ImVec4(((r) / 255.0f), ((g) / 255.0f), ((b) / 255.0f), ((a) / 255.0f))

ImGuiWindow::ImGuiWindow()
{
    window_ = nullptr;
    window_data_ = {};
    width_ = 0;
    height_ = 0;
    window_shown_ = false;
    window_minimized_ = false;
}

auto ImGuiWindow::Initialize(VulkanRenderer*& renderer, VrOverlay*& overlay, const char* name, int width, int height) -> void
{
    width_ = width;
    height_ = height;

    auto sdl_window_flags = SDL_WINDOW_VULKAN | SDL_WINDOW_HIDDEN | SDL_WINDOW_MOUSE_FOCUS;
    window_ = SDL_CreateWindow(name, width, height, sdl_window_flags);
    if (window_ == nullptr) {
#ifdef _WIN32
        MessageBoxA(NULL, SDL_GetError(), "Space Override", MB_OK);
#else
        printf("SDL_CreateWindow(): %s\n", SDL_GetError());
#endif
        return;
    }

    VkSurfaceKHR surface = {};
    if (SDL_Vulkan_CreateSurface(window_, renderer->Instance(), renderer->Allocator(), &surface) == 0) {
#ifdef _WIN32
        MessageBoxA(NULL, SDL_GetError(), "Space Override", MB_OK);
#else
        printf("SDL_Vulkan_CreateSurface(): %s\n", SDL_GetError());
#endif
        return;
    }

    Vulkan_Window* window = &window_data_;
    renderer->SetupWindow(window, surface, width, height);

    SDL_SetWindowPosition(window_, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED);
    SDL_CaptureMouse(false);

    window_shown_ = false;

    IMGUI_CHECKVERSION();

    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    (void)io;

    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_IsSRGB; // NOTE: ImGuiConfigFlags_IsSRGB is not used by ImGui, used to communicate state.

    io.IniFilename = nullptr;
    io.DisplaySize = ImVec2(static_cast<float>(width), static_cast<float>(height));

    io.Fonts->AddFontFromMemoryCompressedTTF(DroidSans_compressed_data, DroidSans_compressed_size, 24.0f);

    ImGui::StyleColorsDark();

    // https://github.com/ocornut/imgui/issues/707#issuecomment-4107169777

    ImGuiStyle& style = ImGui::GetStyle();
    ImVec4* colors = style.Colors;

    style.WindowPadding = ImVec2(12.0f, 12.0f);
    style.FramePadding = ImVec2(6.0f, 4.0f);
    style.ItemSpacing = ImVec2(8.0f, 6.0f);
    style.ScrollbarSize = 14.0f;
    style.GrabMinSize = 12.0f;

    style.WindowRounding = 8.0f;
    style.FrameRounding = 5.0f;
    style.PopupRounding = 5.0f;
    style.ScrollbarRounding = 12.0f;
    style.GrabRounding = 5.0f;
    style.TabRounding = 5.0f;

    style.WindowBorderSize = 1.0f;
    style.FrameBorderSize = 0.0f;
    style.PopupBorderSize = 1.0f;

    colors[ImGuiCol_Text] = IMGUI_NORMALIZED_RGBA(204, 214, 244, 255);
    colors[ImGuiCol_TextDisabled] = IMGUI_NORMALIZED_RGBA(108, 112, 134, 255);

    colors[ImGuiCol_WindowBg] = IMGUI_NORMALIZED_RGBA(30, 30, 46, 255);
    colors[ImGuiCol_ChildBg] = IMGUI_NORMALIZED_RGBA(24, 24, 37, 255);
    colors[ImGuiCol_PopupBg] = IMGUI_NORMALIZED_RGBA(17, 17, 27, 245);

    colors[ImGuiCol_Border] = IMGUI_NORMALIZED_RGBA(49, 50, 68, 255);
    colors[ImGuiCol_BorderShadow] = IMGUI_NORMALIZED_RGBA(0, 0, 0, 0);

    colors[ImGuiCol_FrameBg] = IMGUI_NORMALIZED_RGBA(49, 50, 68, 255);
    colors[ImGuiCol_FrameBgHovered] = IMGUI_NORMALIZED_RGBA(69, 71, 90, 255);
    colors[ImGuiCol_FrameBgActive] = IMGUI_NORMALIZED_RGBA(79, 81, 105, 255);

    colors[ImGuiCol_TitleBg] = IMGUI_NORMALIZED_RGBA(24, 24, 37, 255);
    colors[ImGuiCol_TitleBgActive] = IMGUI_NORMALIZED_RGBA(30, 30, 46, 255);
    colors[ImGuiCol_TitleBgCollapsed] = IMGUI_NORMALIZED_RGBA(17, 17, 27, 255);

    colors[ImGuiCol_MenuBarBg] = IMGUI_NORMALIZED_RGBA(24, 24, 37, 255);

    colors[ImGuiCol_ScrollbarBg] = IMGUI_NORMALIZED_RGBA(24, 24, 37, 255);
    colors[ImGuiCol_ScrollbarGrab] = IMGUI_NORMALIZED_RGBA(79, 81, 105, 255);
    colors[ImGuiCol_ScrollbarGrabHovered] = IMGUI_NORMALIZED_RGBA(95, 99, 128, 255);
    colors[ImGuiCol_ScrollbarGrabActive] = IMGUI_NORMALIZED_RGBA(108, 112, 134, 255);

    colors[ImGuiCol_CheckMark] = IMGUI_NORMALIZED_RGBA(180, 190, 254, 255);

    colors[ImGuiCol_SliderGrab] = IMGUI_NORMALIZED_RGBA(116, 199, 236, 255);
    colors[ImGuiCol_SliderGrabActive] = IMGUI_NORMALIZED_RGBA(116, 199, 236, 255);

    colors[ImGuiCol_Button] = IMGUI_NORMALIZED_RGBA(49, 50, 68, 255);
    colors[ImGuiCol_ButtonHovered] = IMGUI_NORMALIZED_RGBA(203, 166, 247, 255);
    colors[ImGuiCol_ButtonActive] = IMGUI_NORMALIZED_RGBA(186, 150, 233, 255);

    colors[ImGuiCol_Header] = IMGUI_NORMALIZED_RGBA(49, 50, 68, 255);
    colors[ImGuiCol_HeaderHovered] = IMGUI_NORMALIZED_RGBA(69, 71, 90, 255);
    colors[ImGuiCol_HeaderActive] = IMGUI_NORMALIZED_RGBA(79, 81, 105, 255);

    colors[ImGuiCol_Tab] = IMGUI_NORMALIZED_RGBA(30, 30, 46, 255);
    colors[ImGuiCol_TabHovered] = IMGUI_NORMALIZED_RGBA(79, 81, 105, 255);
    colors[ImGuiCol_TabActive] = IMGUI_NORMALIZED_RGBA(49, 50, 68, 255);
    colors[ImGuiCol_TabUnfocused] = IMGUI_NORMALIZED_RGBA(24, 24, 37, 255);
    colors[ImGuiCol_TabUnfocusedActive] = IMGUI_NORMALIZED_RGBA(30, 30, 46, 255);

    colors[ImGuiCol_PlotLines] = IMGUI_NORMALIZED_RGBA(245, 194, 132, 255);
    colors[ImGuiCol_TextSelectedBg] = IMGUI_NORMALIZED_RGBA(79, 81, 105, 255);
    colors[ImGuiCol_NavHighlight] = IMGUI_NORMALIZED_RGBA(180, 190, 254, 255);

    style.ScaleAllSizes(1.0f);
    style.FontScaleDpi = 1.0f;

    if (io.ConfigFlags & ImGuiConfigFlags_IsSRGB) {
        // hack: ImGui doesn't handle sRGB colour spaces properly so convert from Linear -> sRGB
        // https://github.com/ocornut/imgui/issues/8271#issuecomment-2564954070
        // remove when these are merged:
        //  https://github.com/ocornut/imgui/pull/8110
        //  https://github.com/ocornut/imgui/pull/8111
        for (int i = 0; i < ImGuiCol_COUNT; i++) {
            ImVec4& col = style.Colors[i];
            col.x = col.x <= 0.04045f ? col.x / 12.92f : pow((col.x + 0.055f) / 1.055f, 2.4f);
            col.y = col.y <= 0.04045f ? col.y / 12.92f : pow((col.y + 0.055f) / 1.055f, 2.4f);
            col.z = col.z <= 0.04045f ? col.z / 12.92f : pow((col.z + 0.055f) / 1.055f, 2.4f);
        }
    }

    VkSurfaceFormatKHR render_format =
    {
        .format = VK_FORMAT_R8G8B8A8_SRGB,
        .colorSpace = VK_COLOR_SPACE_SRGB_NONLINEAR_KHR
    };

    VkPipelineRenderingCreateInfoKHR pipeline_rendering_create_info =
    {
        .sType = VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO_KHR,
        .viewMask = 0,
        .colorAttachmentCount = 1,
        .pColorAttachmentFormats = &render_format.format,
        .depthAttachmentFormat = VK_FORMAT_UNDEFINED,
        .stencilAttachmentFormat = VK_FORMAT_UNDEFINED,
    };

    ImGui_ImplVulkan_InitInfo init_info = {
        .ApiVersion = VK_API_VERSION_1_3,
        .Instance = renderer->Instance(),
        .PhysicalDevice = renderer->PhysicalDevice(),
        .Device = renderer->Device(),
        .QueueFamily = renderer->QueueFamily(),
        .Queue = renderer->Queue(),
        .DescriptorPool = renderer->DescriptorPool(),
        .MinImageCount = renderer->MinimumConcurrentImageCount(),
        .ImageCount = window->image_count,
        .PipelineCache = renderer->PipelineCache(),
        .PipelineInfoMain = {
            .RenderPass = VK_NULL_HANDLE,
            .Subpass = 0,
            .MSAASamples = VK_SAMPLE_COUNT_1_BIT,
            .PipelineRenderingCreateInfo = pipeline_rendering_create_info,
        },
        .UseDynamicRendering = true,
        .Allocator = renderer->Allocator(),
        .CheckVkResultFn = nullptr,
    };

    ImGui_ImplSDL3_InitForVulkan(window_);
    ImGui_ImplVulkan_Init(&init_info);

    ImGui_ImplOpenVR_InitInfo openvr_init_info =
    {
        .handle = overlay->Handle(),
        .width = width,
        .height = height
    };

    ImGui_ImplOpenVR_Init(&openvr_init_info);

    renderer->SetupRenderTarget(width, height, render_format);

    m_userInterface_ = {};
}

auto ImGuiWindow::Show() -> void
{
    SDL_ShowWindow(window_);
    SDL_RestoreWindow(window_);

    window_shown_ = true;
}

auto ImGuiWindow::SetMinimizedFromEvent(bool state) -> void
{
    window_minimized_ = state;
}

auto ImGuiWindow::Hide() -> void
{
    SDL_HideWindow(window_);

    window_shown_ = false;
}

auto ImGuiWindow::Draw(bool dashboardVisible) -> void
{
    ImGui_ImplVulkan_NewFrame();
    ImGui_ImplSDL3_NewFrame();
    ImGui_ImplOpenVR_NewFrame();

    ImGui::GetIO().DisplaySize = ImVec2(static_cast<float>(width_), static_cast<float>(height_));

    ImGui::NewFrame();

    m_userInterface_.Render(dashboardVisible);

    ImGui::Render();
}

auto ImGuiWindow::Destroy(VulkanRenderer*& renderer) -> void
{
    ImGui_ImplOpenVR_Shutdown();
    ImGui_ImplVulkan_Shutdown();
    ImGui_ImplSDL3_Shutdown();
    SDL_DestroyWindow(window_);
}
