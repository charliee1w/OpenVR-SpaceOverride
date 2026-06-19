// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <imgui.h>
#include <backends/imgui_impl_vulkan.h>

#include <SDL3/SDL.h>

#include "VulkanRenderer.h"
#include "VrOverlay.h"

class ImGuiWindow
{
public:
    explicit ImGuiWindow();
    auto Initialize(VulkanRenderer*& renderer, VrOverlay*& overlay, const char* name, int width, int height) -> void;

    [[nodiscard]] auto Window() const -> SDL_Window* { return window_; };
    [[nodiscard]] auto WindowData() -> Vulkan_Window* { return &window_data_; };
    [[nodiscard]] auto Shown() const -> bool { return window_shown_; };
    [[nodiscard]] auto Minimized() const -> bool { return window_minimized_; };

    auto Hide() -> void;
    auto Show() -> void;
    auto SetMinimizedFromEvent(bool state) -> void;
    auto Draw(bool dashboardVisible) -> void;

    auto Destroy(VulkanRenderer*& renderer) -> void;

private:

    SDL_Window* window_;
    Vulkan_Window window_data_;
    int width_;
    int height_;
    bool window_shown_;
    bool window_minimized_;
};
