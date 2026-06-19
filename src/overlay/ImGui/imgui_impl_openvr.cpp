// dear imgui: Platform backend for OpenVR
// This needs to be used along with a Renderer Backend (e.g. Vulkan)

// Implemented features:
//  [X] Platform: Virtual keyboard support
//  [X] Platform: Mouse emulation
// Missing features or Issues:
//  [ ] Platform: Touch Emulation

// You can use unmodified imgui_impl_* files in your project. See examples/ folder for examples of using this.
// Prefer including the entire imgui/ repository into your project (either as a copy or as a submodule), and only build the backends you need.
// Learn about Dear ImGui:
// - FAQ                  https://dearimgui.com/faq
// - Getting Started      https://dearimgui.com/getting-started
// - Documentation        https://dearimgui.com/docs (same as your local docs/ folder).
// - Introduction, links and more at the top of imgui.cpp

#include <imgui.h>
#ifndef IMGUI_DISABLE
#include "imgui_impl_openvr.h"

#include <openvr.h>

struct ImGui_ImplOpenVR_Data {
    uintptr_t handle;
    uint32_t width;
    uint32_t height;
    bool keyboard_active;
    vr::HmdVector2_t mouse_scale;

    ImGui_ImplOpenVR_Data() { memset((void*)this, 0, sizeof(*this)); }
};

static ImGui_ImplOpenVR_Data* g_openvr_backend = nullptr;

static ImGui_ImplOpenVR_Data* ImGui_ImplOpenVR_GetBackendData()
{
    return g_openvr_backend;
}

bool ImGui_ImplOpenVR_Init(ImGui_ImplOpenVR_InitInfo* initInfo)
{
    IMGUI_CHECKVERSION();
    IM_ASSERT(g_openvr_backend == nullptr && "Already initialized the OpenVR backend!");

    ImGui_ImplOpenVR_Data* bd = IM_NEW(ImGui_ImplOpenVR_Data)();
    g_openvr_backend = bd;

    bd->handle = initInfo->handle;
    bd->width = initInfo->width;
    bd->height = initInfo->height;
    bd->keyboard_active = false;

    return true;
}

bool ImGui_ImplOpenVR_ProcessOverlayEvent(const vr::VREvent_t& event)
{
    ImGui_ImplOpenVR_Data* bd = ImGui_ImplOpenVR_GetBackendData();
    IM_ASSERT(bd != nullptr && "Context or backend not initialized! Did you call ImGui_ImplOpenVR_Init()?");

    ImGuiIO& io = ImGui::GetIO();
    switch (event.eventType)
    {
        case vr::VREvent_MouseMove:
        {
            // OpenGL uses coordinate space Bottom Left == 0,0 where as Vulkan is Top Left == 0,0
            // So we need to flip the y-axis to get the correct mouse position data
            io.AddMousePosEvent(event.data.mouse.x, io.DisplaySize.y - event.data.mouse.y);
            break;
        }
        case vr::VREvent_MouseButtonDown:
        {
            uint64_t mask = event.data.mouse.button;
            mask &= vr::VRMouseButton_Left | vr::VRMouseButton_Right | vr::VRMouseButton_Middle;
            // Ensure that the event sends a bitmask with only a single flag
            // this may be redundant but is here in case driver sends bad data
            if (mask && (mask & (mask - 1))) {
                break;
            }

            int mouse_button = ImGuiMouseButton_COUNT;

            // Most drivers only send VRMouseButton_Left but there are some drivers
            // which also send VRMouseButton_Right and VRMouseButton_Middle
            // right click is often mapped to A, X or B
            // middle click is often mapped to trackpad click
            if (event.data.mouse.button & vr::VRMouseButton_Left)
                mouse_button = ImGuiMouseButton_Left;
            else if (event.data.mouse.button & vr::VRMouseButton_Right)
                mouse_button = ImGuiMouseButton_Right;
            else if (event.data.mouse.button & vr::VRMouseButton_Middle)
                mouse_button = ImGuiMouseButton_Middle;

            if (mouse_button < ImGuiMouseButton_COUNT)
                io.AddMouseButtonEvent(mouse_button, true);
            break;
        }
        case vr::VREvent_MouseButtonUp:
        {
            uint64_t mask = event.data.mouse.button;
            mask &= vr::VRMouseButton_Left | vr::VRMouseButton_Right | vr::VRMouseButton_Middle;
            if (mask && (mask & (mask - 1))) {
                break;
            }

            int mouse_button = ImGuiMouseButton_COUNT;

            if (event.data.mouse.button & vr::VRMouseButton_Left)
                mouse_button = ImGuiMouseButton_Left;
            else if (event.data.mouse.button & vr::VRMouseButton_Right)
                mouse_button = ImGuiMouseButton_Right;
            else if (event.data.mouse.button & vr::VRMouseButton_Middle)
                mouse_button = ImGuiMouseButton_Middle;

            if (mouse_button < ImGuiMouseButton_COUNT)
                io.AddMouseButtonEvent(mouse_button, false);
            break;
        }
        case vr::VREvent_ScrollDiscrete:
        case vr::VREvent_ScrollSmooth:
        {
            // Emulate physical mouse behaviour by only sending y-axis
            // VREvent_ScrollDiscrete sends discrete values [-1.0, 1.0]
            // VREvent_ScrollSmooth sends continuous values [-1.0, 1.0]
            const float y = event.data.scroll.ydelta;
            if (y != 0.0f)
                io.AddMouseWheelEvent(0.0f, y);
            break;
        }
        case vr::VREvent_KeyboardCharInput:
        {
            // Some special inputs ie. Backspace, Enter, etc...
            // are not handled by AddInputCharactersUTF8
            // because it only allows UTF-8 character input
            switch (event.data.keyboard.cNewInput[0])
            {
                case 8: // Backspace
                {
                    io.AddKeyEvent(ImGuiKey_Backspace, true);
                    io.AddKeyEvent(ImGuiKey_Backspace, false);
                    break;
                }
                case 10: // Enter
                {
                    io.AddKeyEvent(ImGuiKey_Enter, true);
                    io.AddKeyEvent(ImGuiKey_Enter, false);
                    vr::VROverlay()->HideKeyboard();
                    break;
                }
                case 27: // arrow keys
                {
                    uint8_t direction = event.data.keyboard.cNewInput[2];
                    switch (direction)
                    {
                    case 68:
                    {
                        io.AddKeyEvent(ImGuiKey_LeftArrow, true);
                        io.AddKeyEvent(ImGuiKey_LeftArrow, false);
                        break;
                    }
                    case 67:
                    {
                        io.AddKeyEvent(ImGuiKey_RightArrow, true);
                        io.AddKeyEvent(ImGuiKey_RightArrow, false);
                        break;
                    }
                    case 65:
                    {
                        io.AddKeyEvent(ImGuiKey_UpArrow, true);
                        io.AddKeyEvent(ImGuiKey_UpArrow, false);
                        break;
                    }
                    case 66:
                    {
                        io.AddKeyEvent(ImGuiKey_DownArrow, true);
                        io.AddKeyEvent(ImGuiKey_DownArrow, false);
                        break;
                    }
                    }
                    break;
                }
                default:
                {
                    io.AddInputCharactersUTF8(event.data.keyboard.cNewInput);
                    break;
                }
            }
            break;
        }
        case vr::VREvent_KeyboardClosed_Global:
        {
            if (!vr::VROverlay())
                return false;

            // let's check when VREvent_KeyboardClosed_Global is sent is our keyboard is still shown
            // this may happen because the dashboard or overlay was closed when the keyboard was open
            if (event.data.keyboard.overlayHandle == bd->handle && bd->keyboard_active) {
                io.AddKeyEvent(ImGuiKey_Enter, true);
                io.AddKeyEvent(ImGuiKey_Enter, false);
                vr::VROverlay()->HideKeyboard();
            }
            break;
        }
    }

    return true;
}

void ImGui_ImplOpenVR_Shutdown()
{
    ImGui_ImplOpenVR_Data* bd = ImGui_ImplOpenVR_GetBackendData();
    IM_ASSERT(bd != nullptr && "Context or backend not initialized! Did you call ImGui_ImplOpenVR_Init()?");

    IM_DELETE(bd);
    g_openvr_backend = nullptr;
}

void ImGui_ImplOpenVR_NewFrame()
{
    ImGui_ImplOpenVR_Data* bd = ImGui_ImplOpenVR_GetBackendData();
    IM_ASSERT(bd != nullptr && "Context or backend not initialized! Did you call ImGui_ImplOpenVR_Init()?");

    ImGuiIO& io = ImGui::GetIO();

    if (!io.WantTextInput) {
        bd->keyboard_active = false;
    }

    if (vr::VROverlay()->IsOverlayVisible(bd->handle) && !bd->keyboard_active && io.WantTextInput) {
        vr::VROverlay()->ShowKeyboardForOverlay(bd->handle, vr::k_EGamepadTextInputModeNormal, vr::k_EGamepadTextInputLineModeSingleLine, vr::KeyboardFlag_Minimal | vr::KeyboardFlag_HideDoneKey | vr::KeyboardFlag_ShowArrowKeys, "ImGui OpenVR Virtual Keyboard", 1, "", 0);
        bd->keyboard_active = true;
    }

    if (bd->width > 0 && bd->height > 0) {
        bd->mouse_scale = { (float)bd->width, (float)bd->height };
        vr::VROverlay()->SetOverlayMouseScale(bd->handle, &bd->mouse_scale);
    }
}

#endif // #ifndef IMGUI_DISABLE
