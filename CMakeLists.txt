cmake_minimum_required(VERSION 3.20)

if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

project(OpenVR-SpaceOverride)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# The Vulkan validation layer adds extra reporting that also surfaces validation    
# issues originating from external sources (e.g. SteamVR). Defaults on.
option(ENABLE_VULKAN_VALIDATION "Enable Vulkan validation layers" OFF)

option(BUILD_OVERLAY "Build OpenVR-SpaceOverride overlay executable" ON)
option(BUILD_DRIVER "Build driver_spaceoverride DLL" ON)

set(OpenVR_ROOT ${CMAKE_SOURCE_DIR}/3rdparty/OpenVR)

if(BUILD_OVERLAY OR BUILD_DRIVER)
    find_package(OpenVR REQUIRED)
endif()

add_subdirectory(3rdparty)

message(STATUS "ENABLE_VULKAN_VALIDATION = ${ENABLE_VULKAN_VALIDATION}")

add_library(calibration_math STATIC
    "src/calibration/Math.cpp"
    "src/calibration/Drift.cpp"
    "src/calibration/PoseBlend.cpp"
    "src/calibration/PredictionLag.cpp"
)

target_compile_features(calibration_math PUBLIC cxx_std_20)

target_include_directories(calibration_math
    PUBLIC
        "${CMAKE_SOURCE_DIR}/include"
)

target_compile_definitions(calibration_math
    PUBLIC
        _HAS_DEPRECATED_NEGATORS=1
        _SILENCE_ALL_CXX17_DEPRECATION_WARNINGS
)

target_link_libraries(calibration_math
    PUBLIC
        Eigen
)

if(BUILD_OVERLAY)
add_executable(OpenVR-SpaceOverride
    "src/overlay/Main.cpp"
    "src/overlay/VulkanRenderer.cpp"
    "src/overlay/ImGuiWindow.cpp"
    "src/overlay/Calibration.cpp"
    "src/overlay/Configuration.cpp"
    "src/overlay/IPCClient.cpp"
    "src/overlay/UserInterface.cpp"
    "src/overlay/EmbeddedFiles.cpp"
)

target_compile_features(OpenVR-SpaceOverride PUBLIC cxx_std_20)

target_include_directories(OpenVR-SpaceOverride
    PRIVATE
        "${CMAKE_SOURCE_DIR}/include/overlay"
        "${CMAKE_SOURCE_DIR}/include/shared"
)

target_compile_definitions(OpenVR-SpaceOverride
    PRIVATE
        _HAS_DEPRECATED_NEGATORS=1
        _SILENCE_ALL_CXX17_DEPRECATION_WARNINGS
)

if (ENABLE_VULKAN_VALIDATION)
    target_compile_definitions(OpenVR-SpaceOverride PRIVATE ENABLE_VULKAN_VALIDATION)
endif()

if (MSVC)
    target_compile_options(OpenVR-SpaceOverride PRIVATE /bigobj)
endif()

# Windows executable icon
if (WIN32)
    target_sources(OpenVR-SpaceOverride PRIVATE "resources/winicon.rc")
endif()


target_link_libraries(OpenVR-SpaceOverride
    PRIVATE
        OpenVR::API
        ImGui
        Eigen
        PicoJSON
        calibration_math
)
endif()

if(BUILD_DRIVER)
add_library(driver_spaceoverride SHARED
    "src/driver/Main.cpp"
    "src/driver/ServerTrackedDeviceProvider.cpp"
    "src/driver/IPCServer.cpp"
    "src/driver/InterfaceHookInjector.cpp"
    "src/driver/Hooking.cpp"
    "src/driver/Logging.cpp"
)

target_compile_features(driver_spaceoverride PRIVATE cxx_std_20)

target_include_directories(driver_spaceoverride PRIVATE
    "${CMAKE_SOURCE_DIR}/include/driver"
    "${CMAKE_SOURCE_DIR}/include/shared"
)

target_link_libraries(driver_spaceoverride PRIVATE
    OpenVR::Driver
    MinHook
    calibration_math
)

set_property(TARGET driver_spaceoverride PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreadedDLL")
set_target_properties(driver_spaceoverride PROPERTIES PREFIX "")
endif()

option(BUILD_TESTS "Build calibration unit tests" ON)

if(BUILD_TESTS)
    include(FetchContent)
    FetchContent_Declare(
        Catch2
        GIT_REPOSITORY https://github.com/catchorg/Catch2.git
        GIT_TAG v3.5.4
    )
    FetchContent_MakeAvailable(Catch2)

    add_executable(calibration_math_test
        "tests/calibration_math_test.cpp"
    )

    target_compile_features(calibration_math_test PRIVATE cxx_std_20)

    target_compile_definitions(calibration_math_test
        PRIVATE
            _HAS_DEPRECATED_NEGATORS=1
            _SILENCE_ALL_CXX17_DEPRECATION_WARNINGS
    )

    target_link_libraries(calibration_math_test
        PRIVATE
            calibration_math
            Catch2::Catch2WithMain
    )

    enable_testing()
    add_test(NAME calibration_math_test COMMAND calibration_math_test)
endif()