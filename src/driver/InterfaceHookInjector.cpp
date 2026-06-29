// SPDX-License-Identifier: AGPL-3.0-only
//
// Pose hook entry point (Windows / MinHook only).
//
// Hooks IVRDriverContext::GetGenericInterface (vtable 0) to intercept
// IVRServerDriverHost_005/_006 and patch TrackedDevicePoseUpdated (vtable 1).
//
// Tested SteamVR builds (manual):
//   - 2.8.x (IVRServerDriverHost_006) — primary dev target
//   - 2.7.x (IVRServerDriverHost_005) — supported via dual hook paths
//
// Risk: Valve may reorder IVRServerDriverHost vtables or change vrserver ABI.
// After any SteamVR update, verify override still activates and check driver log.
// See Agents.md "SteamVR hook compatibility" for maintenance checklist.

#include "Logging.h"
#include "Hooking.h"
#include "InterfaceHookInjector.h"
#include "ServerTrackedDeviceProvider.h"
#include "Main.h"

#include <atomic>
#include <windows.h>

static std::atomic<bool> g_driverShuttingDown{ false };

static Hook<void*(*)(void*, const char *, vr::EVRInitError *)>
	GetGenericInterfaceHook("IVRDriverContext::GetGenericInterface");

static Hook<void(*)(void*, uint32_t, const vr::DriverPose_t &, uint32_t)>
	TrackedDevicePoseUpdatedHook005("IVRServerDriverHost005::TrackedDevicePoseUpdated");

static Hook<void(*)(void*, uint32_t, const vr::DriverPose_t &, uint32_t)>
	TrackedDevicePoseUpdatedHook006("IVRServerDriverHost006::TrackedDevicePoseUpdated");

static bool g_poseHooksInstalled = false;

bool PoseHooksInstalled()
{
	return g_poseHooksInstalled;
}

void SetDriverShuttingDown(bool shuttingDown)
{
	g_driverShuttingDown.store(shuttingDown, std::memory_order_release);
}

bool IsDriverShuttingDown()
{
	return g_driverShuttingDown.load(std::memory_order_acquire);
}

namespace
{
	bool LogPoseSizeMismatch(uint32_t device, uint32_t structSize)
	{
		static ULONGLONG lastLogMs = 0;
		const ULONGLONG now = GetTickCount64();
		if (now - lastLogMs < 5000)
			return false;
		lastLogMs = now;
		LOG("DriverPose_t size mismatch: expected %zu got %u (device %u) — pose dropped",
			sizeof(vr::DriverPose_t), structSize, device);
		return true;
	}

	bool InstallPoseHook(
		void* originalInterface,
		Hook<void(*)(void*, uint32_t, const vr::DriverPose_t &, uint32_t)>& hook,
		void* detour)
	{
		if (IHook::Exists(hook.name))
			return true;

		if (!hook.CreateHookInObjectVTable(originalInterface, 1, detour))
		{
			LOG("Failed to install pose hook for %s", hook.name.c_str());
			return false;
		}

		IHook::Register(&hook);
		g_poseHooksInstalled = true;
		LOG("Enabled hook for %s", hook.name.c_str());
		return true;
	}
}

static void DetourTrackedDevicePoseUpdated005(void* _this, uint32_t unWhichDevice, const vr::DriverPose_t &newPose, uint32_t unPoseStructSize)
{
	if (IsDriverShuttingDown())
	{
		TrackedDevicePoseUpdatedHook005.originalFunc(_this, unWhichDevice, newPose, unPoseStructSize);
		return;
	}

	if (sizeof(vr::DriverPose_t) != unPoseStructSize)
	{
		LogPoseSizeMismatch(unWhichDevice, unPoseStructSize);
		return;
	}
	auto pose = newPose;
	if (g_server.HandleDevicePoseUpdated(unWhichDevice, pose))
	{
		TrackedDevicePoseUpdatedHook005.originalFunc(_this, unWhichDevice, pose, unPoseStructSize);
	}
}

static void DetourTrackedDevicePoseUpdated006(void* _this, uint32_t unWhichDevice, const vr::DriverPose_t &newPose, uint32_t unPoseStructSize)
{
	if (IsDriverShuttingDown())
	{
		TrackedDevicePoseUpdatedHook006.originalFunc(_this, unWhichDevice, newPose, unPoseStructSize);
		return;
	}

	if (sizeof(vr::DriverPose_t) != unPoseStructSize)
	{
		LogPoseSizeMismatch(unWhichDevice, unPoseStructSize);
		return;
	}
	auto pose = newPose;
	if (g_server.HandleDevicePoseUpdated(unWhichDevice, pose))
	{
		TrackedDevicePoseUpdatedHook006.originalFunc(_this, unWhichDevice, pose, unPoseStructSize);
	}
}

static void *DetourGetGenericInterface(void* _this, const char *pchInterfaceVersion, vr::EVRInitError *peError)
{
	auto originalInterface = GetGenericInterfaceHook.originalFunc(_this, pchInterfaceVersion, peError);
	if (!originalInterface)
		return nullptr;

	if (!pchInterfaceVersion)
		return originalInterface;

	std::string iface(pchInterfaceVersion);
	if (iface == "IVRServerDriverHost_005")
	{
		InstallPoseHook(originalInterface, TrackedDevicePoseUpdatedHook005, &DetourTrackedDevicePoseUpdated005);
	}
	else if (iface == "IVRServerDriverHost_006")
	{
		InstallPoseHook(originalInterface, TrackedDevicePoseUpdatedHook006, &DetourTrackedDevicePoseUpdated006);
	}

	return originalInterface;
}

bool InjectHooks(vr::IVRDriverContext *pDriverContext)
{
	auto err = MH_Initialize();
	if (err != MH_OK && err != MH_ERROR_ALREADY_INITIALIZED)
	{
		LOG("MH_Initialize error: %s", MH_StatusToString(err));
		return false;
	}

	if (!GetGenericInterfaceHook.CreateHookInObjectVTable(pDriverContext, 0, &DetourGetGenericInterface))
	{
		LOG("Failed to install GetGenericInterface hook");
		MH_Uninitialize();
		return false;
	}

	IHook::Register(&GetGenericInterfaceHook);
	LOG("Pose hook injector initialized");
	return true;
}

void DisableHooks()
{
	SetDriverShuttingDown(true);
	LOG("Disabling pose hooks");
	IHook::DestroyAll();
	g_poseHooksInstalled = false;
	MH_Uninitialize();
}