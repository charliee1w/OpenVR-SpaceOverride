// SPDX-License-Identifier: AGPL-3.0-only

#include "Logging.h"
#include "Hooking.h"
#include "InterfaceHookInjector.h"
#include "ServerTrackedDeviceProvider.h"
#include "Main.h"

#include <atomic>
#include <cstdint>

static std::atomic<bool> g_driverShuttingDown{ false };
static thread_local uint32_t g_poseHookDepth = 0;

// Scope guard for the re-entrancy depth. The counter used to be raised and lowered by bare
// ++/--, so anything that threw between them (CaptureWrite appends to a std::string, so
// std::bad_alloc is reachable on the pose path) left the depth pinned above zero for the life
// of that thread — after which every pose on it silently took the pass-through branch and the
// driver was inert with no log line and no counter moving.
struct PoseHookDepthGuard
{
	PoseHookDepthGuard() { ++g_poseHookDepth; }
	~PoseHookDepthGuard() { --g_poseHookDepth; }
	PoseHookDepthGuard(const PoseHookDepthGuard&) = delete;
	PoseHookDepthGuard& operator=(const PoseHookDepthGuard&) = delete;
};

// A DriverPose_t size mismatch means SteamVR changed the struct under us. Passing the pose
// through unmodified is the only safe response: the previous `return;` DROPPED it, so a future
// SteamVR would have produced total tracking loss for every device, silently, with the driver
// the last place anyone would look. Logged once — this fires per pose callback, so an
// unthrottled log would be the second failure.
static void LogPoseStructMismatchOnce(uint32_t got)
{
	static std::atomic<bool> logged{ false };
	bool expected = false;
	if (logged.compare_exchange_strong(expected, true))
		LOG("DriverPose_t size mismatch: SteamVR passed %u, this driver was built against %u. "
			"Passing all poses through UNMODIFIED — SpaceOverride is inert until rebuilt.",
			got, (uint32_t)sizeof(vr::DriverPose_t));
}

// The pose path can throw (the guard comment above names CaptureWrite's std::bad_alloc), and an
// exception that unwinds out of the detour crosses into vrserver frames that have no handler —
// std::terminate with the user in the headset. The detours therefore catch everything, publish
// the ORIGINAL pose (the local copy may be half-rewritten at the throw point), and log once.
// Per-frame, not sticky: the next callback runs the full path again.
static void LogPoseHookExceptionOnce()
{
	static std::atomic<bool> logged{ false };
	bool expected = false;
	if (logged.compare_exchange_strong(expected, true))
		LOG("Exception escaped the pose path; affected frames publish their ORIGINAL pose "
			"unmodified. Capture/diagnostics may be incomplete. Logged once.");
}

void SetDriverShuttingDown(bool shuttingDown)
{
	g_driverShuttingDown.store(shuttingDown, std::memory_order_release);
}

static Hook<void*(*)(void*, const char *, vr::EVRInitError *)>
	GetGenericInterfaceHook("IVRDriverContext::GetGenericInterface");

static Hook<void(*)(void*, uint32_t, const vr::DriverPose_t &, uint32_t)>
	TrackedDevicePoseUpdatedHook005("IVRServerDriverHost005::TrackedDevicePoseUpdated");

static Hook<void(*)(void*, uint32_t, const vr::DriverPose_t &, uint32_t)>
	TrackedDevicePoseUpdatedHook006("IVRServerDriverHost006::TrackedDevicePoseUpdated");

static void DetourTrackedDevicePoseUpdated005(void* _this, uint32_t unWhichDevice, const vr::DriverPose_t &newPose, uint32_t unPoseStructSize)
{
	if (g_driverShuttingDown.load(std::memory_order_acquire))
	{
		TrackedDevicePoseUpdatedHook005.originalFunc(_this, unWhichDevice, newPose, unPoseStructSize);
		return;
	}
	if (sizeof(vr::DriverPose_t) != unPoseStructSize)
	{
		LogPoseStructMismatchOnce(unPoseStructSize);
		TrackedDevicePoseUpdatedHook005.originalFunc(_this, unWhichDevice, newPose, unPoseStructSize);
		return;
	}

	// Depth guard against detour re-entry. The original need was a
	// GetRawTrackedDevicePoses call inside the override (since removed); the guard
	// stays because any OpenVR call made from this detour could re-enter it.
	if (g_poseHookDepth > 0)
	{
		TrackedDevicePoseUpdatedHook005.originalFunc(_this, unWhichDevice, newPose, unPoseStructSize);
		return;
	}

	PoseHookDepthGuard depthGuard;
	auto pose = newPose;
	bool publish;
	try
	{
		publish = g_server.HandleDevicePoseUpdated(unWhichDevice, pose);
	}
	catch (...)
	{
		LogPoseHookExceptionOnce();
		TrackedDevicePoseUpdatedHook005.originalFunc(_this, unWhichDevice, newPose, unPoseStructSize);
		return;
	}
	if (publish)
	{
		TrackedDevicePoseUpdatedHook005.originalFunc(_this, unWhichDevice, pose, unPoseStructSize);
	}
}

static void DetourTrackedDevicePoseUpdated006(void* _this, uint32_t unWhichDevice, const vr::DriverPose_t &newPose, uint32_t unPoseStructSize)
{
	if (g_driverShuttingDown.load(std::memory_order_acquire))
	{
		TrackedDevicePoseUpdatedHook006.originalFunc(_this, unWhichDevice, newPose, unPoseStructSize);
		return;
	}
	if (sizeof(vr::DriverPose_t) != unPoseStructSize)
	{
		LogPoseStructMismatchOnce(unPoseStructSize);
		TrackedDevicePoseUpdatedHook006.originalFunc(_this, unWhichDevice, newPose, unPoseStructSize);
		return;
	}

	if (g_poseHookDepth > 0)
	{
		TrackedDevicePoseUpdatedHook006.originalFunc(_this, unWhichDevice, newPose, unPoseStructSize);
		return;
	}

	PoseHookDepthGuard depthGuard;
	auto pose = newPose;
	bool publish;
	try
	{
		publish = g_server.HandleDevicePoseUpdated(unWhichDevice, pose);
	}
	catch (...)
	{
		LogPoseHookExceptionOnce();
		TrackedDevicePoseUpdatedHook006.originalFunc(_this, unWhichDevice, newPose, unPoseStructSize);
		return;
	}
	if (publish)
	{
		TrackedDevicePoseUpdatedHook006.originalFunc(_this, unWhichDevice, pose, unPoseStructSize);
	}
}

static void *DetourGetGenericInterface(void* _this, const char *pchInterfaceVersion, vr::EVRInitError *peError)
{
	TRACE("ServerTrackedDeviceProvider::DetourGetGenericInterface(%s)", pchInterfaceVersion);
	auto originalInterface = GetGenericInterfaceHook.originalFunc(_this, pchInterfaceVersion, peError);

	std::string iface(pchInterfaceVersion);
	if (iface == "IVRServerDriverHost_005")
	{
		if (!IHook::Exists(TrackedDevicePoseUpdatedHook005.name))
		{
			TrackedDevicePoseUpdatedHook005.CreateHookInObjectVTable(originalInterface, 1, &DetourTrackedDevicePoseUpdated005);
			IHook::Register(&TrackedDevicePoseUpdatedHook005);
		}
	}
	else if (iface == "IVRServerDriverHost_006")
	{
		if (!IHook::Exists(TrackedDevicePoseUpdatedHook006.name))
		{
			TrackedDevicePoseUpdatedHook006.CreateHookInObjectVTable(originalInterface, 1, &DetourTrackedDevicePoseUpdated006);
			IHook::Register(&TrackedDevicePoseUpdatedHook006);
		}
	}

	return originalInterface;
}

void InjectHooks(vr::IVRDriverContext *pDriverContext)
{
	g_driverShuttingDown.store(false, std::memory_order_release);

	auto err = MH_Initialize();
	if (err == MH_OK)
	{
		GetGenericInterfaceHook.CreateHookInObjectVTable(pDriverContext, 0, &DetourGetGenericInterface);
		IHook::Register(&GetGenericInterfaceHook);
	}
	else
	{
		LOG("MH_Initialize error: %s", MH_StatusToString(err));
	}
}

void DisableHooks()
{
	IHook::DestroyAll();
	MH_Uninitialize();
}
