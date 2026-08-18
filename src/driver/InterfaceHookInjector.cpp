// SPDX-License-Identifier: AGPL-3.0-only

#include "Logging.h"
#include "Hooking.h"
#include "InterfaceHookInjector.h"
#include "ServerTrackedDeviceProvider.h"
#include "Main.h"

#include <atomic>
#include <cstdint>

// ---- pose-hook guards --------------------------------------------------------------------
// ACCEPTANCE (why the happy-path output is bit-identical):
//   * sizeof(DriverPose_t) == unPoseStructSize on every current SteamVR, so the size branch is
//     not taken;
//   * g_poseHookDepth is always 0 -- there is exactly one trampoline call site in the whole
//     driver, and nothing on the pose path calls back into TrackedDevicePoseUpdated;
//   * g_driverShuttingDown is false for the whole of play;
//   * MSVC x64 table-based EH is zero-cost on the non-throwing path, so the try/catch adds no
//     instructions to a frame that does not throw.
// Every branch added here is therefore unreachable in a normal session, and the pose that
// reaches HandleDevicePoseUpdated is the same pose, unmodified.

static std::atomic<bool> g_driverShuttingDown{ false };
static thread_local uint32_t g_poseHookDepth = 0;

// Scope guard for the re-entrancy depth. A counter raised and lowered by bare ++/-- is left
// pinned above zero for the life of the thread by anything that throws in between (the pose
// path allocates and calls into OpenVR, so std::bad_alloc is reachable) -- after which every
// pose on that thread silently takes the pass-through branch and the driver is inert with no
// log line and no counter moving.
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
// the last place anyone would look. Logged once -- this fires per pose callback, so an
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

// The pose path can throw, and an exception that unwinds out of the detour crosses into vrserver
// frames that have no handler -- std::terminate with the user in the headset. The detours
// therefore catch everything, publish the ORIGINAL pose (the local copy may be half-rewritten at
// the throw point), and log once. Per-frame, not sticky: the next callback runs the full path
// again.
static void LogPoseHookExceptionOnce()
{
	static std::atomic<bool> logged{ false };
	bool expected = false;
	if (logged.compare_exchange_strong(expected, true))
		LOG("Exception escaped the pose path; affected frames publish their ORIGINAL pose "
			"unmodified. Logged once.");
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

	// Depth guard against detour re-entry: the override path calls
	// GetRawTrackedDevicePoses, and any OpenVR call made from this detour could
	// re-enter it. Pass through before any lock is taken, so re-entry can never
	// recurse into a shared_lock the outer frame already holds.
	if (g_poseHookDepth > 0)
	{
		TrackedDevicePoseUpdatedHook005.originalFunc(_this, unWhichDevice, newPose, unPoseStructSize);
		return;
	}

	PoseHookDepthGuard depthGuard;
	//TRACE("ServerTrackedDeviceProvider::DetourTrackedDevicePoseUpdated(%d)", unWhichDevice);
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
	//TRACE("ServerTrackedDeviceProvider::DetourTrackedDevicePoseUpdated(%d)", unWhichDevice);
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
	// Same latch shape as IPCServer::Run()'s `stop` reset: this is a file-scope global, so a
	// same-process Cleanup->Init would otherwise come back up with the shutdown flag still set
	// and every pose passing straight through.
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