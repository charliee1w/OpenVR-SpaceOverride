// SPDX-License-Identifier: AGPL-3.0-only

#include "ServerTrackedDeviceProvider.h"
#include "Logging.h"
#include "InterfaceHookInjector.h"
#include "PoseMath.h"
#include "CaptureFormat.h"

#include "Version.h"

#include <cmath>
#include <string>

using namespace pose_math;

namespace {

inline capture::CapQuat CapQ(const vr::HmdQuaternion_t& q)
{
	return { q.w, q.x, q.y, q.z };
}

inline capture::CapVec3 CapV(const vr::HmdVector3d_t& v)
{
	return { v.v[0], v.v[1], v.v[2] };
}

inline capture::CapVec3 CapV3(const double p[3])
{
	return { p[0], p[1], p[2] };
}

inline bool PoseConfigEqual(const pose_est::PoseConfig& a, const pose_est::PoseConfig& b)
{
	auto qeq = [](const vr::HmdQuaternion_t& x, const vr::HmdQuaternion_t& y) {
		return x.w == y.w && x.x == y.x && x.y == y.y && x.z == y.z;
	};
	auto veq = [](const vr::HmdVector3d_t& x, const vr::HmdVector3d_t& y) {
		return x.v[0] == y.v[0] && x.v[1] == y.v[1] && x.v[2] == y.v[2];
	};
	return a.fusionMode == b.fusionMode
		&& a.native == b.native
		&& a.slamFallback == b.slamFallback
		&& a.enableAngularVelocity == b.enableAngularVelocity
		&& a.headFilterEnabled == b.headFilterEnabled
		&& a.predictionTime == b.predictionTime
		&& a.calibrationScale == b.calibrationScale
		&& a.hmdScale == b.hmdScale
		&& qeq(a.offsetRotation, b.offsetRotation)
		&& veq(a.offsetTranslation, b.offsetTranslation)
		&& qeq(a.calibrationRotation, b.calibrationRotation)
		&& veq(a.calibrationTranslation, b.calibrationTranslation);
}

// ---- IPC input boundary ------------------------------------------------------------------
// AUDIT.md listed "values arriving over IPC are unvalidated" as a never-closed gap: the pipe
// carries whatever a local process writes, the Request union is uninitialised client-side, and
// the pipe has a default DACL. These setters wrote straight into the config a pose callback
// reads.
//
// Policy is REJECT AND LOG, never silently clamp — a clamped bad value produces mis-tracking
// that looks like a calibration problem, which is the most expensive kind of bug this project
// has. Note every test is written so NaN FAILS it: `>=`/`<=` are false for NaN, so the checks
// are phrased positively rather than as negated bounds.

inline bool FiniteQuatUnit(const vr::HmdQuaternion_t& q)
{
	if (!std::isfinite(q.w) || !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z))
		return false;
	const double n = sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
	return fabs(1.0 - n) <= 1e-3;
}

inline bool FiniteVecWithin(const vr::HmdVector3d_t& v, double maxAbs)
{
	if (!std::isfinite(v.v[0]) || !std::isfinite(v.v[1]) || !std::isfinite(v.v[2]))
		return false;
	return fabs(v.v[0]) <= maxAbs && fabs(v.v[1]) <= maxAbs && fabs(v.v[2]) <= maxAbs;
}

inline bool ScaleInRange(double s)
{
	return std::isfinite(s) && s >= 0.5 && s <= 2.0;
}

// One-Euro parameters. NaN previously survived `p.minCutoff < 0.01 ? 0.01 : p.minCutoff`
// because the comparison is false for NaN, so the NaN was stored and propagated into alpha().
inline double SanitizeOneEuro(double v, double lo, double fallback)
{
	if (!std::isfinite(v))
		return fallback;
	return v < lo ? lo : v;
}

} // namespace

// Every filter and bound on the pose path that can be switched at runtime, in one table.
//
// Init() and RunFrame() both walk this list, so a new filter is wired into the startup read AND
// the live poll by adding one row -- the two cannot drift apart, which is the failure this table
// exists to prevent. Keys live under driver_spaceoverride/ in steamvr.vrsettings and are polled
// once a second, so every one of these takes effect without a SteamVR restart.
//
// The defaults are not uniform, deliberately:
//
//   trackerFilter   OFF. KalmanFilterXYZ on the tracker position. In OVERRIDE mode that filtered
//                   position IS the published head pose, so this is view lag. Shipped constants
//                   (Q=2.5e-7, R=1.0e-5, A=4.0) settle to K=0.146 -- ~125 ms at a 50 Hz pose
//                   rate -- and the adaptive term only opens past a 2*sqrt(R) = 6.3 mm per-frame
//                   dead-band. A 20 degree nod moves a helmet-top tracker ~7.7 cm, ~3.8 mm per
//                   frame: under the dead-band, so ordinary nodding takes the full lag while
//                   large motions punch through. User-reported 2026-08-15. What it suppresses is
//                   sub-millimetre, and HW1's second base station halved that while leaving the
//                   cost unchanged. AUDIT finding 12 flagged it; N4-e step (1) is to drop it.
//   driftFilter     ON. One-Euro on the correction transform. Smooths the correction, not head
//                   motion, so it costs no view latency -- but off lets observation noise into
//                   the published transform.
//   headVelFilter   ON. One-Euro on the PUBLISHED angular velocity only. The pose is identical
//                   either way; only games that extrapolate from vecAngularVelocity feel it.
//   publishSlew     ON. SAFETY BOUND. GatePublish's jump hold and reconvergence slew. Off means
//                   a far candidate is snapped to instead of approached -- the teleport the gate
//                   exists to prevent. Engages only while reconverging, so it is free at rest.
//   corrRateLimit   ON. SAFETY BOUND. The N1-h correction rate limit. Off restores the measured
//                   17.87-degree single-frame view yaw step on covariance reset. No real head
//                   motion passes through the correction channel, so this adds latency to
//                   nothing the user does.
const ServerTrackedDeviceProvider::FilterToggle
ServerTrackedDeviceProvider::kFilterToggles[5] = {
	{ "trackerFilter", &ServerTrackedDeviceProvider::trackerFilterEnabled, false },
	{ "driftFilter",   &ServerTrackedDeviceProvider::driftFilterEnabled,   true  },
	{ "headVelFilter", &ServerTrackedDeviceProvider::headVelFilterEnabled, true  },
	{ "publishSlew",   &ServerTrackedDeviceProvider::publishSlewEnabled,   true  },
	{ "corrRateLimit", &ServerTrackedDeviceProvider::corrRateLimitEnabled, true  },
};

vr::EVRInitError ServerTrackedDeviceProvider::Init(vr::IVRDriverContext* pDriverContext)
{
	TRACE("ServerTrackedDeviceProvider::Init()");
	VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);

	OpenLogFile();
	LOG("OpenVR-SpaceOverride " SPACECAL_VERSION_STRING " loaded");
	LOG("auto-log session: %s", GetSessionLogPath()[0] ? GetSessionLogPath() : "(cwd fallback)");
	LOG("auto-log: events=tracker_ok/bad, speed_reject, HMD_JUMP/HOLD, last_good_hold, 60s heartbeat");
	LOG("gates: isfinite, pre-publish jump + reconverge<=3m/s, last-good<=150ms, tracker-hook-cache (no GetRaw while quash)");

	for (uint32_t i = 0; i < vr::k_unMaxTrackedDeviceCount; i++)
	{
		transforms[i].enabled = false;
		transforms[i].translation = { 0, 0, 0 };
		transforms[i].rotation = { 1, 0, 0, 0 };
		transforms[i].scale = 1.0;
	}
	memset(slamSync, 0, sizeof slamSync);
	diag = pose_est::PoseDiag{};
	poseState = pose_est::PoseState{};
	sharedDrift = SharedDrift{};
	cachedTracker = CachedTrackerPose{};
	displayHzQueried = false;
	cachedDisplayHz = 90.0;

	{
		vr::EVRSettingsError serr = vr::VRSettingsError_None;
		bool f = vr::VRSettings()->GetBool("driver_spaceoverride", "fusionMode", &serr);
		fusionMode = (serr == vr::VRSettingsError_None) ? f : false;
		serr = vr::VRSettingsError_None;
		bool d = vr::VRSettings()->GetBool("driver_spaceoverride", "fusionDiag", &serr);
		fusionDiag = (serr == vr::VRSettingsError_None) ? d : false;

		// Every switchable filter/bound, read through one table so Init and RunFrame cannot
		// drift apart. See kFilterToggles above for what each one does and why its default is
		// what it is.
		for (const FilterToggle &t : kFilterToggles)
		{
			serr = vr::VRSettingsError_None;
			const bool v = vr::VRSettings()->GetBool("driver_spaceoverride", t.key, &serr);
			this->*(t.field) = (serr == vr::VRSettingsError_None) ? v : t.defaultOn;
		}

	}
	for (const FilterToggle &t : kFilterToggles)
		LOG("filter %s: %s%s", t.key, this->*(t.field) ? "ON" : "OFF",
			(this->*(t.field) == t.defaultOn) ? " (default)" : " (overridden)");
	LOG("mode: %s", fusionMode
		? "FUSION (SLAM source, tracker observes correction, slam-step cancel)"
		: "OVERRIDE (tracker source, classic)");

	if (fusionMode && fusionDiag)
	{
		// Epoch first, then open — a frame that sees the sink open must not log against a stale
		// epoch (V6).
		LARGE_INTEGER _ds{}; QueryPerformanceCounter(&_ds); poseState.diagStart = _ds.QuadPart;
		poseState.diagLastWrite = 0;
		OpenDiagCsv();
		SetCaptureEnabled(true);
		LOG("diagnostics enabled: fusion_diag %s, capture %s",
			DiagCsvOpen() ? "open" : "FAILED to open",
			CaptureEnabled() ? "open" : "FAILED to open");
	}

	poseState.drift.rotationFilter.params = { 3.0, 1.3, 0.6 };
	poseState.drift.translationFilter.params = { 3.0, 1.3, 0.6 };
	poseState.headFilter.rotationFilter.params = { 5.0, 0.8, 1.0 };
	poseState.headFilter.translationFilter.params = { 5.0, 0.8, 1.0 };
	poseState.headVel.filter.params = { 8.0, 1.0, 1.0 };

	poseState.trackerFilter.translation.SetQ(2.5e-7);
	poseState.trackerFilter.translation.SetR(1.0e-5);
	poseState.trackerFilter.translation.SetAdaptiveGain(4.0);

	InjectHooks(pDriverContext);
	server.Run();

	return vr::VRInitError_None;
}

void ServerTrackedDeviceProvider::RunFrame()
{
	LARGE_INTEGER now{}, freq{};
	QueryPerformanceCounter(&now);
	QueryPerformanceFrequency(&freq);
	if (settingsLastPoll.QuadPart != 0)
	{
		double since = (now.QuadPart - settingsLastPoll.QuadPart) / (double)freq.QuadPart;
		if (since < 1.0)
			return;
	}
	settingsLastPoll = now;

	vr::EVRSettingsError serr = vr::VRSettingsError_None;
	const bool wantFusion = vr::VRSettings()->GetBool("driver_spaceoverride", "fusionMode", &serr);
	if (serr == vr::VRSettingsError_None && wantFusion != fusionMode)
	{
		{
			std::unique_lock<std::shared_mutex> lock(configMutex);
			fusionMode = wantFusion;
			ResetEstimators(/*clearLastGood=*/false, /*clearSlamSync=*/false);
		}
		LOG("mode switched -> %s (live, via overlay)", fusionMode ? "FUSION" : "OVERRIDE");
	}

	for (const FilterToggle &t : kFilterToggles)
	{
		serr = vr::VRSettingsError_None;
		const bool want = vr::VRSettings()->GetBool("driver_spaceoverride", t.key, &serr);
		if (serr != vr::VRSettingsError_None || want == this->*(t.field))
			continue;

		{
			// Same lock discipline as the fusion toggle: these feed PoseConfig, which a pose
			// callback reads under the shared lock, so flipping one bare would tear an in-flight
			// frame. Reset the filter states too -- re-enabling a filter that still holds the
			// state from before it was bypassed would step the pose by however far the head
			// moved in between.
			std::unique_lock<std::shared_mutex> lock(configMutex);
			this->*(t.field) = want;
			poseState.trackerFilter.reset();
			poseState.headVel.reset();
		}
		LOG("filter %s -> %s (live, via settings)", t.key, want ? "ON" : "OFF");
	}

	serr = vr::VRSettingsError_None;
	const bool wantDiag = vr::VRSettings()->GetBool("driver_spaceoverride", "fusionDiag", &serr);
	if (serr == vr::VRSettingsError_None && wantDiag != fusionDiag)
	{
		fusionDiag = wantDiag;
		LOG("fusion diagnostic CSV -> %s", fusionDiag ? "on" : "off");
	}

	{
		const bool wantCsv = fusionMode && fusionDiag;
		if (wantCsv != DiagCsvOpen())
		{
			if (wantCsv)
			{
				// Stamp the epoch BEFORE the sink exists. DiagCsvOpen() is what a pose frame
				// tests, so opening first leaves a window in which a frame logs a row while
				// diagStart still holds the *previous* open's counter — the first rows after a
				// mid-session re-enable then carry a tms offset from the old epoch.
				// Capture epoch (configSeq / frameIndex) is NOT reset here: OpenDiagCsv can fail
				// while the .sor stays open, and re-entering this branch every poll would then
				// restart frameIndex at 0 inside one file and spam duplicate Rec_Config.
				std::unique_lock<std::shared_mutex> lock(configMutex);
				LARGE_INTEGER _ds{}; QueryPerformanceCounter(&_ds); poseState.diagStart = _ds.QuadPart;
				poseState.diagLastWrite = 0;
			}
			if (wantCsv)
				OpenDiagCsv();
			else
				CloseDiagCsv();
			LOG("fusion diagnostic CSV file -> %s", wantCsv ? "open" : "closed");
		}
		// Desired-state, idempotent, unconditional. Deliberately NOT inside the transition test
		// above: that test keys off DiagCsvOpen(), and anything that keys off a *different*
		// sink's state is how a second sink ends up half-wired.
		//
		// The file-local counters (configSeq/frameIndex) are NOT reset here: the new file is
		// already published by the time SetCaptureEnabled returns, so an HMD pose frame could
		// land between that and this thread taking configMutex — writing a Rec_Frame whose
		// counters still belonged to the previous file. The pose path resets them itself, keyed
		// on CaptureGeneration(), on the only thread that writes them.
		SetCaptureEnabled(wantCsv);
	}
}

void ServerTrackedDeviceProvider::EnterStandby()
{
	LOG("EnterStandby");
}

void ServerTrackedDeviceProvider::LeaveStandby()
{
	// Everything time-derived is stale across a standby of unknown length. Keep lastGood: it is
	// bounded by its own 150 ms age test, and clearing it would remove the only reference the
	// publish gate has for the first frame back — which is exactly when a jump is most likely.
	// Keep the slamSync set too; membership is a property of the room, not of the session.
	{
		std::unique_lock<std::shared_mutex> lock(configMutex);
		ResetEstimators(/*clearLastGood=*/false, /*clearSlamSync=*/false);
		diag.trackerStateKnown = false;
	}
	LOG("LeaveStandby — estimators reset (tracker cache, EKF, filters, drift)");
}

void ServerTrackedDeviceProvider::Cleanup()
{
	// Same gate as the heartbeat (fusionMode && !native): bare fusionMode claims mode=F for a
	// session where Discard-Calibrated-Offset left the EKF inert the whole time.
	const char sessionMode = (fusionMode && !hmdTracker.native) ? 'F' : 'O';
	LOG("session stats: frames=%llu ok=%llu bad=%llu jumps=%llu jump_holds=%llu last_good_holds=%llu speed_rej=%llu fallback=%llu nonfinite=%llu play_primed=%d mode=%c slam_steps=%llu absorbed=%.2fm",
		(unsigned long long)diag.frames,
		(unsigned long long)diag.trackerOkFrames,
		(unsigned long long)diag.trackerBadFrames,
		(unsigned long long)diag.jumpEvents,
		(unsigned long long)diag.jumpHolds,
		(unsigned long long)diag.lastGoodHolds,
		(unsigned long long)diag.speedRejects,
		(unsigned long long)diag.fallbackFrames,
		(unsigned long long)diag.nonFiniteDrops,
		diag.playPrimed ? 1 : 0,
		sessionMode,
		(unsigned long long)diag.slamSteps,
		diag.slamStepSumM);
	LOG("OpenVR-SpaceOverride unloading");
	SetDriverShuttingDown(true);
	server.Stop();
	DisableHooks();
	VR_CLEANUP_SERVER_DRIVER_CONTEXT();
	LOG("OpenVR-SpaceOverride unloaded");
	SetCaptureEnabled(false);
	CloseDiagCsv();
	CloseLogFile();
}

void ServerTrackedDeviceProvider::SetDeviceTransform(const protocol::SetDeviceTransform& newTransform)
{
	if (newTransform.openVRID >= vr::k_unMaxTrackedDeviceCount)
		return;

	// Validate before taking the lock, and reject the whole message rather than applying the
	// good fields of a bad one. tf.scale multiplies vecPosition for every device that carries a
	// transform, so a NaN or zero here corrupts poses the driver does not otherwise touch.
	if (newTransform.updateTranslation && !FiniteVecWithin(newTransform.translation, 10.0))
	{
		LOG("IPC REJECT SetDeviceTransform id=%u: translation not finite / >10 m", newTransform.openVRID);
		return;
	}
	if (newTransform.updateRotation && !FiniteQuatUnit(newTransform.rotation))
	{
		LOG("IPC REJECT SetDeviceTransform id=%u: rotation not a finite unit quaternion", newTransform.openVRID);
		return;
	}
	if (newTransform.updateScale && !ScaleInRange(newTransform.scale))
	{
		LOG("IPC REJECT SetDeviceTransform id=%u: scale %.6f outside [0.5, 2.0]",
			newTransform.openVRID, newTransform.scale);
		return;
	}

	std::unique_lock<std::shared_mutex> lock(configMutex);

	auto& tf = transforms[newTransform.openVRID];
	tf.enabled = newTransform.enabled;

	if (newTransform.updateTranslation)
		tf.translation = newTransform.translation;

	if (newTransform.updateRotation)
		tf.rotation = newTransform.rotation;

	if (newTransform.updateScale)
		tf.scale = newTransform.scale;
}

void ServerTrackedDeviceProvider::SetHmdTracker(const protocol::SetHmdTracker& cmd)
{
	if (cmd.hmdID >= vr::k_unMaxTrackedDeviceCount)
		return;

	if (cmd.enabled && cmd.trackerID >= vr::k_unMaxTrackedDeviceCount)
		return;

	// This message defines the entire calibration the pose path runs on. Reject it whole if any
	// field is unusable — a partially-applied calibration is worse than none, because the driver
	// keeps tracking and the error looks like a bad calibration rather than a bad message.
	if (!FiniteQuatUnit(cmd.offsetRotation) || !FiniteQuatUnit(cmd.calibrationRotation))
	{
		LOG("IPC REJECT SetHmdTracker: offset/calibration rotation is not a finite unit quaternion");
		return;
	}
	if (!FiniteVecWithin(cmd.offsetTranslation, 10.0) || !FiniteVecWithin(cmd.calibrationTranslation, 10.0))
	{
		LOG("IPC REJECT SetHmdTracker: offset/calibration translation not finite or >10 m");
		return;
	}
	if (!ScaleInRange(cmd.calibrationScale) || !ScaleInRange(cmd.hmdScale))
	{
		LOG("IPC REJECT SetHmdTracker: calibrationScale %.6f / hmdScale %.6f outside [0.5, 2.0]",
			cmd.calibrationScale, cmd.hmdScale);
		return;
	}
	if (!std::isfinite(cmd.predictionTime))
	{
		LOG("IPC REJECT SetHmdTracker: predictionTime not finite");
		return;
	}

	// Both are in range by the check above; the ternaries are kept so the applied value is still
	// explicit at the point of use.
	const double newScale = cmd.calibrationScale > 0.0 ? cmd.calibrationScale : 1.0;
	const double newHmdScale = cmd.hmdScale > 0.0 ? cmd.hmdScale : 1.0;

	std::unique_lock<std::shared_mutex> lock(configMutex);

	const bool trackerChanged = hmdTracker.trackerID != cmd.trackerID;

	const bool changed =
		hmdTracker.enabled != cmd.enabled
		|| hmdTracker.native != cmd.native
		|| hmdTracker.slamFallback != cmd.slamFallback
		|| hmdTracker.enableAngularVelocity != cmd.enableAngularVelocity
		|| hmdTracker.predictionTime != cmd.predictionTime
		|| hmdTracker.hmdID != cmd.hmdID
		|| hmdTracker.trackerID != cmd.trackerID
		|| hmdTracker.calibrationScale != newScale
		|| hmdTracker.hmdScale != newHmdScale
		|| hmdTracker.offsetRotation.w != cmd.offsetRotation.w
		|| hmdTracker.offsetRotation.x != cmd.offsetRotation.x
		|| hmdTracker.offsetRotation.y != cmd.offsetRotation.y
		|| hmdTracker.offsetRotation.z != cmd.offsetRotation.z
		|| hmdTracker.offsetTranslation.v[0] != cmd.offsetTranslation.v[0]
		|| hmdTracker.offsetTranslation.v[1] != cmd.offsetTranslation.v[1]
		|| hmdTracker.offsetTranslation.v[2] != cmd.offsetTranslation.v[2]
		|| hmdTracker.calibrationRotation.w != cmd.calibrationRotation.w
		|| hmdTracker.calibrationRotation.x != cmd.calibrationRotation.x
		|| hmdTracker.calibrationRotation.y != cmd.calibrationRotation.y
		|| hmdTracker.calibrationRotation.z != cmd.calibrationRotation.z
		|| hmdTracker.calibrationTranslation.v[0] != cmd.calibrationTranslation.v[0]
		|| hmdTracker.calibrationTranslation.v[1] != cmd.calibrationTranslation.v[1]
		|| hmdTracker.calibrationTranslation.v[2] != cmd.calibrationTranslation.v[2];

	hmdTracker.enabled = cmd.enabled;
	hmdTracker.native = cmd.native;
	hmdTracker.slamFallback = cmd.slamFallback;
	hmdTracker.enableAngularVelocity = cmd.enableAngularVelocity;
	hmdTracker.predictionTime = cmd.predictionTime;
	hmdTracker.hmdID = cmd.hmdID;
	hmdTracker.trackerID = cmd.trackerID;
	hmdTracker.offsetRotation = cmd.offsetRotation;
	hmdTracker.offsetTranslation = cmd.offsetTranslation;
	hmdTracker.calibrationRotation = cmd.calibrationRotation;
	hmdTracker.calibrationTranslation = cmd.calibrationTranslation;
	hmdTracker.calibrationScale = newScale;
	hmdTracker.hmdScale = newHmdScale;

	if (!cmd.enabled)
	{
		ResetEstimators(/*clearLastGood=*/true, /*clearSlamSync=*/true);
		diag.trackerStateKnown = false;
		diag.playPrimed = false;
	}
	else if (trackerChanged)
	{
		ResetEstimators(/*clearLastGood=*/true, /*clearSlamSync=*/false);
	}

	lock.unlock();

	if (changed)
	{
		LOG("SetHmdTracker enabled=%d native=%d slamFallback=%d pred=%.2f hmd=%u tracker=%u scale=%.5f hmdScale=%.5f eAngVel=%d",
			cmd.enabled ? 1 : 0,
			cmd.native ? 1 : 0,
			cmd.slamFallback ? 1 : 0,
			cmd.predictionTime,
			cmd.hmdID,
			cmd.trackerID,
			newScale,
			newHmdScale,
			cmd.enableAngularVelocity ? 1 : 0);
	}
}

void ServerTrackedDeviceProvider::SetSlamSync(const protocol::SetSlamSync& cmd)
{
	if (cmd.openVRID < vr::k_unMaxTrackedDeviceCount)
	{
		bool changed = false;
		std::string members;
		{
			std::unique_lock<std::shared_mutex> lock(configMutex);
			changed = (slamSync[cmd.openVRID] != cmd.enabled);
			slamSync[cmd.openVRID] = cmd.enabled;
			if (changed)
			{
				for (uint32_t i = 0; i < vr::k_unMaxTrackedDeviceCount; i++)
				{
					if (!slamSync[i])
						continue;
					char idbuf[16];
					snprintf(idbuf, sizeof idbuf, "%s%u", members.empty() ? "" : ",", i);
					members += idbuf;
				}
			}
		}
		// The slamSync set was never logged, which left "body drifted relative to view"
		// undiagnosable: a lighthouse device wrongly in this set gets the SLAM correction and
		// slamScale applied on top of a pose that was already in lighthouse space, so it walks
		// as `corr` moves. On this machine only genuinely SLAM-space devices belong here —
		// Index controllers and Tundra pucks are lighthouse-native and should NOT appear.
		if (changed)
			LOG("SetSlamSync id=%u enabled=%d -> sync set = {%s}",
				cmd.openVRID, cmd.enabled ? 1 : 0,
				members.empty() ? "none" : members.c_str());
	}
}

void ServerTrackedDeviceProvider::SetOneEuro(const protocol::SetOneEuro& cmd)
{
	auto toParams = [](const protocol::OneEuroParams& p) {
		oneeuro::Params out;
		out.minCutoff = SanitizeOneEuro(p.minCutoff, 0.01, 1.0);
		out.beta = SanitizeOneEuro(p.beta, 0.0, 0.0);
		out.dCutoff = SanitizeOneEuro(p.dCutoff, 0.01, 1.0);
		return out;
	};

	std::unique_lock<std::shared_mutex> lock(configMutex);

	poseState.headFilter.rotationFilter.params = toParams(cmd.head);
	poseState.headFilter.translationFilter.params = toParams(cmd.head);
	poseState.drift.rotationFilter.params = toParams(cmd.drift);
	poseState.drift.translationFilter.params = toParams(cmd.drift);

	if (poseState.headFilter.enabled && !cmd.headEnabled)
		poseState.headFilter.reset();
	poseState.headFilter.enabled = cmd.headEnabled;
}

void ServerTrackedDeviceProvider::ResetEstimators(bool clearLastGood, bool clearSlamSync)
{
	// Caller holds configMutex exclusive.
	poseState.Reset(clearLastGood);
	{
		std::lock_guard<std::mutex> driftLock(driftMutex);
		sharedDrift = SharedDrift{};
	}
	{
		std::lock_guard<std::mutex> cacheLock(trackerCacheMutex);
		cachedTracker.valid = false;
	}
	if (clearLastGood)
		diag.haveLastHmdPos = false;
	if (clearSlamSync)
		memset(slamSync, 0, sizeof slamSync);
}

void ServerTrackedDeviceProvider::PublishDrift()
{
	std::lock_guard<std::mutex> lock(driftMutex);
	sharedDrift.valid = poseState.drift.valid;
	sharedDrift.rotation = poseState.drift.rotation;
	sharedDrift.translation = poseState.drift.translation;
}

bool ServerTrackedDeviceProvider::ApplySharedDrift(vr::DriverPose_t& pose)
{
	vr::HmdQuaternion_t driftRotation;
	vr::HmdVector3d_t driftTranslation;
	{
		std::lock_guard<std::mutex> lock(driftMutex);
		if (!sharedDrift.valid)
			return false;
		driftRotation = sharedDrift.rotation;
		driftTranslation = sharedDrift.translation;
	}

	const double slamScale = pose_math::SlamToCorrectedScale(
		hmdTracker.hmdScale, hmdTracker.calibrationScale, hmdTracker.native);

	pose.qWorldFromDriverRotation = quaternionNormalize(driftRotation * pose.qWorldFromDriverRotation);

	pose.vecPosition[0] *= slamScale;
	pose.vecPosition[1] *= slamScale;
	pose.vecPosition[2] *= slamScale;

	pose.vecDriverFromHeadTranslation[0] *= slamScale;
	pose.vecDriverFromHeadTranslation[1] *= slamScale;
	pose.vecDriverFromHeadTranslation[2] *= slamScale;

	double scaledTranslation[3] = {
		pose.vecWorldFromDriverTranslation[0] * slamScale,
		pose.vecWorldFromDriverTranslation[1] * slamScale,
		pose.vecWorldFromDriverTranslation[2] * slamScale
	};
	vr::HmdVector3d_t rotatedTranslation = quaternionRotateVector(driftRotation, scaledTranslation);
	pose.vecWorldFromDriverTranslation[0] = rotatedTranslation.v[0] + driftTranslation.v[0];
	pose.vecWorldFromDriverTranslation[1] = rotatedTranslation.v[1] + driftTranslation.v[1];
	pose.vecWorldFromDriverTranslation[2] = rotatedTranslation.v[2] + driftTranslation.v[2];
	return true;
}

double ServerTrackedDeviceProvider::GetCachedDisplayHz(uint32_t hmdOpenVRID)
{
	LARGE_INTEGER now{}, freq{};
	QueryPerformanceCounter(&now);
	QueryPerformanceFrequency(&freq);

	bool need = !displayHzQueried;
	if (displayHzQueried)
	{
		double elapsed = (now.QuadPart - displayHzLastQuery.QuadPart) / (double)freq.QuadPart;
		if (elapsed >= 1.0)
			need = true;
	}

	if (need && hmdOpenVRID < vr::k_unMaxTrackedDeviceCount)
	{
		vr::PropertyContainerHandle_t container = vr::VRProperties()->TrackedDeviceToPropertyContainer(hmdOpenVRID);
		double hz = vr::VRProperties()->GetFloatProperty(container, vr::Prop_DisplayFrequency_Float);
		if (hz >= 1.0)
			cachedDisplayHz = hz;
		else if (!displayHzQueried)
			cachedDisplayHz = 90.0;
		displayHzLastQuery = now;
		displayHzQueried = true;
	}

	return cachedDisplayHz > 1.0 ? cachedDisplayHz : 90.0;
}

// Names a device, once per capture FILE. Rec_Device carries only the OpenVR enumeration index,
// which is assigned in connection order: it is not stable across restarts and means nothing to a
// reader. The first real capture held 16 devices that could not be told apart — not even a base
// station from a foot puck — which is what blocked B6 and the body-drift question (N2-d(c)).
//
// Keyed on CaptureGeneration(), not a bool: re-opening the sink starts a NEW file, and that file
// must describe its own devices rather than depend on records written into the previous one.
static void CaptureDeviceInfoOnce(uint32_t openVRID, capture::DeviceRole role, int64_t clockNow)
{
	const uint32_t gen = CaptureGeneration();
	if (gen == 0)
		return;                  // sink closed; nothing to describe

	static std::mutex infoMutex;
	static uint32_t infoGen[vr::k_unMaxTrackedDeviceCount] = {};
	{
		std::lock_guard<std::mutex> lock(infoMutex);
		if (infoGen[openVRID] == gen)
			return;
		infoGen[openVRID] = gen;
	}

	capture::RecDeviceInfoPayload rec{};
	rec.clockNow = clockNow;
	rec.deviceId = openVRID;
	rec.role = (uint8_t)role;

	// PROPERTY reads, not GetRaw* pose reads — the C11 quash invariant is about the latter and
	// does not apply here. Runs once per device per file, i.e. far rarer than the 1 Hz property
	// read GetCachedDisplayHz already performs on this same thread.
	if (vr::VRProperties())
	{
		const vr::PropertyContainerHandle_t container =
			vr::VRProperties()->TrackedDeviceToPropertyContainer(openVRID);
		vr::ETrackedPropertyError err = vr::TrackedProp_Success;
		rec.deviceClass = vr::VRProperties()->GetInt32Property(container, vr::Prop_DeviceClass_Int32, &err);
		if (err != vr::TrackedProp_Success)
			rec.deviceClass = 0;
		vr::VRProperties()->GetStringProperty(container, vr::Prop_SerialNumber_String,
			rec.serial, (uint32_t)sizeof rec.serial);
		vr::VRProperties()->GetStringProperty(container, vr::Prop_ModelNumber_String,
			rec.model, (uint32_t)sizeof rec.model);
	}
	// A serial longer than the field is truncated, not left unterminated.
	rec.serial[sizeof rec.serial - 1] = 0;
	rec.model[sizeof rec.model - 1] = 0;

	CaptureWrite(capture::Rec_DeviceInfo, &rec, sizeof rec);
}

// Emits one Rec_Device: a device's LIGHTHOUSE world pose, composed exactly the way
// CacheTrackerWorldPose does, PRE-correction. Generic over device class — a new kind of tracked
// device needs no new code here, it just appears in the stream with its own id and role.
//
// Log-only: nothing in the driver reads the capture, so this cannot affect tracking.
// Throttled per device (~20 Hz); a full FBT set at 90-120 Hz would otherwise dominate the file
// while telling us nothing extra about a slowly-drifting relationship.
static void CaptureDevicePose(uint32_t openVRID, capture::DeviceRole role,
	const vr::DriverPose_t& pose)
{
	// Sink closed is the normal-play case: bail before the throttle mutex, the QPC pair and
	// ComposeWorldPose. Without this, every device's pose callback contended on one process-wide
	// mutex (and took g_captureMutex inside CaptureWrite) to discover capture was off.
	if (CaptureGeneration() == 0)
		return;

	if (openVRID >= vr::k_unMaxTrackedDeviceCount)
		return;
	if (!pose.poseIsValid || !pose.deviceIsConnected)
		return;
	if (pose.result != vr::TrackingResult_Running_OK
		&& pose.result != vr::TrackingResult_Running_OutOfRange)
		return;

	static std::mutex throttleMutex;
	static int64_t lastTick[vr::k_unMaxTrackedDeviceCount] = {};

	LARGE_INTEGER nowLi{}, freqLi{};
	QueryPerformanceCounter(&nowLi);
	QueryPerformanceFrequency(&freqLi);
	if (freqLi.QuadPart <= 0)
		return;

	{
		std::lock_guard<std::mutex> lock(throttleMutex);
		const int64_t prev = lastTick[openVRID];
		if (prev != 0 && (double)(nowLi.QuadPart - prev) / (double)freqLi.QuadPart < 0.05)
			return;
		lastTick[openVRID] = nowLi.QuadPart;
	}

	// Same composition as the publish gate / tracker cache / HmdInput — one rule for world space.
	vr::HmdQuaternion_t worldRot{};
	double pos[3] = {};
	ComposeWorldPose(pose, worldRot, pos);
	if (!IsFiniteQuat(worldRot) || !IsFiniteVec3(pos))
		return;

	capture::RecDevicePayload rec{};
	rec.clockNow = nowLi.QuadPart;
	rec.deviceId = openVRID;
	rec.role = (uint8_t)role;
	rec.poseIsValid = 1;
	rec.result = (int32_t)pose.result;
	rec.rotation = { worldRot.w, worldRot.x, worldRot.y, worldRot.z };
	rec.position = { pos[0], pos[1], pos[2] };

	// Always ahead of this device's first row in the file, so a reader can name the id before it
	// has to interpret a pose for it.
	CaptureDeviceInfoOnce(openVRID, role, nowLi.QuadPart);
	CaptureWrite(capture::Rec_Device, &rec, sizeof rec);
}

void ServerTrackedDeviceProvider::CacheTrackerWorldPose(const vr::DriverPose_t& pose)
{
	auto invalidate = [this]() {
		std::lock_guard<std::mutex> lock(trackerCacheMutex);
		cachedTracker.valid = false;
	};

	if (!pose.poseIsValid || !pose.deviceIsConnected)
	{
		invalidate();
		return;
	}
	if (pose.result != vr::TrackingResult_Running_OK
		&& pose.result != vr::TrackingResult_Running_OutOfRange)
	{
		invalidate();
		return;
	}

	vr::HmdQuaternion_t worldRot{};
	double pos[3] = {};
	ComposeWorldPose(pose, worldRot, pos);
	if (!IsFiniteQuat(worldRot) || !IsFiniteVec3(pos))
	{
		invalidate();
		return;
	}

	if (fabs(pos[1]) > 100.0)
	{
		invalidate();
		return;
	}

	double vel[3] = {
		pose.vecVelocity[0], pose.vecVelocity[1], pose.vecVelocity[2]
	};
	double ang[3] = {
		pose.vecAngularVelocity[0], pose.vecAngularVelocity[1], pose.vecAngularVelocity[2]
	};
	const double measuredSpeed = std::isfinite(vel[0]) && std::isfinite(vel[1]) && std::isfinite(vel[2])
		? sqrt(vel[0] * vel[0] + vel[1] * vel[1] + vel[2] * vel[2])
		: 0.0;

	SanitizeVec3Velocity(vel, kMaxPlausibleHeadSpeed);
	SanitizeVec3Velocity(ang, 20.0);

	// B6 capture: the head tracker's own lighthouse world pose, same frame and same file as
	// the body rows, so an FBT-implied head pose can be differenced against it offline.
	CaptureDevicePose(hmdTracker.trackerID, capture::Role_Head, pose);

	// ---------------------------------------------------------------------------------
	// R9 diagnostic — which frame is vecAngularVelocity in?  LOG ONLY: nothing below this
	// block reads its results, and the pose path is untouched.
	//
	// R9 is CLOSED for the lighthouse tracker (device-local; see Agents.md). The sampler is
	// kept only for a possible R9b re-point at the HMD, and runs only while fusionDiag is on
	// so normal play does not pay ~250 Hz mutex+QPC work or ~130 log lines of a settled
	// verdict. R9c fixed the log text so it no longer generalises the tracker result to GatePublish.
	//
	// Ground truth is independent of the reported vector: differentiate successive *world*
	// rotations. The two hypotheses coincide whenever the tracker is square to the room, so
	// samples are only counted when they actually disagree — `n` is the discriminating count.
	// Head ROLL and PITCH generate these; pure upright yaw does not.
	if (fusionDiag)
	{
		static std::mutex r9Mutex;
		static bool r9Have = false;
		static vr::HmdQuaternion_t r9PrevWorldRot = { 1, 0, 0, 0 };
		static int64_t r9PrevTick = 0;
		static int64_t r9LastLog = 0;
		static double r9SumDriver = 0.0;   // residual if vecAngularVelocity is driver-space
		static double r9SumLocal = 0.0;    // residual if it is device-local
		static uint64_t r9N = 0;           // discriminating samples
		static uint64_t r9Seen = 0;        // rotating samples considered

		LARGE_INTEGER nowLi{}, freqLi{};
		QueryPerformanceCounter(&nowLi);
		QueryPerformanceFrequency(&freqLi);
		const int64_t nowTick = nowLi.QuadPart;
		const double qpc = (double)freqLi.QuadPart;

		std::lock_guard<std::mutex> lock(r9Mutex);

		if (r9Have && r9PrevTick != 0 && qpc > 0.0)
		{
			const double dt = (double)(nowTick - r9PrevTick) / qpc;
			if (dt > 1e-4 && dt < 0.1)
			{
				vr::HmdVector3d_t wNum = quaternionAngularVelocity(worldRot, r9PrevWorldRot, dt);
				const double wMag = sqrt(wNum.v[0] * wNum.v[0] + wNum.v[1] * wNum.v[1] + wNum.v[2] * wNum.v[2]);
				// ~17 deg/s: below this the numeric derivative is mostly sample noise.
				if (std::isfinite(wMag) && wMag > 0.3)
				{
					++r9Seen;
					vr::HmdVector3d_t hDriver = quaternionRotateVector(
						quaternionNormalize(pose.qWorldFromDriverRotation), ang);
					vr::HmdVector3d_t hLocal = quaternionRotateVector(
						quaternionNormalize(pose.qWorldFromDriverRotation * pose.qRotation), ang);

					const double sx = hDriver.v[0] - hLocal.v[0];
					const double sy = hDriver.v[1] - hLocal.v[1];
					const double sz = hDriver.v[2] - hLocal.v[2];
					const double sep = sqrt(sx * sx + sy * sy + sz * sz);

					// Only count samples where the hypotheses meaningfully disagree.
					if (sep > 0.25 * wMag)
					{
						const double ax = hDriver.v[0] - wNum.v[0];
						const double ay = hDriver.v[1] - wNum.v[1];
						const double az = hDriver.v[2] - wNum.v[2];
						const double bx = hLocal.v[0] - wNum.v[0];
						const double by = hLocal.v[1] - wNum.v[1];
						const double bz = hLocal.v[2] - wNum.v[2];
						r9SumDriver += sqrt(ax * ax + ay * ay + az * az);
						r9SumLocal += sqrt(bx * bx + by * by + bz * bz);
						++r9N;
					}
				}
			}
		}

		r9PrevWorldRot = worldRot;
		r9PrevTick = nowTick;
		r9Have = true;

		if (r9LastLog == 0)
			r9LastLog = nowTick;
		if (qpc > 0.0 && (double)(nowTick - r9LastLog) / qpc >= 20.0)
		{
			if (r9N >= 50)
			{
				const double mDriver = r9SumDriver / (double)r9N;
				const double mLocal = r9SumLocal / (double)r9N;
				// R9c: this line used to end "...GatePublish is wrong". It does not follow.
				// GatePublish rotates the HMD's angular velocity, and this sampler only ever
				// sees the lighthouse TRACKER — a different device, published by a different
				// driver, with no obligation to share a convention. That is R9b, still open
				// precisely because nothing has measured the HMD side. Stating the conclusion
				// here put a claim the register denies into the log a future session reads first.
				const char* verdict =
					(mDriver < mLocal * 0.7) ? "DRIVER-SPACE fits => tracker cache read (qWorldFromDriver*qRotation) is WRONG"
					: (mLocal < mDriver * 0.7) ? "DEVICE-LOCAL fits => tracker cache read is correct (says nothing about the HMD path - see R9b)"
					: "INCONCLUSIVE - need more head roll/pitch";
				LOG("R9 angvel frame: n=%llu seen=%llu resid_driverspace=%.3f resid_devicelocal=%.3f rad/s -> %s",
					(unsigned long long)r9N, (unsigned long long)r9Seen,
					mDriver, mLocal, verdict);
			}
			else
			{
				LOG("R9 angvel frame: only %llu discriminating samples (seen=%llu) - roll and pitch the head, upright yaw cannot separate the hypotheses",
					(unsigned long long)r9N, (unsigned long long)r9Seen);
			}
			r9LastLog = nowTick;
		}
	}
	// ---------------------------------------------------------------------------------

	vr::HmdVector3d_t worldVel = quaternionRotateVector(
		quaternionNormalize(pose.qWorldFromDriverRotation), vel);
	vr::HmdVector3d_t worldAng = quaternionRotateVector(
		quaternionNormalize(pose.qWorldFromDriverRotation * pose.qRotation),
		ang);

	CachedTrackerPose next;
	next.rotation = worldRot;
	next.position[0] = pos[0];
	next.position[1] = pos[1];
	next.position[2] = pos[2];
	next.velocity[0] = worldVel.v[0];
	next.velocity[1] = worldVel.v[1];
	next.velocity[2] = worldVel.v[2];
	next.angularVelocity[0] = worldAng.v[0];
	next.angularVelocity[1] = worldAng.v[1];
	next.angularVelocity[2] = worldAng.v[2];
	next.measuredSpeed = measuredSpeed;
	next.result = pose.result;
	QueryPerformanceCounter(&next.timestamp);
	next.valid = true;

	std::lock_guard<std::mutex> lock(trackerCacheMutex);
	cachedTracker = next;
}

bool ServerTrackedDeviceProvider::FetchTrackerSample(
	vr::HmdQuaternion_t& outRot, double outPos[3],
	double outVel[3], double outAngVel[3], double& outLinSpeed, double& outAgeSec,
	vr::ETrackingResult& outResult)
{
	LARGE_INTEGER now{}, freq{};
	QueryPerformanceCounter(&now);
	QueryPerformanceFrequency(&freq);

	const double kCacheMaxAgeSec = 0.15;
	const double maxAcceptLinSpeed = kMaxPlausibleHeadSpeed;

	outLinSpeed = 0.0;
	outAgeSec = 0.0;
	outResult = vr::TrackingResult_Uninitialized;

	CachedTrackerPose cache;
	{
		std::lock_guard<std::mutex> lock(trackerCacheMutex);
		cache = cachedTracker;
	}

	if (!cache.valid)
		return false;

	double age = (now.QuadPart - cache.timestamp.QuadPart) / (double)freq.QuadPart;
	if (age < 0.0 || age > kCacheMaxAgeSec)
		return false;
	outAgeSec = age;
	outResult = cache.result;

	outRot = quaternionIntegrateOmega(cache.rotation, cache.angularVelocity, age);
	for (int i = 0; i < 3; i++)
	{
		outPos[i] = cache.position[i] + cache.velocity[i] * age;
		outVel[i] = cache.velocity[i];
		outAngVel[i] = cache.angularVelocity[i];
	}
	outLinSpeed = cache.measuredSpeed;

	if (outLinSpeed > maxAcceptLinSpeed
		|| !IsFiniteQuat(outRot)
		|| !IsFiniteVec3(outPos)
		|| !IsFiniteVec3(outVel)
		|| fabs(outPos[1]) > 100.0)
	{
		return false;
	}

	return true;
}

bool ServerTrackedDeviceProvider::HandleDevicePoseUpdated(uint32_t openVRID, vr::DriverPose_t& pose)
{
	if (openVRID >= vr::k_unMaxTrackedDeviceCount)
		return true;

	std::shared_lock<std::shared_mutex> configLock(configMutex);

	// Head-mounted lighthouse tracker while override is active: cache + quash.
	// Never GetRaw the published pose while quashing (C11).
	if (hmdTracker.enabled
		&& hmdTracker.trackerID < vr::k_unMaxTrackedDeviceCount
		&& openVRID == hmdTracker.trackerID)
	{
		CacheTrackerWorldPose(pose);

		pose.vecPosition[0] = -pose.vecWorldFromDriverTranslation[0];
		pose.vecPosition[1] = -pose.vecWorldFromDriverTranslation[1] + 9001.0;
		pose.vecPosition[2] = -pose.vecWorldFromDriverTranslation[2];
		for (int i = 0; i < 3; i++)
		{
			pose.vecVelocity[i] = 0;
			pose.vecAngularVelocity[i] = 0;
		}
		pose.poseIsValid = true;
		pose.deviceIsConnected = true;
		pose.result = vr::TrackingResult_Running_OK;
		return true;
	}

	// Role_LH capture must see the pose BEFORE the per-device transform block below.
	// Schema (and CaptureFormat.h comments) promise lighthouse-native uncorrected poses;
	// after transforms[] the row would carry calibration rot/trans/scale that the stream
	// never records, and offline head-vs-body subtraction would call that "drift".
	if (hmdTracker.enabled
		&& openVRID != hmdTracker.hmdID
		&& !slamSync[openVRID])
	{
		CaptureDevicePose(openVRID, capture::Role_LH, pose);
	}

	auto& tf = transforms[openVRID];
	if (tf.enabled && !hmdTracker.native)
	{
		pose.qWorldFromDriverRotation = tf.rotation * pose.qWorldFromDriverRotation;

		pose.vecPosition[0] *= tf.scale;
		pose.vecPosition[1] *= tf.scale;
		pose.vecPosition[2] *= tf.scale;

		vr::HmdVector3d_t rotatedTranslation = quaternionRotateVector(tf.rotation, pose.vecWorldFromDriverTranslation);
		pose.vecWorldFromDriverTranslation[0] = rotatedTranslation.v[0] + tf.translation.v[0];
		pose.vecWorldFromDriverTranslation[1] = rotatedTranslation.v[1] + tf.translation.v[1];
		pose.vecWorldFromDriverTranslation[2] = rotatedTranslation.v[2] + tf.translation.v[2];
	}

	if (hmdTracker.enabled)
	{
		if (openVRID == hmdTracker.hmdID)
		{
			// Snapshot config for the pure estimator (no locks inside ProcessHmdFrame).
			pose_est::PoseConfig cfg;
			cfg.fusionMode = fusionMode;
			cfg.trackerFilterEnabled = trackerFilterEnabled;
			cfg.driftFilterEnabled = driftFilterEnabled;
			cfg.headVelFilterEnabled = headVelFilterEnabled;
			cfg.publishSlewEnabled = publishSlewEnabled;
			cfg.corrRateLimitEnabled = corrRateLimitEnabled;
			cfg.native = hmdTracker.native;
			cfg.slamFallback = hmdTracker.slamFallback;
			cfg.enableAngularVelocity = hmdTracker.enableAngularVelocity;
			cfg.predictionTime = hmdTracker.predictionTime;
			cfg.offsetRotation = hmdTracker.offsetRotation;
			cfg.offsetTranslation = hmdTracker.offsetTranslation;
			cfg.calibrationRotation = hmdTracker.calibrationRotation;
			cfg.calibrationTranslation = hmdTracker.calibrationTranslation;
			cfg.calibrationScale = hmdTracker.calibrationScale;
			cfg.hmdScale = hmdTracker.hmdScale;
			cfg.headFilterEnabled = poseState.headFilter.enabled;

			pose_est::PoseClock clock;
			LARGE_INTEGER _now{}, _freq{};
			QueryPerformanceCounter(&_now);
			QueryPerformanceFrequency(&_freq);
			clock.now = _now.QuadPart;
			clock.freq = _freq.QuadPart;

			pose_est::HmdInput hmdIn;
			hmdIn.rawValid = pose.poseIsValid && pose.deviceIsConnected && pose.result == vr::TrackingResult_Running_OK;
			if (hmdIn.rawValid)
				ComposeWorldPose(pose, hmdIn.rotation, hmdIn.position);

			double displayHz = GetCachedDisplayHz(openVRID);

			pose_est::TrackerInput trackerIn;
			const bool trackerPoseOk = FetchTrackerSample(
				trackerIn.rotation, trackerIn.position, trackerIn.velocity, trackerIn.angularVelocity,
				trackerIn.linSpeed, trackerIn.ageSec, trackerIn.result);
			trackerIn.poseOk = trackerPoseOk;
			trackerIn.speedReject = !trackerPoseOk && trackerIn.linSpeed > kMaxPlausibleHeadSpeed;
			trackerIn.okForOverride =
				trackerPoseOk && trackerIn.result == vr::TrackingResult_Running_OK;

			// Record estimator INPUTS before ProcessHmdFrame mutates `pose`. CaptureWrite is a
			// no-op when the sink is off, so this path stays free on normal play.
			const uint32_t capGen = CaptureGeneration();
			if (capGen != 0)
			{
				// A new generation is a new FILE. Reset the file-local counters here, on the one
				// thread that writes them: leaving captureConfigWrittenSeq behind the bump forces
				// a fresh Rec_Config ahead of this file's first Rec_Frame, and frameIndex restarts
				// so contiguity-from-zero holds per file. Doing this in RunFrame raced this path
				// (the file is published before RunFrame can take configMutex exclusive), and a
				// same-process Cleanup->Init never re-runs the member initialisers at all.
				if (capGen != captureLastGen)
				{
					captureLastGen = capGen;
					captureConfigSeq++;
					captureFrameIndex = 0;
				}

				// trackerID is NOT part of PoseConfig, so a tracker swap would otherwise keep
				// the old configSeq and a replay would attribute the new tracker's samples to
				// the old one. Compare it alongside.
				if (!captureLastCfgValid
					|| !PoseConfigEqual(cfg, captureLastCfg)
					|| captureLastTrackerID != hmdTracker.trackerID)
				{
					if (captureLastCfgValid)
						captureConfigSeq++;
					captureLastCfg = cfg;
					captureLastTrackerID = hmdTracker.trackerID;
					captureLastCfgValid = true;
				}
				if (captureConfigWrittenSeq != captureConfigSeq)
				{
					capture::RecConfigPayload crec{};
					crec.seq = captureConfigSeq;
					crec.fusionMode = cfg.fusionMode ? 1 : 0;
					crec.native = cfg.native ? 1 : 0;
					crec.slamFallback = cfg.slamFallback ? 1 : 0;
					crec.enableAngularVelocity = cfg.enableAngularVelocity ? 1 : 0;
					crec.headFilterEnabled = cfg.headFilterEnabled ? 1 : 0;
					crec.predictionTime = cfg.predictionTime;
					crec.offsetRotation = CapQ(cfg.offsetRotation);
					crec.offsetTranslation = CapV(cfg.offsetTranslation);
					crec.calibrationRotation = CapQ(cfg.calibrationRotation);
					crec.calibrationTranslation = CapV(cfg.calibrationTranslation);
					crec.calibrationScale = cfg.calibrationScale;
					crec.hmdScale = cfg.hmdScale;
					// 1 by construction — this block only runs under `if (hmdTracker.enabled)`.
					// Recorded so a replay reads the flag instead of assuming it; the disable
					// transition is carried by the Rec_Event mirror of the SetHmdTracker line.
					crec.enabled = hmdTracker.enabled ? 1 : 0;
					crec.trackerID = hmdTracker.trackerID;
					CaptureWrite(capture::Rec_Config, &crec, sizeof crec);
					captureConfigWrittenSeq = captureConfigSeq;
				}

				capture::RecFramePayload frec{};
				frec.configSeq = captureConfigSeq;
				frec.frameIndex = captureFrameIndex++;
				frec.clockNow = clock.now;
				frec.clockFreq = clock.freq;
				frec.displayHz = displayHz;

				frec.hmdRawValid = hmdIn.rawValid ? 1 : 0;
				frec.hmdRotation = CapQ(hmdIn.rotation);
				frec.hmdPosition = CapV3(hmdIn.position);

				frec.trkPoseOk = trackerIn.poseOk ? 1 : 0;
				frec.trkOkForOverride = trackerIn.okForOverride ? 1 : 0;
				frec.trkSpeedReject = trackerIn.speedReject ? 1 : 0;
				frec.trkResult = (int32_t)trackerIn.result;
				frec.trkRotation = CapQ(trackerIn.rotation);
				frec.trkPosition = CapV3(trackerIn.position);
				frec.trkVelocity = CapV3(trackerIn.velocity);
				frec.trkAngularVelocity = CapV3(trackerIn.angularVelocity);
				frec.trkLinSpeed = trackerIn.linSpeed;
				frec.trkAgeSec = trackerIn.ageSec;

				frec.inPoseIsValid = pose.poseIsValid ? 1 : 0;
				frec.inDeviceIsConnected = pose.deviceIsConnected ? 1 : 0;
				frec.inResult = (int32_t)pose.result;
				frec.inRotation = CapQ(pose.qRotation);
				frec.inPosition = { pose.vecPosition[0], pose.vecPosition[1], pose.vecPosition[2] };
				frec.inWorldFromDriverRotation = CapQ(pose.qWorldFromDriverRotation);
				frec.inWorldFromDriverTranslation = {
					pose.vecWorldFromDriverTranslation[0],
					pose.vecWorldFromDriverTranslation[1],
					pose.vecWorldFromDriverTranslation[2]
				};
				frec.inDriverFromHeadRotation = CapQ(pose.qDriverFromHeadRotation);
				frec.inDriverFromHeadTranslation = {
					pose.vecDriverFromHeadTranslation[0],
					pose.vecDriverFromHeadTranslation[1],
					pose.vecDriverFromHeadTranslation[2]
				};
				frec.inVelocity = { pose.vecVelocity[0], pose.vecVelocity[1], pose.vecVelocity[2] };
				frec.inAngularVelocity = {
					pose.vecAngularVelocity[0],
					pose.vecAngularVelocity[1],
					pose.vecAngularVelocity[2]
				};
				CaptureWrite(capture::Rec_Frame, &frec, sizeof frec);
			}

			// Pure decision path — no SteamVR, no GetRaw, no locks.
			pose_est::ProcessHmdFrame(cfg, clock, displayHz, hmdIn, trackerIn, pose, poseState, diag);

			// Body devices (slamSync) need the correction under driftMutex.
			PublishDrift();
			return true;
		}
		else if (slamSync[openVRID])
		{
			// Capture AFTER the correction so Role_Sync rows are lighthouse-world, same frame as
			// Role_Head from CacheTrackerWorldPose. Pre-correction capture was SLAM-space and
			// made offline head-vs-body subtraction report the entire `corr` as "body drift".
			// role=sync still means this device received the drift + slamScale on publish.
			ApplySharedDrift(pose);
			CaptureDevicePose(openVRID, capture::Role_Sync, pose);
		}
		// Role_LH was captured above, before transforms[].
	}

	return true;
}
