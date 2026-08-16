// SPDX-License-Identifier: AGPL-3.0-only
//
// Pure HMD pose decision path (fusion EKF + override rebuild + publish gate).
// No locks, no SteamVR runtime calls, no GetRaw. Callers inject time via PoseClock
// and a config snapshot. The live MinHook path is one caller; offline file replay
// is another (link this TU, feed recorded samples + synthetic ticks).

#pragma once

#include "KalmanFilter.h"
#include "OneEuroFilter.h"
#include "PoseMath.h"

#include <openvr_driver.h>

#include <cstdint>

namespace pose_est {

// Injected clock. Live path fills from QueryPerformanceCounter/Frequency once per
// HMD frame. Offline path uses synthetic monotonic ticks (freq arbitrary, e.g. 10 MHz).
struct PoseClock
{
	int64_t now = 0;
	int64_t freq = 1;

	double AgeSec(int64_t then) const
	{
		if (freq == 0)
			return 0.0;
		return (now - then) / (double)freq;
	}
};

// Immutable for the duration of ProcessHmdFrame (caller holds config lock / snapshot).
struct PoseConfig
{
	bool fusionMode = false;
	bool native = false;
	bool slamFallback = true;
	bool enableAngularVelocity = false;
	float predictionTime = 1.0f;

	vr::HmdQuaternion_t offsetRotation = { 1, 0, 0, 0 };
	vr::HmdVector3d_t offsetTranslation = { 0, 0, 0 };
	vr::HmdQuaternion_t calibrationRotation = { 1, 0, 0, 0 };
	vr::HmdVector3d_t calibrationTranslation = { 0, 0, 0 };
	double calibrationScale = 1.0;
	double hmdScale = 1.0;

	bool headFilterEnabled = false;
	// KalmanFilterXYZ pre-filter on the tracker position. False bypasses it entirely, which in
	// override mode removes ~125 ms of lag from the published head pose. See the settings read
	// in ServerTrackedDeviceProvider::Init for why this defaults off.
	bool trackerFilterEnabled = false;
	// One-Euro on the drift/correction transform (UpdateDrift). Smoothing of the correction,
	// not of head motion -- real head motion does not pass through this channel.
	bool driftFilterEnabled = true;
	// One-Euro on the PUBLISHED head angular velocity. Does not move the pose, but games that
	// extrapolate from vecAngularVelocity feel it.
	bool headVelFilterEnabled = true;
	// SAFETY BOUND, not smoothing: GatePublish's jump hold and reconvergence slew. Off means a
	// far-away pose candidate is snapped to rather than approached -- the teleport this exists
	// to prevent. Only engages while reconverging, so it costs nothing in steady state.
	bool publishSlewEnabled = true;
	// SAFETY BOUND, not smoothing: the N1-h correction rate limit. Off restores the measured
	// 17.87-degree single-frame view yaw step on covariance reset. Only engages on re-anchor.
	bool corrRateLimitEnabled = true;

	double SlamScale() const
	{
		return pose_math::SlamToCorrectedScale(hmdScale, calibrationScale, native);
	}
};

// Tracker observation after cache/fetch (may be dead-reckoned). PredictionTime is applied
// inside ProcessHmdFrame for override mode, to a LOCAL COPY — this struct is not modified.
// ProcessHmdFrame treats its inputs as read-only, which is what lets the offline replay
// harness (N2-a) feed the same sample to repeated runs and get the same answer.
struct TrackerInput
{
	bool poseOk = false;          // usable sample (any accepted result)
	bool okForOverride = false;   // Running_OK only
	bool speedReject = false;     // measured speed exceeded gate
	vr::HmdQuaternion_t rotation = { 1, 0, 0, 0 };
	double position[3] = { 0, 0, 0 };
	double velocity[3] = { 0, 0, 0 };
	double angularVelocity[3] = { 0, 0, 0 };
	double linSpeed = 0.0;
	double ageSec = 0.0;
	vr::ETrackingResult result = vr::TrackingResult_Uninitialized;
};

// SLAM / raw HMD pose in world space (composed before ProcessHmdFrame).
struct HmdInput
{
	bool rawValid = false;
	vr::HmdQuaternion_t rotation = { 1, 0, 0, 0 };
	double position[3] = { 0, 0, 0 };
};

// Mutable estimator state. HMD-thread only for live use; single-threaded offline.
struct PoseState
{
	struct DriftCorrection
	{
		bool valid = false;
		vr::HmdQuaternion_t rotation = { 1, 0, 0, 0 };
		vr::HmdVector3d_t translation = { 0, 0, 0 };
		int64_t lastUpdate = 0;
		oneeuro::Quat rotationFilter;
		oneeuro::Vec3 translationFilter;
	} drift;

	struct HeadFilter
	{
		bool enabled = false;
		bool valid = false;
		int64_t lastUpdate = 0;
		oneeuro::Quat rotationFilter;
		oneeuro::Vec3 translationFilter;

		void reset()
		{
			valid = false;
			rotationFilter.reset();
			translationFilter.reset();
		}
	} headFilter;

	struct TrackerFilter
	{
		KalmanFilterXYZ translation;
		void reset() { translation.reset(); }
	} trackerFilter;

	struct HeadVelocity
	{
		bool valid = false;
		int64_t lastUpdate = 0;
		vr::HmdQuaternion_t prevRotation = { 1, 0, 0, 0 };
		oneeuro::Vec3 filter;

		void reset()
		{
			valid = false;
			filter.reset();
		}
	} headVel;

	struct FusionState
	{
		bool havePrev = false;
		int64_t lastFrame = 0;
		int64_t lastCancel = 0;
		double prevRawPos[3] = { 0, 0, 0 };
		double prevObsPos[3] = { 0, 0, 0 };
	} fusion;

	struct FusionEkf
	{
		bool valid = false;
		vr::HmdQuaternion_t yawCorr = { 1, 0, 0, 0 };
		double trans[3] = { 0, 0, 0 };
		double Ptheta = 1.0;
		double Pt = 1.0;
		int outlierRun = 0;
		int64_t lastUpdate = 0;
	} ekf;

	struct LastGoodHmd
	{
		bool valid = false;
		int64_t timestamp = 0;
		vr::HmdQuaternion_t rotation = { 1, 0, 0, 0 };
		double position[3] = { 0, 0, 0 };
		double velocity[3] = { 0, 0, 0 };
		double angularVelocity[3] = { 0, 0, 0 };
	} lastGoodHmd;

	bool reconverging = false;

	// Optional fusion CSV timing (live path only; offline may leave zeroed).
	int64_t diagStart = 0;
	int64_t diagLastWrite = 0;

	void Reset(bool clearLastGood)
	{
		ekf = FusionEkf{};
		fusion = FusionState{};
		reconverging = false;
		headFilter.reset();
		headVel.reset();
		trackerFilter.reset();
		drift.valid = false;
		drift.rotationFilter.reset();
		drift.translationFilter.reset();
		if (clearLastGood)
			lastGoodHmd.valid = false;
	}
};

// Diagnostics counters mutated by ProcessHmdFrame (subset of session diag).
struct PoseDiag
{
	bool haveLastHmdPos = false;
	double lastHmdPos[3] = { 0, 0, 0 };
	int64_t lastHmdPosTime = 0;
	bool lastTrackerOk = false;
	bool trackerStateKnown = false;
	bool playPrimed = false;
	uint64_t frames = 0;
	uint64_t trackerOkFrames = 0;
	uint64_t trackerBadFrames = 0;
	uint64_t jumpEvents = 0;
	uint64_t jumpHolds = 0;
	uint64_t lastGoodHolds = 0;
	uint64_t speedRejects = 0;
	uint64_t fallbackFrames = 0;
	uint64_t nonFiniteDrops = 0;
	uint64_t slamSteps = 0;
	// N1-h: times the correction's own slew bound engaged. This is an artifact counter in the
	// N2-c sense, not a filter statistic — a nonzero value means the estimator tried to move the
	// correction faster than it physically can, which is the signature that produced the
	// 17.87 deg one-frame view step. Steady play must read 0.
	uint64_t corrSlewLimited = 0;
	// Total SLAM-origin motion absorbed into the correction this play period. `corr` is a
	// state variable, not an error: after a step the correction *must* grow by the step size.
	// Logging the two together stops a large-but-correct `corr` from reading as a fault
	// (session_20260731_103323: corr held 74.9 cm at disp_avg 0.1-0.3 cm — nominal — and the
	// unexplained number prompted a needless manual re-enable mid-session).
	double slamStepSumM = 0;
	double dispSumCm = 0;
	double dispMaxCm = 0;
	uint64_t dispN = 0;
	int64_t lastHeartbeat = 0;
	int64_t lastJumpLog = 0;
	int64_t lastBadLog = 0;
};

// Apply the current correction to a driver-local pose (fusion / slamSync / fallback).
// Uses state.drift; no locks.
bool ApplyDrift(vr::DriverPose_t& pose, const PoseConfig& cfg, const PoseState& state);

// One HMD frame: fusion or override decision path + publish gate.
// Rewrites `pose` in place. Does not acquire locks. Does not call GetRaw*.
//
// Returns true when the pose is ready to publish (always the case after processing;
// kept for symmetry with future early-outs). Live hook ignores the return and
// always continues.
bool ProcessHmdFrame(
	const PoseConfig& cfg,
	const PoseClock& clock,
	double displayHz,
	const HmdInput& hmd,
	// const: override prediction works on a local copy (`rawTrackerPos`), so this is genuinely
	// read-only. The old signature and its "prediction mutates position" comment contradicted
	// both the struct's own documentation and N2-a's pure-function contract, which is what lets
	// a replay feed one sample to repeated runs and get the same answer.
	const TrackerInput& tracker,
	vr::DriverPose_t& pose,
	PoseState& state,
	PoseDiag& diag);

inline double EkfYawCorrDeg(const PoseState& state)
{
	if (!state.ekf.valid)
		return 0.0;
	return 2.0 * atan2(state.ekf.yawCorr.y, state.ekf.yawCorr.w) * 180.0 / 3.14159265358979323846;
}

inline double EkfTransMagCm(const PoseState& state)
{
	if (!state.ekf.valid)
		return 0.0;
	return sqrt(state.ekf.trans[0] * state.ekf.trans[0]
		+ state.ekf.trans[1] * state.ekf.trans[1]
		+ state.ekf.trans[2] * state.ekf.trans[2]) * 100.0;
}

} // namespace pose_est
