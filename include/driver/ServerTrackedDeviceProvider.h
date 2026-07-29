// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include "IPCServer.h"
#include "OneEuroFilter.h"
#include "KalmanFilter.h"

#include <openvr_driver.h>

#include <mutex>
#include <shared_mutex>

class ServerTrackedDeviceProvider : public vr::IServerTrackedDeviceProvider
{
public:
	////// Start vr::IServerTrackedDeviceProvider functions

	/** initializes the driver. This will be called before any other methods are called. */
	virtual vr::EVRInitError Init(vr::IVRDriverContext *pDriverContext) override;

	/** cleans up the driver right before it is unloaded */
	virtual void Cleanup() override;

	/** Returns the version of the ITrackedDeviceServerDriver interface used by this driver */
	virtual const char * const *GetInterfaceVersions() { return vr::k_InterfaceVersions; }

	/** Allows the driver do to some work in the main loop of the server.
	* Polls live driver settings (fusionMode / fusionDiag) that the overlay writes. */
	virtual void RunFrame() override;

	/** Returns true if the driver wants to block Standby mode. */
	virtual bool ShouldBlockStandbyMode() { return false; }

	/** Called when the system is entering Standby mode. The driver should switch itself into whatever sort of low-power
	* state it has. */
	virtual void EnterStandby() { }

	/** Called when the system is leaving Standby mode. The driver should switch itself back to
	full operation. */
	virtual void LeaveStandby() { }

	////// End vr::IServerTrackedDeviceProvider functions

	ServerTrackedDeviceProvider() : server(this) { }
	void SetDeviceTransform(const protocol::SetDeviceTransform &newTransform);
	void SetHmdTracker(const protocol::SetHmdTracker &cmd);
	void SetSlamSync(const protocol::SetSlamSync &cmd);
	void SetOneEuro(const protocol::SetOneEuro &cmd);
	bool HandleDevicePoseUpdated(uint32_t openVRID, vr::DriverPose_t &pose);

private:
	void UpdateDrift(const vr::HmdQuaternion_t &correctedRotation, const double (&correctedPosition)[3],
		const vr::HmdQuaternion_t &rawRotation, const double (&rawPosition)[3], double weight = 1.0);
	// Applies the current correction if one is valid; returns whether it did.
	// Snapshots the correction under driftMutex, since non-HMD pose threads call
	// this (slamSync) concurrently with the HMD thread writing it.
	bool ApplyDrift(vr::DriverPose_t &pose);

	// Converged EKF correction, for the heartbeat. Read without the lock like the
	// neighbouring ekf.Pt diagnostic: these are logged figures, and a torn read costs
	// one slightly-off log line rather than anything in the pose path.
	double EkfYawCorrDeg() const
	{
		if (!ekf.valid)
			return 0.0;
		// yawCorr is a yaw-only quaternion, so the angle is 2*atan2(y, w).
		return 2.0 * atan2(ekf.yawCorr.y, ekf.yawCorr.w) * 180.0 / 3.14159265358979323846;
	}

	double EkfTransMagCm() const
	{
		if (!ekf.valid)
			return 0.0;
		return sqrt(ekf.trans[0] * ekf.trans[0] + ekf.trans[1] * ekf.trans[1]
			+ ekf.trans[2] * ekf.trans[2]) * 100.0;
	}

	double SlamToCorrectedScale() const
	{
		double k = hmdTracker.hmdScale > 0.0 ? 1.0 / hmdTracker.hmdScale : 1.0;
		return hmdTracker.native ? k : k * hmdTracker.calibrationScale;
	}

	// P1-b: cache head tracker from its own pose hook (avoids GetRaw every HMD frame).
	void CacheTrackerWorldPose(const vr::DriverPose_t &pose);
	// Prefer fresh hook cache; fall back to unpredicted GetRaw.
	bool FetchTrackerSample(vr::HmdQuaternion_t &outRot, double outPos[3],
		double outVel[3], double outAngVel[3], double &outLinSpeed);
	// P2-a: display Hz queried at most ~1 Hz.
	double GetCachedDisplayHz(uint32_t hmdOpenVRID);
	// P0-b: finite pos/quat.
	static bool IsFiniteVec3(const double v[3]);
	static bool IsFiniteQuat(const vr::HmdQuaternion_t &q);
	// P0-a / P1-a: last published good HMD override pose.
	// Always stored in WORLD space so the two modes are directly comparable: override
	// writes the world pose into vecPosition, while fusion leaves a driver-local
	// vecPosition plus a worldFromDriver correction. Storing the raw pose fields made
	// lastGoodHmd mean different things per mode and corrupted the first frames after
	// a live mode switch.
	void StoreLastGoodHmd(const vr::HmdQuaternion_t &worldRot, const double worldPos[3],
		const double velocity[3], const double angularVelocity[3]);
	bool ApplyLastGoodHmd(vr::DriverPose_t &pose, double maxAgeSec) const;
	// P0-a: if candidate jumps too far vs last good, hold last good instead.
	bool ShouldHoldForJump(const double newPos[3], double dtSec) const;

	IPCServer server;

	struct DeviceTransform
	{
		bool enabled = false;
		vr::HmdVector3d_t translation;
		vr::HmdQuaternion_t rotation;
		double scale;
	};

	DeviceTransform transforms[vr::k_unMaxTrackedDeviceCount];

	struct HmdTracker
	{
		bool enabled = false;
		bool native = false;
		bool slamFallback = true;
		bool enableAngularVelocity = false;
		float predictionTime = 1.0f;
		uint32_t hmdID = vr::k_unTrackedDeviceIndex_Hmd;
		uint32_t trackerID = vr::k_unTrackedDeviceIndexInvalid;
		vr::HmdQuaternion_t offsetRotation = { 1, 0, 0, 0 };
		vr::HmdVector3d_t offsetTranslation = { 0, 0, 0 };
		vr::HmdQuaternion_t calibrationRotation = { 1, 0, 0, 0 };
		vr::HmdVector3d_t calibrationTranslation = { 0, 0, 0 };
		double calibrationScale = 1.0;
		double hmdScale = 1.0;
	} hmdTracker;

	bool slamSync[vr::k_unMaxTrackedDeviceCount];

	struct DriftCorrection
	{
		bool valid = false;
		vr::HmdQuaternion_t rotation = { 1, 0, 0, 0 };
		vr::HmdVector3d_t translation = { 0, 0, 0 };

		LARGE_INTEGER lastUpdate = {};
		oneeuro::Quat rotationFilter;
		oneeuro::Vec3 translationFilter;
	} drift;

	struct HeadFilter
	{
		bool enabled = false;
		bool valid = false;
		LARGE_INTEGER lastUpdate = {};
		oneeuro::Quat rotationFilter;
		oneeuro::Vec3 translationFilter;

		void reset() { valid = false; rotationFilter.reset(); translationFilter.reset(); }
	} headFilter;

	struct TrackerFilter
	{
		KalmanFilterXYZ translation;
		void reset() { translation.reset(); }
	} trackerFilter;

	struct HeadVelocity
	{
		bool valid = false;
		LARGE_INTEGER lastUpdate = {};
		vr::HmdQuaternion_t prevRotation = { 1, 0, 0, 0 };
		oneeuro::Vec3 filter;

		void reset() { valid = false; filter.reset(); }
	} headVel;

	// P1-b: last good tracker sample from the tracker device's pose hook.
	struct CachedTrackerPose
	{
		bool valid = false;
		LARGE_INTEGER timestamp = {};
		vr::HmdQuaternion_t rotation = { 1, 0, 0, 0 };
		double position[3] = { 0, 0, 0 };
		double velocity[3] = { 0, 0, 0 };
		double angularVelocity[3] = { 0, 0, 0 };
	} cachedTracker;

	// V2-b: IPC setters exclusive; pose threads hold shared for the whole callback.
	std::shared_mutex configMutex;
	// V2-a: tracker cache crosses pose threads (tracker writes, HMD reads) while both
	// hold configMutex shared — needs its own lock. Order: configMutex → trackerCacheMutex.
	std::mutex trackerCacheMutex;
	// Same problem for the drift correction: the HMD pose thread writes it while other
	// device threads read it on the slamSync path, both holding configMutex only
	// *shared*, which does not exclude them from each other.
	// Order: configMutex → driftMutex.
	std::mutex driftMutex;

	// V1: currently slewing toward a far candidate (bounded catch-up, HMD thread only).
	bool reconverging = false;

	// FUSION mode (steamvr.vrsettings driver_spaceoverride/fusionMode, read at Init and
	// re-polled in RunFrame so the overlay can switch modes live):
	// SLAM is the pose source, the tracker only observes the SLAM→lighthouse correction.
	// LOS loss becomes a non-event; SLAM relocation steps are cancelled same-frame.
	bool fusionMode = false;

	// Per-frame fusion diagnostic CSV (driver_spaceoverride/fusionDiag). Answers
	// "is disp low during motion real alignment or the filter coasting?" by logging
	// speed alongside the Kalman gain Kt and the innovation disp.
	bool fusionDiag = false;
	LARGE_INTEGER diagStart = {};
	LARGE_INTEGER diagLastWrite = {};
	LARGE_INTEGER settingsLastPoll = {};

	// Fusion step-attribution state (HMD thread only).
	struct FusionState
	{
		bool havePrev = false;
		LARGE_INTEGER lastFrame = {};
		LARGE_INTEGER lastCancel = {};
		double prevRawPos[3] = { 0, 0, 0 };
		double prevObsPos[3] = { 0, 0, 0 };
	} fusion;

	// FUSION estimator: error-state Kalman filter on the yaw+translation correction.
	// Replaces the One-Euro drift filter in fusion mode; publishes its state into
	// `drift` (under driftMutex) so ApplyDrift / slamSync / fallback stay unchanged.
	// HMD thread only. Tracker-vs-SLAM latency skew is absorbed into motion-inflated
	// measurement noise, not corrected by a time-offset term — see the note below.
	struct FusionEkf
	{
		bool valid = false;
		vr::HmdQuaternion_t yawCorr = { 1, 0, 0, 0 };
		double trans[3] = { 0, 0, 0 };
		double Ptheta = 1.0;   // yaw error variance, rad^2 (large = unconverged)
		double Pt = 1.0;       // translation error variance, m^2, isotropic
		int outlierRun = 0;    // consecutive gated samples -> covariance reset
		LARGE_INTEGER lastUpdate = {};
	} ekf;

	// One EKF measurement step. Returns false when the sample was rejected by the
	// innovation gate (tracker glitch / unattributed step). Tracker-vs-SLAM latency
	// skew is absorbed into the motion-inflated measurement noise, not corrected via
	// an online time-offset — validated 2026-07-23: at the ~2 mm tracker noise floor
	// a time-offset estimator has no observable signal: the post-fit residual is noise,
	// and tau*velocity is degenerate with the translation state, so it rode noise.
	bool FusionEkfUpdate(const vr::HmdQuaternion_t &obsRot, const double obsPos[3],
		const vr::HmdQuaternion_t &rawRot, const double rawPos[3],
		double linSpeed, double angSpeed);

	// P0-b/V0-c/V1 publish gates shared by override and fusion paths.
	// Returns false when the pose was replaced by a last-good hold or invalidated
	// (caller must skip drift bookkeeping for this frame).
	bool GatePublish(vr::DriverPose_t &pose, double displayHz, double linSpeed);


	// P0/P1: last published override HMD pose (hold / jump gate).
	struct LastGoodHmd
	{
		bool valid = false;
		LARGE_INTEGER timestamp = {};
		vr::HmdQuaternion_t rotation = { 1, 0, 0, 0 };
		double position[3] = { 0, 0, 0 };
		double velocity[3] = { 0, 0, 0 };
		double angularVelocity[3] = { 0, 0, 0 };
	} lastGoodHmd;

	// P2-a
	double cachedDisplayHz = 90.0;
	LARGE_INTEGER displayHzLastQuery = {};
	bool displayHzQueried = false;

	// Auto session diagnostics (starts with driver load — no user action).
	struct SessionDiag
	{
		bool haveLastHmdPos = false;
		double lastHmdPos[3] = { 0, 0, 0 };
		LARGE_INTEGER lastHmdPosTime = {};
		bool lastTrackerOk = false;
		bool trackerStateKnown = false;
		bool playPrimed = false; // P2-b: counters reset after first solid OK
		uint64_t frames = 0;
		uint64_t trackerOkFrames = 0;
		uint64_t trackerBadFrames = 0;
		uint64_t jumpEvents = 0;
		uint64_t jumpHolds = 0;
		uint64_t lastGoodHolds = 0;
		uint64_t speedRejects = 0;
		uint64_t fallbackFrames = 0;
		uint64_t nonFiniteDrops = 0;
		// Fusion diagnostics: cancelled SLAM relocation steps + per-heartbeat-window
		// disparity (instantaneous vs filtered correction, cm).
		uint64_t slamSteps = 0;
		double dispSumCm = 0;
		double dispMaxCm = 0;
		uint64_t dispN = 0;
		LARGE_INTEGER lastHeartbeat = {};
		LARGE_INTEGER lastJumpLog = {};
		LARGE_INTEGER lastBadLog = {};
	} diag;
};
