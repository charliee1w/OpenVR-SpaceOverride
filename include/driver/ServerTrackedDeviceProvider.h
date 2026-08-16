// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include "IPCServer.h"
#include "PoseEstimator.h"

#include <openvr_driver.h>

#include <mutex>
#include <shared_mutex>

class ServerTrackedDeviceProvider : public vr::IServerTrackedDeviceProvider
{
public:
	////// Start vr::IServerTrackedDeviceProvider functions

	virtual vr::EVRInitError Init(vr::IVRDriverContext *pDriverContext) override;
	virtual void Cleanup() override;
	virtual const char * const *GetInterfaceVersions() { return vr::k_InterfaceVersions; }
	virtual void RunFrame() override;
	virtual bool ShouldBlockStandbyMode() { return false; }
	virtual void EnterStandby() override;
	// On resume the tracker cache, the One-Euro filters, headVel.prevRotation and fusion.havePrev
	// all hold pre-standby state, and only one of them has a staleness guard. VNEXT's
	// session-lifecycle section said standby was invisible to the driver because there is no
	// event loop — that was wrong: SteamVR calls these, they were simply empty.
	virtual void LeaveStandby() override;

	////// End vr::IServerTrackedDeviceProvider functions

	ServerTrackedDeviceProvider() : server(this) { }
	void SetDeviceTransform(const protocol::SetDeviceTransform &newTransform);
	void SetHmdTracker(const protocol::SetHmdTracker &cmd);
	void SetSlamSync(const protocol::SetSlamSync &cmd);
	void SetOneEuro(const protocol::SetOneEuro &cmd);
	bool HandleDevicePoseUpdated(uint32_t openVRID, vr::DriverPose_t &pose);

private:
	// Snapshot published correction for slamSync body devices (other pose threads).
	// HMD path updates poseState.drift lock-free; then PublishDrift copies under mutex.
	bool ApplySharedDrift(vr::DriverPose_t &pose);
	void PublishDrift();
	void ResetEstimators(bool clearLastGood, bool clearSlamSync);

	double EkfYawCorrDeg() const { return pose_est::EkfYawCorrDeg(poseState); }
	double EkfTransMagCm() const { return pose_est::EkfTransMagCm(poseState); }

	// P1-b: cache head tracker from its own pose hook (avoids GetRaw every HMD frame).
	void CacheTrackerWorldPose(const vr::DriverPose_t &pose);
	bool FetchTrackerSample(vr::HmdQuaternion_t &outRot, double outPos[3],
		double outVel[3], double outAngVel[3], double &outLinSpeed, double &outAgeSec,
		vr::ETrackingResult &outResult);
	double GetCachedDisplayHz(uint32_t hmdOpenVRID);

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

	// Pure estimator (HMD thread). Offline tools can drive the same pose_est::ProcessHmdFrame.
	pose_est::PoseState poseState;
	pose_est::PoseDiag diag;

	// Published correction for non-HMD slamSync devices.
	struct SharedDrift
	{
		bool valid = false;
		vr::HmdQuaternion_t rotation = { 1, 0, 0, 0 };
		vr::HmdVector3d_t translation = { 0, 0, 0 };
	} sharedDrift;

	struct CachedTrackerPose
	{
		bool valid = false;
		LARGE_INTEGER timestamp = {};
		vr::HmdQuaternion_t rotation = { 1, 0, 0, 0 };
		double position[3] = { 0, 0, 0 };
		double velocity[3] = { 0, 0, 0 };
		double angularVelocity[3] = { 0, 0, 0 };
		double measuredSpeed = 0.0;
		vr::ETrackingResult result = vr::TrackingResult_Uninitialized;
	} cachedTracker;

	// Lock order: configMutex → trackerCacheMutex / driftMutex
	std::shared_mutex configMutex;
	std::mutex trackerCacheMutex;
	std::mutex driftMutex;

	bool fusionMode = false;
	bool fusionDiag = false;
	// Every filter and bound on the pose path, individually switchable at runtime. Defaults
	// match what each one is worth: the tracker pre-filter costs ~125 ms of view lag for
	// sub-millimetre noise and is off; the two safety bounds are on. See kFilterToggles in
	// ServerTrackedDeviceProvider.cpp, which is the single list Init() and RunFrame() both walk
	// -- adding a filter there wires up both the initial read and the live poll.
	bool trackerFilterEnabled = false;
	bool driftFilterEnabled = true;
	bool headVelFilterEnabled = true;
	bool publishSlewEnabled = true;
	bool corrRateLimitEnabled = true;

	// The table itself is a class member rather than a file-static because its pointers-to-member
	// address the private flags above. Defined in ServerTrackedDeviceProvider.cpp, where the
	// per-toggle rationale lives.
	struct FilterToggle
	{
		const char *key;
		bool ServerTrackedDeviceProvider::*field;
		bool defaultOn;
	};
	static const FilterToggle kFilterToggles[5];
	LARGE_INTEGER settingsLastPoll = {};

	double cachedDisplayHz = 90.0;
	LARGE_INTEGER displayHzLastQuery = {};
	bool displayHzQueried = false;

	// Unified capture (CaptureFormat.h). configSeq is referenced by Rec_Frame; bumped on
	// PoseConfig change and on capture re-open so a new file always gets a fresh Rec_Config.
	// All file-local counters are written ONLY by the HMD pose thread, which detects a new
	// file via captureLastGen vs CaptureGeneration() — resetting them from RunFrame raced the
	// pose path (the new file is published before RunFrame can take configMutex), and Init
	// cannot reset them for a same-process re-Init it never sees.
	uint32_t captureConfigSeq = 1;
	uint32_t captureConfigWrittenSeq = 0;
	uint32_t captureFrameIndex = 0;
	uint32_t captureLastGen = 0;
	pose_est::PoseConfig captureLastCfg{};
	// Tracked beside captureLastCfg because trackerID is not a PoseConfig field: without it a
	// tracker swap keeps the old configSeq and a replay mis-attributes the new tracker's samples.
	uint32_t captureLastTrackerID = vr::k_unTrackedDeviceIndexInvalid;
	bool captureLastCfgValid = false;
};
