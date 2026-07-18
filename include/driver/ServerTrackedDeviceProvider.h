// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include "IPCServer.h"
#include "OneEuroFilter.h"
#include "KalmanFilter.h"

#include <openvr_driver.h>

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

	/** Allows the driver do to some work in the main loop of the server. */
	virtual void RunFrame() { }

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
	void ApplyDrift(vr::DriverPose_t &pose) const;

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
	void StoreLastGoodHmd(const vr::DriverPose_t &pose);
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
		LARGE_INTEGER lastHeartbeat = {};
		LARGE_INTEGER lastJumpLog = {};
		LARGE_INTEGER lastBadLog = {};
	} diag;
};
