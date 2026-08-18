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
		const vr::HmdQuaternion_t &rawRotation, const double (&rawPosition)[3]);
	// Reads `drift` directly. Only ever called from the HMD branch, i.e. the thread that owns
	// `drift`; other pose threads go through ApplySharedDrift instead.
	void ApplyDrift(vr::DriverPose_t &pose) const;

	// Snapshot of the correction published for slamSync body devices, which are updated on
	// other pose threads. The HMD path updates `drift` under the shared config lock and then
	// PublishDrift copies it here under driftMutex.
	void PublishDrift();
	bool ApplySharedDrift(vr::DriverPose_t &pose);

	// Display refresh rate for the prediction interval, cached and floored. Upstream read the
	// property inline on every HMD frame and divided by it unchecked.
	double GetCachedDisplayHz(uint32_t hmdOpenVRID);

	double SlamToCorrectedScale() const
	{
		double k = hmdTracker.hmdScale > 0.0 ? 1.0 / hmdTracker.hmdScale : 1.0;
		return hmdTracker.native ? k : k * hmdTracker.calibrationScale;
	}

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

	// Published correction for non-HMD slamSync devices. Written once per HMD frame by
	// PublishDrift; read by every other pose thread under driftMutex, which is what removes
	// the torn read of a quaternion being written concurrently.
	struct SharedDrift
	{
		bool valid = false;
		vr::HmdQuaternion_t rotation = { 1, 0, 0, 0 };
		vr::HmdVector3d_t translation = { 0, 0, 0 };
	} sharedDrift;

	// Backing store for GetCachedDisplayHz. Written only from the HMD branch of the pose
	// callback, i.e. by the same thread that reads it.
	double cachedDisplayHz = 90.0;
	LARGE_INTEGER displayHzLastQuery = {};
	bool displayHzQueried = false;

	// Lock order: configMutex -> driftMutex. Never the reverse; nothing takes driftMutex first.
	//
	// configMutex is held SHARED for a whole pose callback and EXCLUSIVE in the IPC setters, so
	// a config write cannot tear an in-flight pose: upstream applied SetHmdTracker field by
	// field while a pose callback was part-way through reading them, which could compose a head
	// pose from half of one calibration and half of another.
	std::shared_mutex configMutex;
	std::mutex driftMutex;
};