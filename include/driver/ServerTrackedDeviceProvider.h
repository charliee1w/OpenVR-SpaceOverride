// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include "IPCServer.h"
#include "OneEuroFilter.h"

#include <openvr_driver.h>
#include <atomic>
#include <deque>
#include <mutex>
#include <vector>

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
	bool SetDeviceTransform(const protocol::SetDeviceTransform &newTransform);
	bool SetHmdTracker(const protocol::SetHmdTracker &cmd);
	bool SetSlamSync(const protocol::SetSlamSync &cmd);
	bool SetOneEuro(const protocol::SetOneEuro &cmd);
	protocol::PredictionTelemetry GetPredictionTelemetry() const;
	protocol::DriverTelemetry GetDriverTelemetry() const;
	bool HandleDevicePoseUpdated(uint32_t openVRID, vr::DriverPose_t &pose);

private:
	void UpdateDrift(const vr::HmdQuaternion_t &correctedRotation, const double (&correctedPosition)[3],
		const vr::HmdQuaternion_t &rawRotation, const double (&rawPosition)[3]);
	void ApplyDrift(vr::DriverPose_t &pose) const;

	IPCServer server;
	mutable std::mutex stateMutex;
	std::atomic<bool> poseWorkActive{ false };
	void RecomputePoseWorkActive();
	bool AnySlamSyncEnabled() const;
	void InvalidateDriftIfSlamSyncInactive();

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
		bool slamFallback = false;
		bool enableAngularVelocity = false;
		float predictionTime = 1.0f;
		float manualPredictionTime = 1.0f;
		bool predictionAuto = false;
		uint32_t hmdID = vr::k_unTrackedDeviceIndex_Hmd;
		uint32_t trackerID = vr::k_unTrackedDeviceIndexInvalid;
		vr::HmdQuaternion_t offsetRotation = { 1, 0, 0, 0 };
		vr::HmdVector3d_t offsetTranslation = { 0, 0, 0 };
		vr::HmdQuaternion_t calibrationRotation = { 1, 0, 0, 0 };
		vr::HmdVector3d_t calibrationTranslation = { 0, 0, 0 };
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

	struct HmdPoseSample
	{
		bool valid = false;
		vr::HmdQuaternion_t rotation = { 1, 0, 0, 0 };
		double position[3] = { 0, 0, 0 };
	};

	struct TrackerBlendState
	{
		bool active = false;
		bool trackerWasValid = false;
		bool hasEmittedPose = false;
		HmdPoseSample emittedPose;

		enum class Mode
		{
			ToTracker,
			ToSlam,
		};
		Mode mode = Mode::ToTracker;

		double durationSec = 0.4;
		LARGE_INTEGER startTime = {};
		bool timePrimed = false;

		HmdPoseSample fromPose;
		HmdPoseSample toPose;

		void reset()
		{
			active = false;
			trackerWasValid = false;
			hasEmittedPose = false;
			emittedPose = {};
			timePrimed = false;
			startTime = {};
			fromPose = {};
			toPose = {};
		}
	} trackerBlend;

	struct PredictionAutoTune
	{
		static constexpr std::size_t MaxSamples = 200;
		static constexpr double WindowSec = 2.0;
		static constexpr double MinAngSpeed = 0.35;
		static constexpr double MinLinSpeed = 0.15;

		std::deque<double> trackerSamples;
		std::deque<double> slamSamples;
		std::deque<double> trackerLinSamples;
		std::deque<double> slamLinSamples;
		std::deque<double> sampleTimes;

		bool estimateValid = false;
		float estimatedLagFrames = 0.0f;
		float estimatedLagMs = 0.0f;
		float appliedPredictionFrames = 1.0f;
		float displayHz = 90.0f;

		void reset()
		{
			trackerSamples.clear();
			slamSamples.clear();
			trackerLinSamples.clear();
			slamLinSamples.clear();
			sampleTimes.clear();
			estimateValid = false;
			estimatedLagFrames = 0.0f;
			estimatedLagMs = 0.0f;
			appliedPredictionFrames = 1.0f;
		}
	} predictionTune;

	struct TrackerHealth
	{
		bool poseValid = false;
		double lastValidQpc = 0.0;
	} trackerHealth;

	float EffectivePredictionFrames() const;
	void RecordPredictionSample(double timeSec, float trackerAngSpeed, float slamAngSpeed, float trackerLinSpeed, float slamLinSpeed);
	void UpdatePredictionAutoTune();
	void UpdateDisplayHz();
	void BeginTrackerBlend(TrackerBlendState::Mode mode, const HmdPoseSample& fromPose, const HmdPoseSample& toPose, double durationSec);
	HmdPoseSample ResolveBlendedHmdPose(bool trackerValid, const HmdPoseSample& trackerPose, const HmdPoseSample& slamPose, bool slamPoseValid);
	HmdPoseSample BuildTrackerHmdPose(const vr::TrackedDevicePose_t& tp) const;
	HmdPoseSample BuildSlamHmdPose(const vr::DriverPose_t& pose, bool valid) const;
	void WriteHmdPoseToDriver(vr::DriverPose_t& pose, const HmdPoseSample& hmdPose, const vr::TrackedDevicePose_t* trackerPose);

	struct HeadVelocity
	{
		bool valid = false;
		LARGE_INTEGER lastUpdate = {};
		vr::HmdQuaternion_t prevRotation = { 1, 0, 0, 0 };
		oneeuro::Vec3 filter;

		void reset() { valid = false; filter.reset(); }
	} headVel;
};