// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include <Core>
#include <openvr.h>
#include <vector>
#include <string>
#include <iostream>

#include "Protocol.h"
#include "FilterDefaults.h"

enum class CalibrationState
{
	None,
	Begin,
	Detect,
	Sampling,
	PartialSampling,
	Recovering,
	Editing,
};

enum class OverrideInactiveReason
{
	Active,
	NoProfile,
	TrackerMissing,
	TrackerSerialAmbiguous,
	HmdSerialMismatch,
	HmdTrackingSystemMismatch,
	InvalidRelativeOffset,
};

struct OverrideStatus
{
	bool active = false;
	OverrideInactiveReason inactiveReason = OverrideInactiveReason::NoProfile;
};

struct CalibrationContext
{
	CalibrationState state = CalibrationState::None;
	uint32_t targetID;

	Eigen::Vector3d calibratedRotation;
	Eigen::Vector3d calibratedTranslation;
	double calibratedScale;

	vr::HmdQuaternion_t relativeRotation = { 1, 0, 0, 0 };
	vr::HmdVector3d_t relativeTranslation = { 0, 0, 0 };
	bool validRelativeOffset = false;

	std::string targetTrackingSystem;

	std::string hmdSerial;
	std::string hmdTrackingSystem;
	std::string trackerSerial;
	std::string preferredTrackerSerial;

	bool enabled = false;
	bool validProfile = false;
	double timeLastTick = 0, timeLastScan = 0, timeLastTelemetryFetch = 0, timeLastIpcHealthCheck = 0;
	double wantedUpdateInterval = 1.0;

	bool enableNative = false;
	bool fallbackToSlam = false;
	bool enableAngularVelocity = true;
	bool continuousSync = false;
	bool syncHmdDrift = false;
	float predictionTime = 1.0f;
	bool predictionAuto = true;
	float predictionLagMs = 0.0f;
	float predictionLagFrames = 0.0f;
	float autoPredictionFrames = 1.0f;
	bool predictionTelemetryValid = false;
	bool driverTelemetryValid = false;
	bool ipcHealthy = true;
	protocol::DriverTelemetry driverTelemetry{};

	double lastCalibrationTime = 0.0;
	double timeLastDriverTelemetryFetch = 0.0;

	bool liveCalibrationQualityValid = false;
	double liveCalibrationRmsMm = 0.0;
	bool trackerIsTundra = false;

	bool runtimeResidualValid = false;
	double runtimeResidualMm = 0.0;
	bool guardianShiftSuspect = false;
	double guardianShiftSlamJumpMm = 0.0;
	bool mountRigidityWarning = false;
	double baselineMountRmsMm = 0.0;
	double lastPartialMountRmsMm = 0.0;
	double timeLastRuntimeQualityUpdate = 0.0;

	bool headFilterEnabled = true;
	protocol::OneEuroParams headFilterParams = filter_defaults::Head;
	protocol::OneEuroParams driftFilterParams = filter_defaults::Drift;

	OverrideStatus overrideStatus;

	vr::VRNotificationId notificationId = 0;

	enum Speed
	{
		FAST = 0,
		SLOW = 1,
		VERY_SLOW = 2
	};
	Speed calibrationSpeed = FAST;

	vr::TrackedDevicePose_t devicePoses[vr::k_unMaxTrackedDeviceCount];

	struct Chaperone
	{
		bool valid = false;
		bool autoApply = true;
		std::vector<vr::HmdQuad_t> geometry;
		vr::HmdMatrix34_t standingCenter;
		vr::HmdVector2_t playSpaceSize;
	} chaperone;

	void Clear()
	{
		chaperone.geometry.clear();
		chaperone.standingCenter = vr::HmdMatrix34_t();
		chaperone.playSpaceSize = vr::HmdVector2_t();
		chaperone.valid = false;

		calibratedRotation = Eigen::Vector3d();
		calibratedTranslation = Eigen::Vector3d();
		calibratedScale = 1.0;
		relativeRotation = { 1, 0, 0, 0 };
		relativeTranslation = { 0, 0, 0 };
		validRelativeOffset = false;
		targetTrackingSystem = "";
		hmdSerial = "";
		hmdTrackingSystem = "";
		trackerSerial = "";
		enabled = false;
		validProfile = false;
		fallbackToSlam = false;
		continuousSync = false;
		syncHmdDrift = false;
		overrideStatus = {};
		calibrationFailureOffer = false;
		lastCalibrationRmsMm = 0.0;
		baselineMountRmsMm = 0.0;
		lastPartialMountRmsMm = 0.0;
		runtimeResidualValid = false;
		runtimeResidualMm = 0.0;
		guardianShiftSuspect = false;
		guardianShiftSlamJumpMm = 0.0;
		mountRigidityWarning = false;
	}

	static constexpr size_t PartialSampleCount = 40;
	static constexpr double PartialMountRmsThresholdMeters = 0.03;
	static constexpr double FullCalibrationRmsThresholdMeters = 0.1;
	static constexpr int MaxCalibrationAutoAttempts = 3;
	static constexpr size_t LiveQualityMinSamples = 8;
	static constexpr double TundraJitterWarnThresholdMeters = 0.012;
	static constexpr double RuntimeResidualWarnMm = 30.0;
	static constexpr int RuntimeResidualHighStreakThreshold = 15;
	static constexpr double AutoPartialRecalCooldownSec = 300.0;

	bool autoPartialRecalOnMountDrift = true;
	int runtimeResidualHighStreak = 0;
	double timeLastAutoPartialRecal = 0.0;
	static constexpr double MountRigidityWarnRatio = 1.5;
	static constexpr double MountRigidityWarnMinDeltaMm = 5.0;

	bool calibrationFailureOffer = false;
	double lastCalibrationRmsMm = 0.0;

	size_t SampleCount()
	{
		switch (calibrationSpeed)
		{
		case FAST:
			return 100;
		case SLOW:
			return 250;
		case VERY_SLOW:
			return 500;
		}
		return 100;
	}

	struct Message
	{
		enum Type
		{
			String,
			Progress
		} type = String;

		Message(Type type) : type(type) { }

		std::string str;
		int progress, target;
	};

	std::vector<Message> messages;

	void Log(const std::string &msg)
	{
		if (messages.empty() || messages.back().type == Message::Progress)
			messages.push_back(Message(Message::String));

		messages.back().str += msg;
		std::cerr << msg;
	}

	void Progress(int current, int target)
	{
		if (messages.empty() || messages.back().type == Message::String)
			messages.push_back(Message(Message::Progress));

		messages.back().progress = current;
		messages.back().target = target;
	}
};

extern CalibrationContext CalCtx;

void InitCalibrator();
void CalibrationTick(double time);
void StartCalibration();
void RetryCalibrationAfterFailure();
void RestoreCalibrationAfterFailure();
const char* CalibrationSpeedName(CalibrationContext::Speed speed);
void StartPartialRecalibration();
void LoadChaperoneBounds();
void ApplyChaperoneBounds();
void SendOneEuroParams();
void FetchPredictionTelemetry();
void FetchDriverTelemetry();
void InvalidateAppliedDriverState();
void ApplyRuntimeDriverSettings(double timeSec);
void UpdateRuntimeTrackingQuality(CalibrationContext& ctx);
OverrideStatus EvaluateOverrideStatus(const CalibrationContext& ctx);
const char* OverrideInactiveReasonText(OverrideInactiveReason reason);
const char* DriverOverrideInactiveReasonText(protocol::DriverOverrideInactiveReason reason);
bool IsStageTrackingSpace();
vr::VRNotificationId ShowCalibrationNotification(const char* text);