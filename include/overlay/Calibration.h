// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include <Core>
#include <openvr.h>
#include <vector>
#include <string>
#include <iostream>

#include "Protocol.h"

enum class CalibrationState
{
	None,
	Begin,
	Detect,
	Sampling,
	Editing,
};

struct CalibrationContext
{
	CalibrationState state = CalibrationState::None;
	uint32_t targetID;

	Eigen::Vector3d calibratedRotation;
	Eigen::Vector3d calibratedTranslation;
	double calibratedScale = 1.0;
	double targetModelScale = 1.0;
	double hmdScale = 1.0;

	vr::HmdQuaternion_t relativeRotation = { 1, 0, 0, 0 };
	vr::HmdVector3d_t relativeTranslation = { 0, 0, 0 };
	bool validRelativeOffset = false;
	// Number of calibrations averaged into relativeTranslation so far. Persisted
	// alongside it (see Configuration.cpp) so the running average actually
	// accumulates across sessions instead of resetting every relaunch.
	int leverSamples = 0;
	// Serial of the physical tracker the rel_*/lever_n average was measured on. Persisted with
	// it, and deliberately SEPARATE from trackerSerial: trackerSerial tracks the live target and
	// is overwritten the moment sampling starts, while this field only advances when a solve
	// actually produces a measurement. The swap-detecting reset in ComputeRelativeOffset keys on
	// this, so an abandoned run can neither contaminate the average across a tracker swap (the
	// old volatile-flag scheme lost the pending reset when the overlay closed, while SaveProfile
	// had already persisted the new serial) nor wipe it when the user swaps back before solving.
	std::string leverSerial;

	std::string targetTrackingSystem;

	std::string hmdSerial;
	std::string trackerSerial;

	bool enabled = false;
	bool validProfile = false;
	double timeLastTick = 0, timeLastScan = 0;
	double wantedUpdateInterval = 1.0;

	bool enableNative = false;
	bool fallbackToSlam = true;
	bool enableAngularVelocity = false;
	bool continuousSync = true;
	float predictionTime = 1.0f;

	bool headFilterEnabled = false;
	protocol::OneEuroParams headFilterParams = { 5.0, 0.8, 1.0 };
	protocol::OneEuroParams driftFilterParams = { 3.0, 1.3, 0.6 };

	vr::VRNotificationId notificationId = 0;

	// Live guidance shown during the sampling phase: what motion the solver still
	// needs. Level 0 = on track (green), 1 = add this motion (yellow), 2 = slow down
	// because samples are being dropped (orange).
	std::string sampleHint;
	int sampleHintLevel = 0;

	enum Speed
	{
		FAST = 0,
		SLOW = 1,
		VERY_SLOW = 2
	};
	// Prefer Slow for Quest/VD wireless latency; Fast finishes before scale is observed.
	Speed calibrationSpeed = SLOW;

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
		targetModelScale = 1.0;
		hmdScale = 1.0;
		relativeRotation = { 1, 0, 0, 0 };
		relativeTranslation = { 0, 0, 0 };
		validRelativeOffset = false;
		leverSamples = 0;
		leverSerial = "";
		targetTrackingSystem = "";
		hmdSerial = "";
		trackerSerial = "";
		enabled = false;
		validProfile = false;
		continuousSync = true;
	}

	// (SampleCount() removed: nothing read it. Its values were still being maintained,
	// and its comment called it a "cap" — the real ceiling is kMaxTotalSamples in
	// Calibration.cpp. Finish is gated on stations + spread + axis variance.)

	// Distinct occupied poses required (one of three finish gates). A station is a
	// pose within a position/orientation radius; micro-yaw used to mint stations and
	// fill the bar while translation (scale/lever) stayed unobserved. Station radii
	// were widened and finish also requires positional spread + rotation variety.
	size_t StationTarget()
	{
		switch (calibrationSpeed)
		{
		case FAST:
			return 16;
		case SLOW:
			return 22;
		case VERY_SLOW:
			return 30;
		}
		return 16;
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
void LoadChaperoneBounds();
void ApplyChaperoneBounds();
void SendOneEuroParams();