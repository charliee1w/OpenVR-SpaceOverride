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
		targetModelScale = 1.0;
		hmdScale = 1.0;
		relativeRotation = { 1, 0, 0, 0 };
		relativeTranslation = { 0, 0, 0 };
		validRelativeOffset = false;
		targetTrackingSystem = "";
		hmdSerial = "";
		trackerSerial = "";
		enabled = false;
		validProfile = false;
		continuousSync = true;
	}

	// Sample targets. These used to be 100/250/500, which was far more than the solve
	// needs: samples are only taken while nearly still, so a head turning in one spot
	// produces hundreds of near-duplicates that add no information. The cost was paid
	// in user patience -- and worse, the time spent standing still grinding out the
	// count is time not spent covering the play space, which is the one thing scale
	// observability actually requires. Halving them buys back that time.
	size_t SampleCount()
	{
		switch (calibrationSpeed)
		{
		case FAST:
			return 60;
		case SLOW:
			return 140;
		case VERY_SLOW:
			return 250;
		}
		return 60;
	}

	// Distinct occupied poses required to finish. Progress is counted in these rather
	// than in raw samples: sampling runs at 20 Hz while nearly still, so a sample count
	// mostly measures how long you stood there, and could be satisfied without ever
	// moving. A station is capped at a handful of samples, so this can only be reached
	// by looking somewhere new or standing somewhere new.
	size_t StationTarget()
	{
		switch (calibrationSpeed)
		{
		case FAST:
			return 12;
		case SLOW:
			return 18;
		case VERY_SLOW:
			return 25;
		}
		return 12;
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