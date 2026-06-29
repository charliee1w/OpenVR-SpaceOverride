// SPDX-License-Identifier: AGPL-3.0-only

#define WIN32_LEAN_AND_MEAN

#include "Calibration.h"
#include "Configuration.h"
#include "IPCClient.h"
#include "calibration/Math.h"

#include <string>
#include <vector>
#include <iostream> 
#include <algorithm>
#include <cmath>
#include <ctime>

#include <Dense>


static IPCClient Driver;
CalibrationContext CalCtx;

static void AbortAndRestoreProfile(CalibrationContext& ctx);
static std::vector<uint32_t> FindTrackerIdsBySerial(const CalibrationContext& ctx);

void InitCalibrator()
{
	Driver.Connect();
}

static calibration::Pose PoseFromMatrix(vr::HmdMatrix34_t hmdMatrix)
{
	Eigen::Matrix3d rot;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			rot(i, j) = hmdMatrix.m[i][j];
		}
	}
	return calibration::Pose(rot, Eigen::Vector3d(hmdMatrix.m[0][3], hmdMatrix.m[1][3], hmdMatrix.m[2][3]));
}

bool StartsWith(const std::string &str, const std::string &prefix)
{
	if (str.length() < prefix.length())
		return false;

	return str.compare(0, prefix.length(), prefix) == 0;
}

bool EndsWith(const std::string &str, const std::string &suffix)
{
	if (str.length() < suffix.length())
		return false;

	return str.compare(str.length() - suffix.length(), suffix.length(), suffix) == 0;
}

struct DetectionState
{
	std::vector<uint32_t> candidates;
	std::vector<std::vector<double>> candidateAngSpeeds;
	std::vector<std::vector<double>> candidateLinSpeeds;
	std::vector<double> hmdAngSpeeds;
	std::vector<double> hmdLinSpeeds;
	std::vector<Eigen::Matrix3d> prevRot; // [0] = HMD, [i+1] = candidates[i]
	std::vector<Eigen::Vector3d> prevPos; // [0] = HMD, [i+1] = candidates[i]
	bool havePrev = false;
	double prevTime = 0;

	void Clear()
	{
		candidates.clear();
		candidateAngSpeeds.clear();
		candidateLinSpeeds.clear();
		hmdAngSpeeds.clear();
		hmdLinSpeeds.clear();
		prevRot.clear();
		prevPos.clear();
		havePrev = false;
		prevTime = 0;
	}
};

static DetectionState Detection;

static std::string GetDeviceSerial(uint32_t id)
{
	char serial[vr::k_unMaxPropertyStringSize] = {};
	vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_SerialNumber_String, serial, vr::k_unMaxPropertyStringSize);
	return std::string(serial);
}

static std::string GetDeviceTrackingSystem(uint32_t id)
{
	char system[vr::k_unMaxPropertyStringSize] = {};
	vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_TrackingSystemName_String, system, vr::k_unMaxPropertyStringSize);
	return std::string(system);
}

static double AngularSpeedBetween(const Eigen::Matrix3d &cur, const Eigen::Matrix3d &prev, double dt)
{
	Eigen::Matrix3d delta = cur * prev.transpose();
	double c = (delta(0,0) + delta(1,1) + delta(2,2) - 1.0) / 2.0;
	if (c > 1.0) c = 1.0;
	if (c < -1.0) c = -1.0;
	return acos(c) / dt;
}

static double LinearSpeedBetween(const Eigen::Vector3d& cur, const Eigen::Vector3d& prev, double dt)
{
	return (cur - prev).norm() / dt;
}

static double PearsonCorrelation(const std::vector<double> &a, const std::vector<double> &b)
{
	if (a.size() != b.size() || a.empty())
		return 0.0;

	double meanA = 0, meanB = 0;
	for (size_t i = 0; i < a.size(); i++) { meanA += a[i]; meanB += b[i]; }
	meanA /= a.size();
	meanB /= b.size();

	double cov = 0, varA = 0, varB = 0;
	for (size_t i = 0; i < a.size(); i++)
	{
		double da = a[i] - meanA, db = b[i] - meanB;
		cov += da * db;
		varA += da * da;
		varB += db * db;
	}

	if (varA < 1e-9 || varB < 1e-9)
		return 0.0;

	return cov / std::sqrt(varA * varB);
}

calibration::Sample CollectSample(const CalibrationContext &ctx)
{
	vr::TrackedDevicePose_t reference, target;
	reference.bPoseIsValid = false;
	target.bPoseIsValid = false;

	reference = ctx.devicePoses[0];
	target = ctx.devicePoses[ctx.targetID];

	bool ok = true;
	if (!reference.bPoseIsValid)
	{
		CalCtx.Log("Reference device is not tracking\n"); ok = false;
	}
	if (!target.bPoseIsValid)
	{
		CalCtx.Log("Target device is not tracking\n"); ok = false;
	}
	if (!ok)
	{
		CalCtx.Log("Aborting calibration!\n");
		CalCtx.state = CalibrationState::None;
		return calibration::Sample();
	}

	return calibration::Sample(
		PoseFromMatrix(reference.mDeviceToAbsoluteTracking),
		PoseFromMatrix(target.mDeviceToAbsoluteTracking)
	);
}

vr::HmdQuaternion_t VRRotationQuat(Eigen::Vector3d eulerdeg)
{
	auto euler = eulerdeg * EIGEN_PI / 180.0;

	Eigen::Quaterniond rotQuat =
		Eigen::AngleAxisd(euler(0), Eigen::Vector3d::UnitZ()) *
		Eigen::AngleAxisd(euler(1), Eigen::Vector3d::UnitY()) *
		Eigen::AngleAxisd(euler(2), Eigen::Vector3d::UnitX());

	vr::HmdQuaternion_t vrRotQuat;
	vrRotQuat.x = rotQuat.coeffs()[0];
	vrRotQuat.y = rotQuat.coeffs()[1];
	vrRotQuat.z = rotQuat.coeffs()[2];
	vrRotQuat.w = rotQuat.coeffs()[3];
	return vrRotQuat;
}

vr::HmdVector3d_t VRTranslationVec(Eigen::Vector3d transcm)
{
	auto trans = transcm * 0.01;
	vr::HmdVector3d_t vrTrans;
	vrTrans.v[0] = trans[0];
	vrTrans.v[1] = trans[1];
	vrTrans.v[2] = trans[2];
	return vrTrans;
}

static bool IpcResponseOk(const protocol::Response& response)
{
	switch (response.type)
	{
	case protocol::ResponseSuccess:
	case protocol::ResponseHandshake:
	case protocol::ResponsePredictionTelemetry:
	case protocol::ResponseDriverTelemetry:
		return true;
	case protocol::ResponseError:
		std::cerr << "IPC request rejected by driver (error " << response.errorCode << ")" << std::endl;
		return false;
	default:
		std::cerr << "IPC request returned unexpected response type " << response.type << std::endl;
		return false;
	}
}

static bool TrySendBlocking(const protocol::Request& req, protocol::Response* out = nullptr)
{
	try
	{
		auto response = Driver.SendBlocking(req);
		if (out)
			*out = response;
		if (!IpcResponseOk(response))
		{
			CalCtx.ipcHealthy = false;
			return false;
		}
		CalCtx.ipcHealthy = true;
		return true;
	}
	catch (const std::runtime_error& e)
	{
		std::cerr << "IPC request failed: " << e.what() << std::endl;
		CalCtx.ipcHealthy = false;
		return false;
	}
}

static bool IsTundraTrackerModel(uint32_t deviceId)
{
	if (!vr::VRSystem())
		return false;

	char model[vr::k_unMaxPropertyStringSize] = {};
	vr::VRSystem()->GetStringTrackedDeviceProperty(deviceId, vr::Prop_ModelNumber_String, model, sizeof(model));
	std::string modelLower(model);
	std::transform(modelLower.begin(), modelLower.end(), modelLower.begin(),
		[](unsigned char c) { return (char)std::tolower(c); });
	return modelLower.find("tundra") != std::string::npos;
}

static void ApplyTundraFilterDefaultsIfNeeded(CalibrationContext& ctx, uint32_t trackerId)
{
	ctx.trackerIsTundra = IsTundraTrackerModel(trackerId);
	if (!ctx.trackerIsTundra)
		return;

	ctx.headFilterEnabled = true;
	ctx.headFilterParams = filter_defaults::TundraHead;
	ctx.driftFilterParams = filter_defaults::TundraDrift;

	char buf[256];
	snprintf(buf, sizeof buf,
		"Tundra tracker detected — applying stronger smoothing defaults for head mount.\n");
	ctx.Log(buf);
}

static void UpdateLiveCalibrationQuality(CalibrationContext& ctx, const std::vector<calibration::Sample>& samples)
{
	if (samples.size() < CalibrationContext::LiveQualityMinSamples)
	{
		ctx.liveCalibrationQualityValid = false;
		ctx.liveCalibrationRmsMm = 0.0;
		return;
	}

	const auto weights = calibration::computeSampleWeights(samples);

	if (ctx.state == CalibrationState::PartialSampling && ctx.validProfile)
	{
		const Eigen::Matrix3d calRot = calibration::eulerDegreesToRotationMatrix(ctx.calibratedRotation);
		const Eigen::Vector3d calTransM = ctx.calibratedTranslation * 0.01;
		const auto mount = calibration::computeRelativeOffsetWeighted(samples, calRot, calTransM);
		const double rms = calibration::mountOffsetErrorRMSWeighted(samples, calRot, calTransM, mount, weights);
		ctx.liveCalibrationRmsMm = rms * 1000.0;
		ctx.liveCalibrationQualityValid = true;
		return;
	}

	if (ctx.state == CalibrationState::Sampling)
	{
		const auto rotResult = calibration::calibrateRotation(samples);
		if (rotResult.deltaCount == 0)
		{
			ctx.liveCalibrationQualityValid = false;
			return;
		}

		const Eigen::Matrix3d calRot = calibration::eulerDegreesToRotationMatrix(rotResult.eulerDegrees);
		const Eigen::Vector3d calTransM = calibration::calibrateTranslation(samples, calRot) * 0.01;
		const Eigen::Vector3d hmdToTarget = calibration::computeRefToTargetOffset(samples, calRot, calTransM);
		const double rms = calibration::retargetingErrorRMSWeighted(samples, hmdToTarget, calRot, calTransM, weights);
		ctx.liveCalibrationRmsMm = rms * 1000.0;
		ctx.liveCalibrationQualityValid = true;
	}
}

static calibration::RelativeOffsetResult MountFromContext(const CalibrationContext& ctx)
{
	calibration::RelativeOffsetResult mount;
	mount.rotation = Eigen::Quaterniond(
		ctx.relativeRotation.w,
		ctx.relativeRotation.x,
		ctx.relativeRotation.y,
		ctx.relativeRotation.z);
	mount.translation = Eigen::Vector3d(
		ctx.relativeTranslation.v[0],
		ctx.relativeTranslation.v[1],
		ctx.relativeTranslation.v[2]);
	return mount;
}

static bool g_runtimeQualityPrevValid = false;
static calibration::Pose g_runtimePrevSlam;
static calibration::Pose g_runtimePrevTracker;

void UpdateRuntimeTrackingQuality(CalibrationContext& ctx)
{
	ctx.runtimeResidualValid = false;
	ctx.guardianShiftSuspect = false;

	if (!ctx.validProfile || !ctx.validRelativeOffset)
		return;

	const auto trackerMatches = FindTrackerIdsBySerial(ctx);
	if (trackerMatches.size() != 1)
		return;

	const uint32_t trackerID = trackerMatches.front();
	const auto& hmdPose = ctx.devicePoses[vr::k_unTrackedDeviceIndex_Hmd];
	const auto& trackerPose = ctx.devicePoses[trackerID];
	if (!hmdPose.bPoseIsValid || !trackerPose.bPoseIsValid)
		return;

	const calibration::Pose slamHmd = PoseFromMatrix(hmdPose.mDeviceToAbsoluteTracking);
	const calibration::Pose tracker = PoseFromMatrix(trackerPose.mDeviceToAbsoluteTracking);
	const Eigen::Matrix3d calRot = calibration::eulerDegreesToRotationMatrix(ctx.calibratedRotation);
	const Eigen::Vector3d calTransM = ctx.calibratedTranslation * 0.01;
	const auto mount = MountFromContext(ctx);

	ctx.runtimeResidualMm = calibration::runtimeMountResidualMm(slamHmd, tracker, calRot, calTransM, mount);
	ctx.runtimeResidualValid = true;

	if (g_runtimeQualityPrevValid)
	{
		const auto shift = calibration::detectGuardianShift(g_runtimePrevSlam, slamHmd, g_runtimePrevTracker, tracker);
		ctx.guardianShiftSuspect = shift.suspect;
		ctx.guardianShiftSlamJumpMm = shift.slamJumpMm;
	}

	if (ctx.guardianShiftSuspect)
		ctx.runtimeResidualHighStreak = 0;
	else if (ctx.runtimeResidualMm > CalibrationContext::RuntimeResidualWarnMm)
		ctx.runtimeResidualHighStreak++;
	else
		ctx.runtimeResidualHighStreak = 0;

	g_runtimePrevSlam = slamHmd;
	g_runtimePrevTracker = tracker;
	g_runtimeQualityPrevValid = true;
}

void ResetAndDisableOffsets(uint32_t id)
{
	vr::HmdVector3d_t zeroV;
	zeroV.v[0] = zeroV.v[1] = zeroV.v[2] = 0;

	vr::HmdQuaternion_t zeroQ;
	zeroQ.x = 0; zeroQ.y = 0; zeroQ.z = 0; zeroQ.w = 1;

	protocol::Request req(protocol::RequestSetDeviceTransform);
	req.setDeviceTransform = { id, false, zeroV, zeroQ, 1.0 };
	TrySendBlocking(req);
}

void SendOneEuroParams()
{
	protocol::Request req(protocol::RequestSetOneEuro);
	req.setOneEuro.headEnabled = CalCtx.headFilterEnabled;
	req.setOneEuro.head = CalCtx.headFilterParams;
	req.setOneEuro.drift = CalCtx.driftFilterParams;

	try
	{
		Driver.SendBlocking(req);
	}
	catch (const std::runtime_error &e)
	{
		std::cerr << "Failed to send One Euro params: " << e.what() << std::endl;
	}
}

void FetchPredictionTelemetry()
{
	protocol::Request req(protocol::RequestGetPredictionTelemetry);
	try
	{
		auto response = Driver.SendBlocking(req);
		if (response.type != protocol::ResponsePredictionTelemetry)
			return;

		CalCtx.predictionTelemetryValid = response.predictionTelemetry.valid;
		CalCtx.predictionLagMs = response.predictionTelemetry.estimatedLagMs;
		CalCtx.predictionLagFrames = response.predictionTelemetry.estimatedLagFrames;
		CalCtx.autoPredictionFrames = response.predictionTelemetry.appliedPredictionFrames;
	}
	catch (const std::runtime_error& e)
	{
		std::cerr << "Failed to fetch prediction telemetry: " << e.what() << std::endl;
	}
}

void FetchDriverTelemetry()
{
	protocol::Request req(protocol::RequestGetDriverTelemetry);
	try
	{
		auto response = Driver.SendBlocking(req);
		if (response.type != protocol::ResponseDriverTelemetry)
			return;

		CalCtx.driverTelemetry = response.driverTelemetry;
		CalCtx.driverTelemetryValid = true;
	}
	catch (const std::runtime_error& e)
	{
		std::cerr << "Failed to fetch driver telemetry: " << e.what() << std::endl;
		CalCtx.driverTelemetryValid = false;
	}
}

static protocol::SetHmdTracker BuildHmdTrackerCommand(uint32_t hmdID, uint32_t trackerID, bool enabled)
{
	protocol::SetHmdTracker cmd{};
	cmd.hmdID = hmdID;
	cmd.trackerID = trackerID;
	cmd.enabled = enabled;
	cmd.native = CalCtx.enableNative;
	cmd.slamFallback = CalCtx.fallbackToSlam;
	cmd.predictionTime = CalCtx.predictionTime;
	cmd.predictionAuto = CalCtx.predictionAuto;
	cmd.enableAngularVelocity = CalCtx.enableAngularVelocity;
	cmd.offsetRotation = CalCtx.relativeRotation;
	cmd.offsetTranslation = CalCtx.relativeTranslation;
	cmd.calibrationRotation = VRRotationQuat(CalCtx.calibratedRotation);
	cmd.calibrationTranslation = VRTranslationVec(CalCtx.calibratedTranslation);
	return cmd;
}

void SendHmdTrackerCommand(uint32_t hmdID, uint32_t trackerID, bool enabled)
{
	protocol::Request req(protocol::RequestSetHmdTracker);
	req.setHmdTracker = BuildHmdTrackerCommand(hmdID, trackerID, enabled);
	TrySendBlocking(req);
}

namespace
{
	struct AppliedDeviceTransform
	{
		bool known = false;
		bool enabled = false;
		vr::HmdVector3d_t translation{};
		vr::HmdQuaternion_t rotation{ 1, 0, 0, 0 };
		double scale = 1.0;
	};

	struct AppliedDriverState
	{
		bool initialized = false;
		AppliedDeviceTransform deviceTransforms[vr::k_unMaxTrackedDeviceCount]{};
		bool slamSyncKnown[vr::k_unMaxTrackedDeviceCount]{};
		bool slamSync[vr::k_unMaxTrackedDeviceCount]{};
		protocol::SetHmdTracker hmdTracker{};
		bool hmdTrackerKnown = false;
		protocol::SetOneEuro oneEuro{};
		bool oneEuroKnown = false;
		double lastTelemetryFetchTime = 0.0;
	};

	static AppliedDriverState g_applied;

	static bool ApproximatelyEqual(double a, double b, double epsilon = 1e-9)
	{
		return std::abs(a - b) <= epsilon;
	}

	static bool VecApproximatelyEqual(const vr::HmdVector3d_t& a, const vr::HmdVector3d_t& b)
	{
		return ApproximatelyEqual(a.v[0], b.v[0])
			&& ApproximatelyEqual(a.v[1], b.v[1])
			&& ApproximatelyEqual(a.v[2], b.v[2]);
	}

	static bool QuatApproximatelyEqual(const vr::HmdQuaternion_t& a, const vr::HmdQuaternion_t& b)
	{
		const bool same = ApproximatelyEqual(a.w, b.w)
			&& ApproximatelyEqual(a.x, b.x)
			&& ApproximatelyEqual(a.y, b.y)
			&& ApproximatelyEqual(a.z, b.z);
		const bool negated = ApproximatelyEqual(a.w, -b.w)
			&& ApproximatelyEqual(a.x, -b.x)
			&& ApproximatelyEqual(a.y, -b.y)
			&& ApproximatelyEqual(a.z, -b.z);
		return same || negated;
	}

	static bool OneEuroApproximatelyEqual(const protocol::OneEuroParams& a, const protocol::OneEuroParams& b)
	{
		return ApproximatelyEqual(a.minCutoff, b.minCutoff)
			&& ApproximatelyEqual(a.beta, b.beta)
			&& ApproximatelyEqual(a.dCutoff, b.dCutoff);
	}

	static bool DeviceTransformApproximatelyEqual(const AppliedDeviceTransform& a, const protocol::SetDeviceTransform& b)
	{
		return a.enabled == b.enabled
			&& (!b.enabled || (
				VecApproximatelyEqual(a.translation, b.translation)
				&& QuatApproximatelyEqual(a.rotation, b.rotation)
				&& ApproximatelyEqual(a.scale, b.scale)));
	}

	static bool HmdTrackerApproximatelyEqual(const protocol::SetHmdTracker& a, const protocol::SetHmdTracker& b)
	{
		return a.hmdID == b.hmdID
			&& a.trackerID == b.trackerID
			&& a.enabled == b.enabled
			&& a.native == b.native
			&& a.slamFallback == b.slamFallback
			&& a.enableAngularVelocity == b.enableAngularVelocity
			&& ApproximatelyEqual(a.predictionTime, b.predictionTime)
			&& a.predictionAuto == b.predictionAuto
			&& QuatApproximatelyEqual(a.offsetRotation, b.offsetRotation)
			&& VecApproximatelyEqual(a.offsetTranslation, b.offsetTranslation)
			&& QuatApproximatelyEqual(a.calibrationRotation, b.calibrationRotation)
			&& VecApproximatelyEqual(a.calibrationTranslation, b.calibrationTranslation);
	}

	static bool OneEuroCommandApproximatelyEqual(const protocol::SetOneEuro& a, const protocol::SetOneEuro& b)
	{
		return a.headEnabled == b.headEnabled
			&& OneEuroApproximatelyEqual(a.head, b.head)
			&& OneEuroApproximatelyEqual(a.drift, b.drift);
	}

	static void RememberDeviceTransform(uint32_t id, const protocol::SetDeviceTransform& transform)
	{
		auto& applied = g_applied.deviceTransforms[id];
		applied.known = true;
		applied.enabled = transform.enabled;
		applied.translation = transform.translation;
		applied.rotation = transform.rotation;
		applied.scale = transform.scale;
	}

	static protocol::SetOneEuro BuildOneEuroCommand()
	{
		protocol::SetOneEuro cmd{};
		cmd.headEnabled = CalCtx.headFilterEnabled;
		cmd.head = CalCtx.headFilterParams;
		cmd.drift = CalCtx.driftFilterParams;
		return cmd;
	}

	static protocol::SetDeviceTransform MakeDisabledDeviceTransform(uint32_t id)
	{
		vr::HmdVector3d_t zeroV{};
		zeroV.v[0] = zeroV.v[1] = zeroV.v[2] = 0;
		vr::HmdQuaternion_t zeroQ{};
		zeroQ.x = 0;
		zeroQ.y = 0;
		zeroQ.z = 0;
		zeroQ.w = 1;
		return { id, false, zeroV, zeroQ, 1.0 };
	}

	static bool TrySendBatch(const protocol::ApplyBatch& batch)
	{
		if (batch.deviceTransformCount == 0
			&& batch.slamSyncCount == 0
			&& !batch.applyHmdTracker
			&& !batch.applyOneEuro)
		{
			return true;
		}

		protocol::Request req(protocol::RequestApplyBatch);
		req.applyBatch = batch;
		return TrySendBlocking(req);
	}

	static void RememberBatchState(const protocol::ApplyBatch& batch)
	{
		for (uint32_t i = 0; i < batch.deviceTransformCount; ++i)
			RememberDeviceTransform(batch.deviceTransforms[i].openVRID, batch.deviceTransforms[i]);

		for (uint32_t i = 0; i < batch.slamSyncCount; ++i)
		{
			const uint32_t id = batch.slamSync[i].openVRID;
			if (id < protocol::MaxTrackedDevices)
			{
				g_applied.slamSyncKnown[id] = true;
				g_applied.slamSync[id] = batch.slamSync[i].enabled;
			}
		}

		if (batch.applyHmdTracker)
		{
			g_applied.hmdTracker = batch.hmdTracker;
			g_applied.hmdTrackerKnown = true;
		}

		if (batch.applyOneEuro)
		{
			g_applied.oneEuro = batch.oneEuro;
			g_applied.oneEuroKnown = true;
		}
	}

	static bool TryIpcHealthCheck()
	{
		protocol::Request req(protocol::RequestHandshake);
		protocol::Response resp;
		if (!TrySendBlocking(req, &resp))
			return false;
		return resp.type == protocol::ResponseHandshake && resp.protocol.version == protocol::Version;
	}
}

void InvalidateAppliedDriverState()
{
	g_applied = {};
}

static void ApplyRelativeOffset(CalibrationContext &ctx, const std::vector<calibration::Sample> &samples, const Eigen::Matrix3d &calRot, const Eigen::Vector3d &calTrans)
{
	auto result = calibration::computeRelativeOffsetWeighted(samples, calRot, calTrans);
	if (samples.empty())
		return;

	ctx.relativeRotation.w = result.rotation.w();
	ctx.relativeRotation.x = result.rotation.x();
	ctx.relativeRotation.y = result.rotation.y();
	ctx.relativeRotation.z = result.rotation.z();
	ctx.relativeTranslation.v[0] = result.translation.x();
	ctx.relativeTranslation.v[1] = result.translation.y();
	ctx.relativeTranslation.v[2] = result.translation.z();
	ctx.validRelativeOffset = true;
}

static_assert(vr::k_unTrackedDeviceIndex_Hmd == 0, "HMD index expected to be 0");

const char* OverrideInactiveReasonText(OverrideInactiveReason reason)
{
	switch (reason)
	{
	case OverrideInactiveReason::Active:
		return "Override active";
	case OverrideInactiveReason::NoProfile:
		return "No calibration profile";
	case OverrideInactiveReason::TrackerMissing:
		return "Headset tracker not connected";
	case OverrideInactiveReason::TrackerSerialAmbiguous:
		return "Multiple trackers share the saved serial";
	case OverrideInactiveReason::HmdSerialMismatch:
		return "HMD serial changed since calibration";
	case OverrideInactiveReason::HmdTrackingSystemMismatch:
		return "HMD tracking system changed since calibration";
	case OverrideInactiveReason::InvalidRelativeOffset:
		return "Calibration mount offset is invalid";
	}
	return "Override inactive";
}

const char* DriverOverrideInactiveReasonText(protocol::DriverOverrideInactiveReason reason)
{
	switch (reason)
	{
	case protocol::DriverOverrideActive:
		return "active";
	case protocol::DriverOverrideDisabled:
		return "disabled by overlay";
	case protocol::DriverOverrideHooksMissing:
		return "pose hooks not installed";
	case protocol::DriverOverrideTrackerInvalid:
		return "invalid tracker ID";
	case protocol::DriverOverrideTrackerLost:
		return "tracker pose lost";
	}
	return "unknown";
}

bool IsStageTrackingSpace()
{
	if (!vr::VRCompositor())
		return false;

	return vr::VRCompositor()->GetTrackingSpace() == vr::TrackingUniverseStanding;
}

static bool IsSlamTrackingSystem(const std::string& trackingSystem)
{
	return trackingSystem == "oculus"
		|| trackingSystem == "holographic"
		|| StartsWith(trackingSystem, "window");
}

const char* ChaperoneCalibrationStateText(vr::ChaperoneCalibrationState state)
{
	switch (state)
	{
	case vr::ChaperoneCalibrationState_OK: return "OK";
	case vr::ChaperoneCalibrationState_Warning: return "Warning";
	case vr::ChaperoneCalibrationState_Warning_BaseStationMayHaveMoved: return "Warning: base station moved";
	case vr::ChaperoneCalibrationState_Warning_BaseStationRemoved: return "Warning: base station removed";
	case vr::ChaperoneCalibrationState_Warning_SeatedBoundsInvalid: return "Warning: seated bounds invalid";
	case vr::ChaperoneCalibrationState_Error: return "Error";
	case vr::ChaperoneCalibrationState_Error_BaseStationUninitialized: return "Error: base station uninitialized";
	case vr::ChaperoneCalibrationState_Error_BaseStationConflict: return "Error: base station conflict";
	case vr::ChaperoneCalibrationState_Error_PlayAreaInvalid: return "Error: play area invalid";
	case vr::ChaperoneCalibrationState_Error_CollisionBoundsInvalid: return "Error: collision bounds invalid";
	default: return "Unknown";
	}
}

const char* GuardianBoundaryStateText(GuardianBoundaryState state, bool slamHmd)
{
	switch (state)
	{
	case GuardianBoundaryState::Active:
		return slamHmd ? "Quest guardian: ON (bounds in SteamVR)" : "Chaperone bounds: active";
	case GuardianBoundaryState::DisabledLikely:
		return slamHmd
			? "Quest guardian: OFF (likely disabled in Quest dev settings)"
			: "Chaperone bounds: none (disabled or not configured)";
	case GuardianBoundaryState::NotConfigured:
		return slamHmd ? "Quest guardian: not configured (no bounds)" : "Chaperone bounds: not configured";
	case GuardianBoundaryState::Error:
		return slamHmd ? "Quest guardian: error / invalid" : "Chaperone bounds: error";
	default:
		return slamHmd ? "Quest guardian: unknown" : "Chaperone bounds: unknown";
	}
}

void UpdateGuardianBoundaryStatus(CalibrationContext& ctx)
{
	ctx.guardianBoundary = {};

	if (!vr::VRSystem() || !vr::VRChaperone() || !vr::VRChaperoneSetup())
		return;

	auto& status = ctx.guardianBoundary;
	status.valid = true;

	const std::string hmdSystem = GetDeviceTrackingSystem(vr::k_unTrackedDeviceIndex_Hmd);
	status.slamHmd = IsSlamTrackingSystem(hmdSystem);
	status.calibrationState = vr::VRChaperone()->GetCalibrationState();
	status.boundsVisible = vr::VRChaperone()->AreBoundsVisible();

	uint32_t quadCount = 0;
	vr::VRChaperoneSetup()->GetLiveCollisionBoundsInfo(nullptr, &quadCount);
	status.collisionQuadCount = quadCount;

	float sizeX = 0.0f;
	float sizeZ = 0.0f;
	if (vr::VRChaperone()->GetPlayAreaSize(&sizeX, &sizeZ))
	{
		status.playAreaWidthM = sizeX;
		status.playAreaDepthM = sizeZ;
	}

	const bool noCollisionBounds = quadCount == 0;
	const bool noPlayArea = sizeX < 0.01f && sizeZ < 0.01f;
	const bool calError = status.calibrationState >= vr::ChaperoneCalibrationState_Error;

	if (status.slamHmd && noCollisionBounds && noPlayArea)
		status.state = GuardianBoundaryState::DisabledLikely;
	else if (noCollisionBounds && noPlayArea)
		status.state = GuardianBoundaryState::NotConfigured;
	else if (calError && noCollisionBounds)
		status.state = GuardianBoundaryState::Error;
	else if (!noCollisionBounds || !noPlayArea)
		status.state = GuardianBoundaryState::Active;
	else
		status.state = GuardianBoundaryState::Unknown;
}

static std::vector<uint32_t> FindTrackerIdsBySerial(const CalibrationContext& ctx)
{
	std::vector<uint32_t> matches;
	if (ctx.trackerSerial.empty() || !vr::VRSystem())
		return matches;

	for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
	{
		if (vr::VRSystem()->GetTrackedDeviceClass(id) == vr::TrackedDeviceClass_Invalid)
			continue;
		if (GetDeviceSerial(id) == ctx.trackerSerial)
			matches.push_back(id);
	}

	return matches;
}

OverrideStatus EvaluateOverrideStatus(const CalibrationContext& ctx)
{
	OverrideStatus status;

	if (!ctx.validProfile)
	{
		status.inactiveReason = OverrideInactiveReason::NoProfile;
		return status;
	}

	if (!ctx.validRelativeOffset)
	{
		status.inactiveReason = OverrideInactiveReason::InvalidRelativeOffset;
		return status;
	}

	if (!vr::VRSystem())
	{
		status.inactiveReason = OverrideInactiveReason::TrackerMissing;
		return status;
	}

	auto trackerMatches = FindTrackerIdsBySerial(ctx);
	if (trackerMatches.size() > 1)
	{
		status.inactiveReason = OverrideInactiveReason::TrackerSerialAmbiguous;
		return status;
	}

	uint32_t trackerID = trackerMatches.empty()
		? vr::k_unTrackedDeviceIndexInvalid
		: trackerMatches.front();

	if (trackerID == vr::k_unTrackedDeviceIndexInvalid)
	{
		status.inactiveReason = OverrideInactiveReason::TrackerMissing;
		return status;
	}

	if (!ctx.hmdSerial.empty())
	{
		std::string liveHmdSerial = GetDeviceSerial(vr::k_unTrackedDeviceIndex_Hmd);
		if (!liveHmdSerial.empty() && liveHmdSerial != ctx.hmdSerial)
		{
			status.inactiveReason = OverrideInactiveReason::HmdSerialMismatch;
			return status;
		}
	}

	if (!ctx.hmdTrackingSystem.empty())
	{
		std::string liveHmdSystem = GetDeviceTrackingSystem(vr::k_unTrackedDeviceIndex_Hmd);
		if (!liveHmdSystem.empty() && liveHmdSystem != ctx.hmdTrackingSystem)
		{
			status.inactiveReason = OverrideInactiveReason::HmdTrackingSystemMismatch;
			return status;
		}
	}

	status.active = true;
	status.inactiveReason = OverrideInactiveReason::Active;
	return status;
}

static void ScanAndApplyProfile(CalibrationContext &ctx, double timeSec)
{
	char buffer[vr::k_unMaxPropertyStringSize];
	ctx.enabled = ctx.validProfile;

	if (ctx.enabled)
	{
		auto trackerMatches = FindTrackerIdsBySerial(ctx);
		ctx.targetID = trackerMatches.size() == 1
			? trackerMatches.front()
			: vr::k_unTrackedDeviceIndexInvalid;
	}

	const bool forceApply = !g_applied.initialized;
	bool anyDriverUpdate = false;
	protocol::ApplyBatch batch{};

	for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
	{
		auto deviceClass = vr::VRSystem()->GetTrackedDeviceClass(id);
		if (deviceClass == vr::TrackedDeviceClass_Invalid)
		{
			g_applied.deviceTransforms[id] = {};
			g_applied.slamSyncKnown[id] = false;
			continue;
		}

		protocol::SetDeviceTransform desiredTransform(id, false);
		bool wantTransformEnabled = false;

		if (ctx.enabled)
		{
			vr::ETrackedPropertyError err = vr::TrackedProp_Success;
			vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_TrackingSystemName_String, buffer, vr::k_unMaxPropertyStringSize, &err);

			if (err == vr::TrackedProp_Success)
			{
				std::string trackingSystem(buffer);

				if (id != vr::k_unTrackedDeviceIndex_Hmd
					&& trackingSystem == ctx.targetTrackingSystem)
				{
					wantTransformEnabled = true;
					desiredTransform = {
						id,
						true,
						VRTranslationVec(ctx.calibratedTranslation),
						VRRotationQuat(ctx.calibratedRotation),
						ctx.calibratedScale
					};
				}
			}
		}

		desiredTransform.enabled = wantTransformEnabled;
		const auto& appliedTransform = g_applied.deviceTransforms[id];
		if (forceApply
			|| !appliedTransform.known
			|| !DeviceTransformApproximatelyEqual(appliedTransform, desiredTransform))
		{
			if (batch.deviceTransformCount < protocol::MaxTrackedDevices)
			{
				batch.deviceTransforms[batch.deviceTransformCount++] = wantTransformEnabled
					? desiredTransform
					: MakeDisabledDeviceTransform(id);
			}
			else
			{
				std::cerr << "ScanAndApplyProfile: device transform batch truncated at device "
					<< id << " (max " << protocol::MaxTrackedDevices << ")" << std::endl;
			}
		}
	}

	ctx.overrideStatus = EvaluateOverrideStatus(ctx);
	const bool overrideActive = ctx.overrideStatus.active;

	for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
	{
		auto deviceClass = vr::VRSystem()->GetTrackedDeviceClass(id);
		if (deviceClass == vr::TrackedDeviceClass_Invalid)
			continue;

		bool sync = false;
		if (overrideActive)
		{
			sync = ctx.continuousSync
				&& (ctx.syncHmdDrift || id != vr::k_unTrackedDeviceIndex_Hmd)
				&& deviceClass != vr::TrackedDeviceClass_TrackingReference;

			if (sync)
			{
				vr::ETrackedPropertyError err = vr::TrackedProp_Success;
				vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_TrackingSystemName_String, buffer, vr::k_unMaxPropertyStringSize, &err);
				sync = err == vr::TrackedProp_Success && std::string(buffer) != ctx.targetTrackingSystem;
			}
		}

		if (forceApply
			|| !g_applied.slamSyncKnown[id]
			|| g_applied.slamSync[id] != sync)
		{
			if (batch.slamSyncCount < protocol::MaxTrackedDevices)
				batch.slamSync[batch.slamSyncCount++] = { id, sync };
			else
			{
				std::cerr << "ScanAndApplyProfile: slam sync batch truncated at device "
					<< id << " (max " << protocol::MaxTrackedDevices << ")" << std::endl;
			}
		}
	}

	const protocol::SetHmdTracker desiredHmdTracker = overrideActive
		? BuildHmdTrackerCommand(vr::k_unTrackedDeviceIndex_Hmd, ctx.targetID, true)
		: BuildHmdTrackerCommand(vr::k_unTrackedDeviceIndex_Hmd, vr::k_unTrackedDeviceIndexInvalid, false);

	if (forceApply
		|| !g_applied.hmdTrackerKnown
		|| !HmdTrackerApproximatelyEqual(g_applied.hmdTracker, desiredHmdTracker))
	{
		batch.applyHmdTracker = true;
		batch.hmdTracker = desiredHmdTracker;
	}

	const protocol::SetOneEuro desiredOneEuro = BuildOneEuroCommand();
	if (forceApply
		|| !g_applied.oneEuroKnown
		|| !OneEuroCommandApproximatelyEqual(g_applied.oneEuro, desiredOneEuro))
	{
		batch.applyOneEuro = true;
		batch.oneEuro = desiredOneEuro;
	}

	const bool hadBatchWork = batch.deviceTransformCount > 0
		|| batch.slamSyncCount > 0
		|| batch.applyHmdTracker
		|| batch.applyOneEuro;

	if (hadBatchWork)
	{
		if (TrySendBatch(batch))
		{
			RememberBatchState(batch);
			anyDriverUpdate = true;
			g_applied.initialized = true;
		}
		else if (!forceApply)
		{
			std::cerr << "ScanAndApplyProfile: batch apply failed — will retry" << std::endl;
		}
	}
	else
	{
		g_applied.initialized = true;
	}

	constexpr double kTelemetryIntervalSec = 5.0;
	if (anyDriverUpdate
		|| forceApply
		|| (timeSec - g_applied.lastTelemetryFetchTime) >= kTelemetryIntervalSec)
	{
		FetchPredictionTelemetry();
		g_applied.lastTelemetryFetchTime = timeSec;
	}

	if (ctx.enabled && ctx.chaperone.valid && ctx.chaperone.autoApply)
	{
		uint32_t quadCount = 0;
		vr::VRChaperoneSetup()->GetLiveCollisionBoundsInfo(nullptr, &quadCount);

		// Heuristic: when SteamVR resets to a blank-ish chaperone, it uses empty geometry,
		// but manual adjustments (e.g. via a play space mover) will not touch geometry.
		if (quadCount != ctx.chaperone.geometry.size())
		{
			ApplyChaperoneBounds();
		}
	}
}

void ApplyRuntimeDriverSettings(double timeSec)
{
	g_applied.hmdTrackerKnown = false;
	g_applied.oneEuroKnown = false;
	for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
		g_applied.slamSyncKnown[id] = false;

	ScanAndApplyProfile(CalCtx, timeSec);
}

static uint32_t FindTrackerBySerial(const CalibrationContext& ctx)
{
	auto matches = FindTrackerIdsBySerial(ctx);
	return matches.size() == 1 ? matches.front() : vr::k_unTrackedDeviceIndexInvalid;
}

static void BeginSamplingPhase(CalibrationContext &ctx, uint32_t targetID)
{
	ctx.targetID = targetID;
	ctx.targetTrackingSystem = GetDeviceTrackingSystem(targetID);
	ctx.hmdSerial = GetDeviceSerial(vr::k_unTrackedDeviceIndex_Hmd);
	ctx.hmdTrackingSystem = GetDeviceTrackingSystem(vr::k_unTrackedDeviceIndex_Hmd);
	ctx.trackerSerial = GetDeviceSerial(targetID);
	ctx.liveCalibrationQualityValid = false;
	ctx.liveCalibrationRmsMm = 0.0;

	char buf[256];
	snprintf(buf, sizeof buf, "Using headset tracker: %s (id %d)\n", ctx.trackerSerial.c_str(), targetID);
	ctx.Log(buf);

	ApplyTundraFilterDefaultsIfNeeded(ctx, targetID);

	ResetAndDisableOffsets(targetID);
	SendHmdTrackerCommand(vr::k_unTrackedDeviceIndex_Hmd, vr::k_unTrackedDeviceIndexInvalid, false);

	ctx.state = CalibrationState::Sampling;
	ctx.wantedUpdateInterval = 0.0;
	ctx.Log("Starting calibration...\n");
}

static void BeginPartialSamplingPhase(CalibrationContext& ctx, uint32_t targetID)
{
	ctx.targetID = targetID;

	char buf[256];
	snprintf(buf, sizeof buf, "Re-estimating mount offset for tracker %s (id %d)\n", ctx.trackerSerial.c_str(), targetID);
	ctx.Log(buf);

	ResetAndDisableOffsets(targetID);
	SendHmdTrackerCommand(vr::k_unTrackedDeviceIndex_Hmd, vr::k_unTrackedDeviceIndexInvalid, false);

	ctx.state = CalibrationState::PartialSampling;
	ctx.wantedUpdateInterval = 0.0;
	ctx.Log("Collecting mount offset samples — hold still or move smoothly.\n");
}

static std::vector<calibration::Sample> collectedSamples;
static int coplanarRetries = 0;
static int calibrationAutoAttempts = 0;
static CalibrationContext::Speed speedAtSessionStart = CalibrationContext::FAST;
static bool hadValidProfileAtCalStart = false;

const char* CalibrationSpeedName(CalibrationContext::Speed speed)
{
	switch (speed)
	{
	case CalibrationContext::FAST:
		return "Fast";
	case CalibrationContext::SLOW:
		return "Slow";
	case CalibrationContext::VERY_SLOW:
		return "Very Slow";
	}
	return "Unknown";
}

static bool TryAutoRetryFullCalibration(
	CalibrationContext& ctx,
	std::vector<calibration::Sample>& samples,
	double rmsMeters,
	CalibrationContext::Speed failedSpeed)
{
	if (calibrationAutoAttempts >= CalibrationContext::MaxCalibrationAutoAttempts)
		return false;

	calibrationAutoAttempts++;

	CalibrationContext::Speed retrySpeed = failedSpeed;
	if (retrySpeed < CalibrationContext::VERY_SLOW)
		retrySpeed = static_cast<CalibrationContext::Speed>(static_cast<int>(retrySpeed) + 1);

	ctx.calibrationSpeed = retrySpeed;

	char buf[256];
	snprintf(buf, sizeof buf,
		"Calibration RMS %.1f mm exceeds 100 mm at %s speed — auto-retry %d/%d at %s speed...\n",
		rmsMeters * 1000.0,
		CalibrationSpeedName(failedSpeed),
		calibrationAutoAttempts,
		CalibrationContext::MaxCalibrationAutoAttempts,
		CalibrationSpeedName(retrySpeed));
	ctx.Log(buf);

	samples.clear();
	coplanarRetries = 0;
	BeginSamplingPhase(ctx, ctx.targetID);
	return true;
}

static void FailFullCalibration(
	CalibrationContext& ctx,
	std::vector<calibration::Sample>& samples,
	double rmsMeters)
{
	char buf[256];
	snprintf(buf, sizeof buf,
		"Calibration failed after %d automatic attempt(s) — RMS %.1f mm at %s speed. Previous profile restored.\n",
		calibrationAutoAttempts,
		rmsMeters * 1000.0,
		CalibrationSpeedName(ctx.calibrationSpeed));
	ctx.Log(buf);

	ctx.lastCalibrationRmsMm = rmsMeters * 1000.0;
	ctx.calibrationFailureOffer = true;
	AbortAndRestoreProfile(ctx);
	ctx.calibrationSpeed = speedAtSessionStart;

	if (CalCtx.notificationId != 0)
	{
		vr::VRNotifications()->RemoveNotification(CalCtx.notificationId);
		CalCtx.notificationId = 0;
	}

	CalCtx.notificationId = ShowCalibrationNotification(
		"Calibration quality too low after automatic retries.\n"
		"Open Space Override and choose Retry or Restore previous.");

	samples.clear();
}

static void FinishFullCalibration(CalibrationContext& ctx, std::vector<calibration::Sample>& samples)
{
	double axisVariance = calibration::secondAxisVariance(samples);
	if (axisVariance < calibration::AxisVarianceThreshold)
	{
		if (++coplanarRetries >= 10)
		{
			CalCtx.Log("Not enough rotation variety after several attempts, aborting calibration! Previous calibration restored.\n");
			AbortAndRestoreProfile(ctx);
			return;
		}

		char buf[256];
		snprintf(buf, sizeof buf, "Head movement is too uniform (axis variance %.5f), tilt and turn your head in different directions! Collecting more samples...\n", axisVariance);
		CalCtx.Log(buf);
		samples.erase(samples.begin(), samples.begin() + samples.size() / 4);
		return;
	}
	coplanarRetries = 0;

	auto rotResult = calibration::calibrateRotation(samples);
	{
		char buf[256];
		snprintf(buf, sizeof buf, "Got %zd samples with %zd delta samples\n", rotResult.sampleCount, rotResult.deltaCount);
		CalCtx.Log(buf);
		snprintf(buf, sizeof buf, "Calibrated rotation: yaw=%.2f pitch=%.2f roll=%.2f\n",
			rotResult.eulerDegrees[1], rotResult.eulerDegrees[2], rotResult.eulerDegrees[0]);
		CalCtx.Log(buf);
	}
	ctx.calibratedRotation = rotResult.eulerDegrees;

	Eigen::Matrix3d calRot = calibration::eulerDegreesToRotationMatrix(ctx.calibratedRotation);

	ctx.calibratedTranslation = calibration::calibrateTranslation(samples, calRot);
	{
		char buf[256];
		snprintf(buf, sizeof buf, "Calibrated translation x=%.2f y=%.2f z=%.2f\n",
			ctx.calibratedTranslation[0], ctx.calibratedTranslation[1], ctx.calibratedTranslation[2]);
		CalCtx.Log(buf);
	}
	Eigen::Vector3d calTransM = ctx.calibratedTranslation * 0.01;

	const auto weights = calibration::computeSampleWeights(samples);
	Eigen::Vector3d hmdToTarget = calibration::computeRefToTargetOffset(samples, calRot, calTransM);
	double rmsError = calibration::retargetingErrorRMSWeighted(samples, hmdToTarget, calRot, calTransM, weights);

	char buf[256];
	snprintf(buf, sizeof buf, "Calibration residual error (RMS): %.1f mm\n", rmsError * 1000.0);
	CalCtx.Log(buf);

	if (rmsError > CalibrationContext::FullCalibrationRmsThresholdMeters)
	{
		const CalibrationContext::Speed failedSpeed = ctx.calibrationSpeed;
		if (TryAutoRetryFullCalibration(ctx, samples, rmsError, failedSpeed))
			return;

		FailFullCalibration(ctx, samples, rmsError);
		return;
	}

	const double trackerJitter = calibration::trackerTranslationJitterRMS(samples);
	if (trackerJitter > CalibrationContext::TundraJitterWarnThresholdMeters)
	{
		ctx.headFilterEnabled = true;
		ctx.headFilterParams = filter_defaults::TundraHead;
		ctx.driftFilterParams = filter_defaults::TundraDrift;
		snprintf(buf, sizeof buf,
			"Tracker translation jitter %.1f mm exceeds %.1f mm — applying stronger smoothing defaults.\n",
			trackerJitter * 1000.0,
			CalibrationContext::TundraJitterWarnThresholdMeters * 1000.0);
		CalCtx.Log(buf);
	}

	ApplyRelativeOffset(ctx, samples, calRot, calTransM);

	if (!hadValidProfileAtCalStart)
	{
		ctx.predictionAuto = true;
		ctx.enableAngularVelocity = true;
		CalCtx.Log("First calibration — enabled prediction auto-tune and angular velocity.\n");
	}

	ctx.validProfile = true;
	ctx.lastCalibrationTime = (double)std::time(nullptr);
	ctx.lastCalibrationRmsMm = rmsError * 1000.0;
	ctx.runtimeResidualHighStreak = 0;
	InvalidateAppliedDriverState();
	SaveProfile(ctx);
	CalCtx.Log("Finished calibration, profile saved\n");

	if (CalCtx.notificationId != 0) {
		vr::VRNotifications()->RemoveNotification(CalCtx.notificationId);
		CalCtx.notificationId = 0;
	}

	ctx.state = CalibrationState::None;
	samples.clear();
}

static void FinishPartialCalibration(CalibrationContext& ctx, std::vector<calibration::Sample>& samples)
{
	Eigen::Matrix3d calRot = calibration::eulerDegreesToRotationMatrix(ctx.calibratedRotation);
	Eigen::Vector3d calTransM = ctx.calibratedTranslation * 0.01;

	const auto weights = calibration::computeSampleWeights(samples);
	auto mount = calibration::computeRelativeOffsetWeighted(samples, calRot, calTransM);
	double rmsError = calibration::mountOffsetErrorRMSWeighted(samples, calRot, calTransM, mount, weights);
	const double rmsMm = rmsError * 1000.0;

	char buf[256];
	snprintf(buf, sizeof buf, "Mount offset residual error (RMS): %.1f mm\n", rmsMm);
	CalCtx.Log(buf);

	if (rmsError > CalibrationContext::PartialMountRmsThresholdMeters)
	{
		CalCtx.Log("Partial recalibration quality is too low. Your previous calibration is unchanged — press Calibrate for a full recalibration.\n");
		if (CalCtx.notificationId != 0) {
			vr::VRNotifications()->RemoveNotification(CalCtx.notificationId);
			CalCtx.notificationId = 0;
		}
		CalCtx.notificationId = ShowCalibrationNotification(
			"Mount offset re-estimation failed.\n"
			"Open Space Override and press Calibrate for a full recalibration.");
		ctx.state = CalibrationState::None;
		samples.clear();
		return;
	}

	ApplyRelativeOffset(ctx, samples, calRot, calTransM);
	ctx.lastPartialMountRmsMm = rmsMm;
	if (ctx.baselineMountRmsMm <= 0.0)
		ctx.baselineMountRmsMm = rmsMm;
	else
	{
		const double delta = rmsMm - ctx.baselineMountRmsMm;
		ctx.mountRigidityWarning = rmsMm > ctx.baselineMountRmsMm * CalibrationContext::MountRigidityWarnRatio
			&& delta >= CalibrationContext::MountRigidityWarnMinDeltaMm;
		if (ctx.mountRigidityWarning)
		{
			snprintf(buf, sizeof buf,
				"Mount rigidity warning: partial RMS %.1f mm vs baseline %.1f mm — check tracker mount.\n",
				rmsMm, ctx.baselineMountRmsMm);
			CalCtx.Log(buf);
		}
	}

	ctx.lastCalibrationTime = (double)std::time(nullptr);
	ctx.runtimeResidualHighStreak = 0;
	InvalidateAppliedDriverState();
	SaveProfile(ctx);
	CalCtx.Log("Mount offset updated, profile saved\n");

	if (CalCtx.notificationId != 0) {
		vr::VRNotifications()->RemoveNotification(CalCtx.notificationId);
		CalCtx.notificationId = 0;
	}

	ctx.state = CalibrationState::None;
	samples.clear();
}

void StartCalibration()
{
	hadValidProfileAtCalStart = CalCtx.validProfile;
	CalCtx.calibrationFailureOffer = false;
	CalCtx.lastCalibrationRmsMm = 0.0;
	CalCtx.state = CalibrationState::Begin;
	CalCtx.wantedUpdateInterval = 0.0;
	CalCtx.messages.clear();
	Detection.Clear();
	collectedSamples.clear();
	coplanarRetries = 0;
	calibrationAutoAttempts = 0;
	speedAtSessionStart = CalCtx.calibrationSpeed;
}

void RetryCalibrationAfterFailure()
{
	CalCtx.calibrationFailureOffer = false;
	CalCtx.lastCalibrationRmsMm = 0.0;
	StartCalibration();
}

void RestoreCalibrationAfterFailure()
{
	CalCtx.calibrationFailureOffer = false;
	CalCtx.lastCalibrationRmsMm = 0.0;
	AbortAndRestoreProfile(CalCtx);
	CalCtx.calibrationSpeed = speedAtSessionStart;

	if (CalCtx.notificationId != 0)
	{
		vr::VRNotifications()->RemoveNotification(CalCtx.notificationId);
		CalCtx.notificationId = 0;
	}
}

void StartPartialRecalibration()
{
	if (!CalCtx.validProfile)
		return;

	CalCtx.wantedUpdateInterval = 0.0;
	CalCtx.messages.clear();
	Detection.Clear();
	collectedSamples.clear();
	coplanarRetries = 0;
	SendHmdTrackerCommand(vr::k_unTrackedDeviceIndex_Hmd, vr::k_unTrackedDeviceIndexInvalid, false);
	CalCtx.state = CalibrationState::Recovering;
	CalCtx.Log("Tracker recovered — preparing mount offset re-estimation...\n");
}

static void AbortAndRestoreProfile(CalibrationContext &ctx)
{
	if (ctx.targetID != vr::k_unTrackedDeviceIndexInvalid)
		ResetAndDisableOffsets(ctx.targetID);

	InvalidateAppliedDriverState();
	LoadProfile(ctx);
	ctx.state = CalibrationState::None;
	collectedSamples.clear();
	coplanarRetries = 0;
}

void CalibrationTick(double time)
{
	if (!vr::VRSystem())
		return;

	auto &ctx = CalCtx;
	if ((time - ctx.timeLastTick) < 0.05)
		return;

	ctx.timeLastTick = time;
	vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseRawAndUncalibrated, 0.0f, ctx.devicePoses, vr::k_unMaxTrackedDeviceCount);

	if (ctx.timeLastDriverTelemetryFetch == 0.0)
	{
		FetchDriverTelemetry();
		ctx.timeLastDriverTelemetryFetch = time;
	}

	if (ctx.predictionAuto && ctx.validProfile && (time - ctx.timeLastTelemetryFetch) >= 0.5)
	{
		FetchPredictionTelemetry();
		ctx.timeLastTelemetryFetch = time;
	}

	if (ctx.validProfile && ctx.state == CalibrationState::None && (time - ctx.timeLastDriverTelemetryFetch) >= 0.5)
	{
		FetchDriverTelemetry();
		ctx.timeLastDriverTelemetryFetch = time;
	}

	if (ctx.state == CalibrationState::None && (time - ctx.timeLastGuardianBoundaryUpdate) >= 1.0)
	{
		UpdateGuardianBoundaryStatus(ctx);
		ctx.timeLastGuardianBoundaryUpdate = time;
	}

	if (ctx.validProfile && ctx.state == CalibrationState::None && EvaluateOverrideStatus(ctx).active
		&& (time - ctx.timeLastRuntimeQualityUpdate) >= 0.5)
	{
		UpdateRuntimeTrackingQuality(ctx);
		ctx.timeLastRuntimeQualityUpdate = time;

		if (ctx.autoPartialRecalOnMountDrift
			&& !ctx.guardianShiftSuspect
			&& ctx.runtimeResidualHighStreak >= CalibrationContext::RuntimeResidualHighStreakThreshold
			&& (time - ctx.timeLastAutoPartialRecal) >= CalibrationContext::AutoPartialRecalCooldownSec)
		{
			ctx.runtimeResidualHighStreak = 0;
			ctx.timeLastAutoPartialRecal = time;
			StartPartialRecalibration();
			ShowCalibrationNotification(
				"Mount drift detected — re-estimating tracker offset.\n"
				"Keep your head still for a few seconds.");
		}
	}

	if (ctx.state == CalibrationState::None)
	{
		ctx.wantedUpdateInterval = 1.0;

		constexpr double kIpcHealthIntervalSec = 30.0;
		if ((time - ctx.timeLastIpcHealthCheck) >= kIpcHealthIntervalSec)
		{
			if (!TryIpcHealthCheck())
			{
				std::cerr << "IPC health check failed — invalidating applied driver state" << std::endl;
				InvalidateAppliedDriverState();
				ctx.ipcHealthy = false;
			}
			else
			{
				ctx.ipcHealthy = true;
			}
			ctx.timeLastIpcHealthCheck = time;
		}

		if ((time - ctx.timeLastScan) >= 1.0)
		{
			ScanAndApplyProfile(ctx, time);
			ctx.timeLastScan = time;
		}
		return;
	}

	if (ctx.state == CalibrationState::Editing)
	{
		ctx.wantedUpdateInterval = 0.1;

		if ((time - ctx.timeLastScan) >= 0.1)
		{
			ScanAndApplyProfile(ctx, time);
			ctx.timeLastScan = time;
		}
		return;
	}

	if (ctx.state == CalibrationState::Begin)
	{
		SendHmdTrackerCommand(vr::k_unTrackedDeviceIndex_Hmd, vr::k_unTrackedDeviceIndexInvalid, false);

		if (vr::VRSystem()->GetTrackedDeviceClass(vr::k_unTrackedDeviceIndex_Hmd) != vr::TrackedDeviceClass_HMD ||
			!ctx.devicePoses[vr::k_unTrackedDeviceIndex_Hmd].bPoseIsValid)
		{
			ctx.state = CalibrationState::None;
			CalCtx.Log("No tracking HMD found, aborting calibration!\n");
			return;
		}

		std::string hmdSystem = GetDeviceTrackingSystem(vr::k_unTrackedDeviceIndex_Hmd);

		Detection.Clear();
		for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
		{
			if (vr::VRSystem()->GetTrackedDeviceClass(id) != vr::TrackedDeviceClass_GenericTracker)
				continue;
			if (!ctx.devicePoses[id].bPoseIsValid)
				continue;

			Detection.candidates.push_back(id);
		}

		if (Detection.candidates.empty())
		{
			ctx.state = CalibrationState::None;
			CalCtx.Log("No trackers from a different tracking system detected, aborting!\n");
			return;
		}

		if (!ctx.preferredTrackerSerial.empty())
		{
			for (uint32_t id : Detection.candidates)
			{
				if (GetDeviceSerial(id) == ctx.preferredTrackerSerial)
				{
					ctx.targetID = id;
					BeginSamplingPhase(ctx, id);
					return;
				}
			}

			CalCtx.Log("Preferred tracker not connected — falling back to motion detect...\n");
		}

		if (Detection.candidates.size() == 1)
		{
			ctx.targetID = Detection.candidates[0];
			BeginSamplingPhase(ctx, Detection.candidates[0]);
			return;
		}

		Detection.candidateAngSpeeds.resize(Detection.candidates.size());
		Detection.candidateLinSpeeds.resize(Detection.candidates.size());
		CalCtx.Log("Move your head around to identify the headset tracker...\n");
		ctx.state = CalibrationState::Detect;
		ctx.wantedUpdateInterval = 0.0;
		return;
	}

	if (ctx.state == CalibrationState::Detect)
	{
		if (!ctx.devicePoses[vr::k_unTrackedDeviceIndex_Hmd].bPoseIsValid)
			return;

		auto hmdPose = PoseFromMatrix(ctx.devicePoses[vr::k_unTrackedDeviceIndex_Hmd].mDeviceToAbsoluteTracking);
		Eigen::Matrix3d hmdRot = hmdPose.rot;
		Eigen::Vector3d hmdPos = hmdPose.trans;

		std::vector<Eigen::Matrix3d> curRot(Detection.candidates.size());
		std::vector<Eigen::Vector3d> curPos(Detection.candidates.size());
		for (size_t i = 0; i < Detection.candidates.size(); i++)
		{
			auto pose = PoseFromMatrix(ctx.devicePoses[Detection.candidates[i]].mDeviceToAbsoluteTracking);
			curRot[i] = pose.rot;
			curPos[i] = pose.trans;
		}

		double dt = time - Detection.prevTime;
		if (Detection.havePrev && dt > 1e-4)
		{
			Detection.hmdAngSpeeds.push_back(AngularSpeedBetween(hmdRot, Detection.prevRot[0], dt));
			Detection.hmdLinSpeeds.push_back(LinearSpeedBetween(hmdPos, Detection.prevPos[0], dt));
			for (size_t i = 0; i < Detection.candidates.size(); i++)
			{
				Detection.candidateAngSpeeds[i].push_back(AngularSpeedBetween(curRot[i], Detection.prevRot[i + 1], dt));
				Detection.candidateLinSpeeds[i].push_back(LinearSpeedBetween(curPos[i], Detection.prevPos[i + 1], dt));
			}

			CalCtx.Progress((int) Detection.hmdAngSpeeds.size(), 40);
		}

		Detection.prevRot.assign(1, hmdRot);
		Detection.prevRot.insert(Detection.prevRot.end(), curRot.begin(), curRot.end());
		Detection.prevPos.assign(1, hmdPos);
		Detection.prevPos.insert(Detection.prevPos.end(), curPos.begin(), curPos.end());
		Detection.prevTime = time;
		Detection.havePrev = true;

		if ((int) Detection.hmdAngSpeeds.size() < 40)
			return;

		double hmdPeak = 0;
		for (double s : Detection.hmdAngSpeeds)
			hmdPeak = max(hmdPeak, s);

		if (hmdPeak < 0.5)
		{
			Detection.Clear();
			ctx.state = CalibrationState::None;
			CalCtx.Log("Didn't detect enough head movement, aborting! Try again and move your head more.\n");
			return;
		}

		double bestCorr = -2, secondCorr = -2;
		int bestIdx = -1;
		for (size_t i = 0; i < Detection.candidates.size(); i++)
		{
			const double angCorr = PearsonCorrelation(Detection.hmdAngSpeeds, Detection.candidateAngSpeeds[i]);
			const double linCorr = PearsonCorrelation(Detection.hmdLinSpeeds, Detection.candidateLinSpeeds[i]);
			const double corr = 0.3 * angCorr + 0.7 * linCorr;
			if (corr > bestCorr)
			{
				secondCorr = bestCorr;
				bestCorr = corr;
				bestIdx = (int) i;
			}
			else if (corr > secondCorr)
			{
				secondCorr = corr;
			}
		}

		if (bestIdx == -1 || bestCorr < 0.7 || (bestCorr - secondCorr) < 0.1)
		{
			Detection.Clear();
			ctx.state = CalibrationState::None;
			CalCtx.Log("Couldn't clearly identify the headset tracker, aborting! Make sure only the headset tracker moves with your head, then try again.\n");
			return;
		}

		uint32_t targetID = Detection.candidates[bestIdx];
		Detection.Clear();
		BeginSamplingPhase(ctx, targetID);
		return;
	}

	if (ctx.state == CalibrationState::Recovering)
	{
		if (!ctx.validProfile)
		{
			ctx.state = CalibrationState::None;
			return;
		}

		if (!ctx.devicePoses[vr::k_unTrackedDeviceIndex_Hmd].bPoseIsValid)
			return;

		uint32_t targetID = FindTrackerBySerial(ctx);
		if (targetID == vr::k_unTrackedDeviceIndexInvalid || !ctx.devicePoses[targetID].bPoseIsValid)
			return;

		BeginPartialSamplingPhase(ctx, targetID);
		return;
	}

	if (ctx.state == CalibrationState::Sampling || ctx.state == CalibrationState::PartialSampling)
	{
		auto sample = CollectSample(ctx);
		if (!sample.valid)
			return;

		auto& samples = collectedSamples;
		samples.push_back(sample);
		UpdateLiveCalibrationQuality(ctx, samples);

		const size_t targetCount = ctx.state == CalibrationState::PartialSampling
			? CalibrationContext::PartialSampleCount
			: ctx.SampleCount();

		CalCtx.Progress((int)samples.size(), (int)targetCount);

		if (samples.size() >= targetCount)
		{
			CalCtx.Log("\n");
			if (ctx.state == CalibrationState::PartialSampling)
				FinishPartialCalibration(ctx, samples);
			else
				FinishFullCalibration(ctx, samples);
		}
	}
}

void LoadChaperoneBounds()
{
	vr::VRChaperoneSetup()->RevertWorkingCopy();

	uint32_t quadCount = 0;
	vr::VRChaperoneSetup()->GetLiveCollisionBoundsInfo(nullptr, &quadCount);

	CalCtx.chaperone.geometry.resize(quadCount);
	vr::VRChaperoneSetup()->GetLiveCollisionBoundsInfo(&CalCtx.chaperone.geometry[0], &quadCount);
	vr::VRChaperoneSetup()->GetWorkingStandingZeroPoseToRawTrackingPose(&CalCtx.chaperone.standingCenter);
	vr::VRChaperoneSetup()->GetWorkingPlayAreaSize(&CalCtx.chaperone.playSpaceSize.v[0], &CalCtx.chaperone.playSpaceSize.v[1]);
	CalCtx.chaperone.valid = true;
}

void ApplyChaperoneBounds()
{
	vr::VRChaperoneSetup()->RevertWorkingCopy();
	vr::VRChaperoneSetup()->SetWorkingCollisionBoundsInfo(&CalCtx.chaperone.geometry[0], CalCtx.chaperone.geometry.size());
	vr::VRChaperoneSetup()->SetWorkingStandingZeroPoseToRawTrackingPose(&CalCtx.chaperone.standingCenter);
	vr::VRChaperoneSetup()->SetWorkingPlayAreaSize(CalCtx.chaperone.playSpaceSize.v[0], CalCtx.chaperone.playSpaceSize.v[1]);
	vr::VRChaperoneSetup()->CommitWorkingCopy(vr::EChaperoneConfigFile_Live);
}
