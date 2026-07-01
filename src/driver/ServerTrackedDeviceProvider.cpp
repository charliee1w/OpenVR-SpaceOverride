// SPDX-License-Identifier: AGPL-3.0-only

#include "ServerTrackedDeviceProvider.h"
#include "Logging.h"
#include "InterfaceHookInjector.h"
#include "FilterDefaults.h"
#include "calibration/Drift.h"
#include "calibration/PoseBlend.h"
#include "calibration/PredictionLag.h"

#include "Version.h"

namespace
{
	constexpr double BlendToTrackerSec = 0.4;
	constexpr double BlendToSlamSec = 0.225;
	constexpr double MinScale = 0.01;
	constexpr double MaxScale = 100.0;
	constexpr float MaxPredictionFrames = 4.0f;
	constexpr float WirelessStreamBiasFrames = 0.5f;
	constexpr float WirelessPredictionFallbackFrames = 1.5f;

	bool IsValidDeviceIndex(uint32_t id)
	{
		return id < vr::k_unMaxTrackedDeviceCount;
	}

	bool IsFinite(double v)
	{
		return std::isfinite(v);
	}

	bool IsFiniteQuat(const vr::HmdQuaternion_t& q)
	{
		return IsFinite(q.w) && IsFinite(q.x) && IsFinite(q.y) && IsFinite(q.z);
	}

	bool IsFiniteVec3(const vr::HmdVector3d_t& v)
	{
		return IsFinite(v.v[0]) && IsFinite(v.v[1]) && IsFinite(v.v[2]);
	}

	float AngSpeedMagnitude(const double v[3])
	{
		return (float)std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
	}

	double QpcFrequency()
	{
		static double frequency = 0.0;
		if (frequency <= 0.0)
		{
			LARGE_INTEGER freq;
			QueryPerformanceFrequency(&freq);
			frequency = (double)freq.QuadPart;
		}
		return frequency;
	}

	double QpcNowSeconds()
	{
		LARGE_INTEGER now;
		QueryPerformanceCounter(&now);
		return now.QuadPart / QpcFrequency();
	}

	double ElapsedSeconds(LARGE_INTEGER& start, bool& primed)
	{
		LARGE_INTEGER now;
		QueryPerformanceCounter(&now);

		if (!primed)
		{
			start = now;
			primed = true;
			return 0.0;
		}

		return (now.QuadPart - start.QuadPart) / QpcFrequency();
	}
}

#include <algorithm>
#include <cmath>

inline vr::HmdQuaternion_t operator*(const vr::HmdQuaternion_t& lhs, const vr::HmdQuaternion_t& rhs) {
	return {
		(lhs.w * rhs.w) - (lhs.x * rhs.x) - (lhs.y * rhs.y) - (lhs.z * rhs.z),
		(lhs.w * rhs.x) + (lhs.x * rhs.w) + (lhs.y * rhs.z) - (lhs.z * rhs.y),
		(lhs.w * rhs.y) + (lhs.y * rhs.w) + (lhs.z * rhs.x) - (lhs.x * rhs.z),
		(lhs.w * rhs.z) + (lhs.z * rhs.w) + (lhs.x * rhs.y) - (lhs.y * rhs.x)
	};
}

inline vr::HmdVector3d_t quaternionRotateVector(const vr::HmdQuaternion_t& quat, const double(&vector)[3]) {
	vr::HmdQuaternion_t vectorQuat = { 0.0, vector[0], vector[1] , vector[2] };
	vr::HmdQuaternion_t conjugate = { quat.w, -quat.x, -quat.y, -quat.z };
	auto rotatedVectorQuat = quat * vectorQuat * conjugate;
	return { rotatedVectorQuat.x, rotatedVectorQuat.y, rotatedVectorQuat.z };
}

inline vr::HmdQuaternion_t quaternionNormalize(vr::HmdQuaternion_t q) {
	if (!IsFiniteQuat(q))
		return { 1, 0, 0, 0 };
	double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
	if (!std::isfinite(n) || n <= 1e-12)
		return { 1, 0, 0, 0 };
	q.w /= n; q.x /= n; q.y /= n; q.z /= n;
	return q;
}

inline vr::HmdQuaternion_t quaternionConjugate(const vr::HmdQuaternion_t& q) {
	return { q.w, -q.x, -q.y, -q.z };
}

static Eigen::Quaterniond VrQuatToEigen(const vr::HmdQuaternion_t& q)
{
	Eigen::Quaterniond out(q.w, q.x, q.y, q.z);
	return out.normalized();
}

static vr::HmdQuaternion_t EigenQuatToVr(const Eigen::Quaterniond& q)
{
	Eigen::Quaterniond n = q.normalized();
	return { n.w(), n.x(), n.y(), n.z() };
}

inline vr::HmdVector3d_t quaternionAngularVelocity(const vr::HmdQuaternion_t& cur, const vr::HmdQuaternion_t& prev, double dt) {
	if (dt <= 0.0)
		return { 0, 0, 0 };

	vr::HmdQuaternion_t d = quaternionNormalize(cur * quaternionConjugate(prev));
	if (d.w < 0.0) { d.w = -d.w; d.x = -d.x; d.y = -d.y; d.z = -d.z; }

	double s = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
	if (s < 1e-9)
		return { 0, 0, 0 };

	double scale = (2.0 * std::atan2(s, d.w)) / (s * dt);
	return { d.x * scale, d.y * scale, d.z * scale };
}

template < class T >
inline vr::HmdQuaternion_t HmdQuaternion_FromMatrix(const T& matrix)
{
	vr::HmdQuaternion_t q{};

	q.w = sqrt(fmax(0, 1 + matrix.m[0][0] + matrix.m[1][1] + matrix.m[2][2])) / 2;
	q.x = sqrt(fmax(0, 1 + matrix.m[0][0] - matrix.m[1][1] - matrix.m[2][2])) / 2;
	q.y = sqrt(fmax(0, 1 - matrix.m[0][0] + matrix.m[1][1] - matrix.m[2][2])) / 2;
	q.z = sqrt(fmax(0, 1 - matrix.m[0][0] - matrix.m[1][1] + matrix.m[2][2])) / 2;

	q.x = copysign(q.x, matrix.m[2][1] - matrix.m[1][2]);
	q.y = copysign(q.y, matrix.m[0][2] - matrix.m[2][0]);
	q.z = copysign(q.z, matrix.m[1][0] - matrix.m[0][1]);

	return q;
}

static double FilterStep(LARGE_INTEGER& lastUpdate, bool primed)
{
	LARGE_INTEGER now;
	QueryPerformanceCounter(&now);

	double dt = primed ? (now.QuadPart - lastUpdate.QuadPart) / QpcFrequency() : 0.0;
	lastUpdate = now;
	if (dt <= 0.0 || isnan(dt)) dt = 1.0 / 90.0;
	if (dt > 0.1) dt = 0.1;
	return dt;
}

vr::EVRInitError ServerTrackedDeviceProvider::Init(vr::IVRDriverContext* pDriverContext)
{
	TRACE("ServerTrackedDeviceProvider::Init()");
	VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);

	if (!LogFile)
		OpenLogFile();
	LOG("OpenVR-SpaceOverride " SPACECAL_VERSION_STRING " loaded");

	memset(transforms, 0, vr::k_unMaxTrackedDeviceCount * sizeof DeviceTransform);
	memset(slamSync, 0, sizeof slamSync);
	poseWorkActive.store(false, std::memory_order_release);

	auto applyDefaults = [](oneeuro::Params& out, const protocol::OneEuroParams& defaults) {
		out.minCutoff = defaults.minCutoff;
		out.beta = defaults.beta;
		out.dCutoff = defaults.dCutoff;
	};

	applyDefaults(drift.rotationFilter.params, filter_defaults::Off);
	applyDefaults(drift.translationFilter.params, filter_defaults::Off);
	applyDefaults(headFilter.rotationFilter.params, filter_defaults::Off);
	applyDefaults(headFilter.translationFilter.params, filter_defaults::Off);
	headVel.filter.params = { 8.0, 1.0, 1.0 };

	if (!InjectHooks(pDriverContext))
		LOG("Pose hooks failed to install — driver will run in IPC-only mode (override inactive)");

	server.Run();

	return vr::VRInitError_None;
}

void ServerTrackedDeviceProvider::Cleanup()
{
	LOG("OpenVR-SpaceOverride unloading");
	SetDriverShuttingDown(true);
	server.Stop();
	DisableHooks();
	VR_CLEANUP_SERVER_DRIVER_CONTEXT();
	LOG("OpenVR-SpaceOverride unloaded");
	CloseLogFile();
}

void ServerTrackedDeviceProvider::UpdateDisplayHz()
{
	vr::PropertyContainerHandle_t container = vr::VRProperties()->TrackedDeviceToPropertyContainer(vr::k_unTrackedDeviceIndex_Hmd);
	const float displayHz = vr::VRProperties()->GetFloatProperty(container, vr::Prop_DisplayFrequency_Float);
	predictionTune.displayHz = displayHz > 1.0f ? displayHz : 90.0f;
}

void ServerTrackedDeviceProvider::RunFrame()
{
	std::lock_guard<std::mutex> lock(stateMutex);
	UpdateDisplayHz();
	if (hmdTracker.enabled && hmdTracker.predictionAuto)
		UpdatePredictionAutoTune();
}

bool ServerTrackedDeviceProvider::AnySlamSyncEnabled() const
{
	for (uint32_t i = 0; i < vr::k_unMaxTrackedDeviceCount; ++i)
	{
		if (slamSync[i])
			return true;
	}
	return false;
}

void ServerTrackedDeviceProvider::InvalidateDriftIfSlamSyncInactive()
{
	if (AnySlamSyncEnabled())
		return;

	drift.valid = false;
	drift.rotationFilter.reset();
	drift.translationFilter.reset();
}

void ServerTrackedDeviceProvider::RecomputePoseWorkActive()
{
	bool active = hmdTracker.enabled && IsValidDeviceIndex(hmdTracker.trackerID);
	if (!active && drift.valid)
	{
		for (uint32_t i = 0; i < vr::k_unMaxTrackedDeviceCount; ++i)
		{
			if (slamSync[i])
			{
				active = true;
				break;
			}
		}
	}
	if (!active)
	{
		for (uint32_t i = 0; i < vr::k_unMaxTrackedDeviceCount; ++i)
		{
			if (transforms[i].enabled)
			{
				active = true;
				break;
			}
		}
	}
	poseWorkActive.store(active, std::memory_order_release);
}

bool ServerTrackedDeviceProvider::SetDeviceTransform(const protocol::SetDeviceTransform& newTransform)
{
	if (!IsValidDeviceIndex(newTransform.openVRID))
	{
		LOG("SetDeviceTransform rejected invalid openVRID %u", newTransform.openVRID);
		return false;
	}

	if (newTransform.updateTranslation && !IsFiniteVec3(newTransform.translation))
	{
		LOG("SetDeviceTransform rejected non-finite translation for device %u", newTransform.openVRID);
		return false;
	}

	if (newTransform.updateRotation && !IsFiniteQuat(newTransform.rotation))
	{
		LOG("SetDeviceTransform rejected non-finite rotation for device %u", newTransform.openVRID);
		return false;
	}

	if (newTransform.updateScale && !IsFinite(newTransform.scale))
	{
		LOG("SetDeviceTransform rejected non-finite scale for device %u", newTransform.openVRID);
		return false;
	}

	std::lock_guard<std::mutex> lock(stateMutex);
	auto& tf = transforms[newTransform.openVRID];
	tf.enabled = newTransform.enabled;

	if (newTransform.updateTranslation)
		tf.translation = newTransform.translation;

	if (newTransform.updateRotation)
		tf.rotation = quaternionNormalize(newTransform.rotation);

	if (newTransform.updateScale)
		tf.scale = std::clamp(newTransform.scale, MinScale, MaxScale);

	RecomputePoseWorkActive();
	return true;
}

bool ServerTrackedDeviceProvider::SetHmdTracker(const protocol::SetHmdTracker& cmd)
{
	if (!IsValidDeviceIndex(cmd.hmdID) || cmd.hmdID != vr::k_unTrackedDeviceIndex_Hmd)
	{
		LOG("SetHmdTracker rejected invalid hmdID %u", cmd.hmdID);
		return false;
	}

	if (cmd.enabled && !IsValidDeviceIndex(cmd.trackerID))
	{
		LOG("SetHmdTracker rejected invalid trackerID %u", cmd.trackerID);
		return false;
	}

	std::lock_guard<std::mutex> lock(stateMutex);

	hmdTracker.enabled = cmd.enabled;
	hmdTracker.native = cmd.native;
	hmdTracker.slamFallback = cmd.slamFallback;
	hmdTracker.enableAngularVelocity = cmd.enableAngularVelocity;
	hmdTracker.manualPredictionTime = std::clamp(
		std::isfinite(cmd.predictionTime) ? cmd.predictionTime : 1.0f,
		0.0f,
		MaxPredictionFrames);
	hmdTracker.predictionAuto = cmd.predictionAuto;
	hmdTracker.predictionTime = cmd.predictionAuto ? EffectivePredictionFrames() : hmdTracker.manualPredictionTime;
	hmdTracker.hmdID = cmd.hmdID;
	hmdTracker.trackerID = cmd.enabled ? cmd.trackerID : vr::k_unTrackedDeviceIndexInvalid;
	hmdTracker.offsetRotation = quaternionNormalize(cmd.offsetRotation);
	hmdTracker.offsetTranslation = IsFiniteVec3(cmd.offsetTranslation)
		? cmd.offsetTranslation
		: vr::HmdVector3d_t{ 0, 0, 0 };
	hmdTracker.calibrationRotation = quaternionNormalize(cmd.calibrationRotation);
	hmdTracker.calibrationTranslation = IsFiniteVec3(cmd.calibrationTranslation)
		? cmd.calibrationTranslation
		: vr::HmdVector3d_t{ 0, 0, 0 };

	if (!cmd.enabled)
	{
		drift.valid = false;
		drift.rotationFilter.reset();
		drift.translationFilter.reset();
		headFilter.reset();
		headVel.reset();
		trackerBlend.reset();
		predictionTune.reset();
		trackerHealth = {};
		// Keep slamSync[] — calibration temporarily disables the HMD tracker without
		// intending to drop continuous sync on body trackers until profile re-applies.
	}
	else
	{
		trackerBlend.reset();
		if (!cmd.predictionAuto)
			predictionTune.reset();
	}

	RecomputePoseWorkActive();
	return true;
}

bool ServerTrackedDeviceProvider::SetSlamSync(const protocol::SetSlamSync& cmd)
{
	if (!IsValidDeviceIndex(cmd.openVRID))
	{
		LOG("SetSlamSync rejected invalid openVRID %u", cmd.openVRID);
		return false;
	}

	std::lock_guard<std::mutex> lock(stateMutex);
	slamSync[cmd.openVRID] = cmd.enabled;
	InvalidateDriftIfSlamSyncInactive();
	RecomputePoseWorkActive();
	return true;
}

bool ServerTrackedDeviceProvider::SetOneEuro(const protocol::SetOneEuro& cmd)
{
	std::lock_guard<std::mutex> lock(stateMutex);
	auto toParams = [](const protocol::OneEuroParams& p) {
		oneeuro::Params out;
		out.minCutoff = p.minCutoff < 0.01 ? 0.01 : p.minCutoff;
		out.beta = p.beta < 0.0 ? 0.0 : p.beta;
		out.dCutoff = p.dCutoff < 0.01 ? 0.01 : p.dCutoff;
		return out;
	};

	const protocol::OneEuroParams& headParams = cmd.headEnabled ? cmd.head : filter_defaults::Off;

	headFilter.rotationFilter.params = toParams(headParams);
	headFilter.translationFilter.params = toParams(headParams);
	drift.rotationFilter.params = toParams(cmd.drift);
	drift.translationFilter.params = toParams(cmd.drift);

	if (headFilter.enabled && !cmd.headEnabled)
		headFilter.reset();
	headFilter.enabled = cmd.headEnabled;
	return true;
}

float ServerTrackedDeviceProvider::EffectivePredictionFrames() const
{
	if (hmdTracker.predictionAuto)
	{
		const float frames = predictionTune.estimateValid
			? predictionTune.appliedPredictionFrames
			: (WirelessPredictionFallbackFrames + WirelessStreamBiasFrames);
		return std::clamp(frames, 0.0f, MaxPredictionFrames);
	}
	return std::clamp(hmdTracker.manualPredictionTime, 0.0f, MaxPredictionFrames);
}

void ServerTrackedDeviceProvider::RecordPredictionSample(
	double timeSec,
	float trackerAngSpeed,
	float slamAngSpeed,
	float trackerLinSpeed,
	float slamLinSpeed)
{
	auto& tune = predictionTune;
	tune.trackerSamples.push_back(trackerAngSpeed);
	tune.slamSamples.push_back(slamAngSpeed);
	tune.trackerLinSamples.push_back(trackerLinSpeed);
	tune.slamLinSamples.push_back(slamLinSpeed);
	tune.sampleTimes.push_back(timeSec);

	while (tune.sampleTimes.size() > PredictionAutoTune::MaxSamples)
	{
		tune.trackerSamples.pop_front();
		tune.slamSamples.pop_front();
		tune.trackerLinSamples.pop_front();
		tune.slamLinSamples.pop_front();
		tune.sampleTimes.pop_front();
	}

	while (!tune.sampleTimes.empty() && (timeSec - tune.sampleTimes.front()) > PredictionAutoTune::WindowSec)
	{
		tune.trackerSamples.pop_front();
		tune.slamSamples.pop_front();
		tune.trackerLinSamples.pop_front();
		tune.slamLinSamples.pop_front();
		tune.sampleTimes.pop_front();
	}
}

void ServerTrackedDeviceProvider::UpdatePredictionAutoTune()
{
	if (!hmdTracker.predictionAuto)
		return;

	auto& tune = predictionTune;
	calibration::AngVelSeries series;
	series.tracker.assign(tune.trackerSamples.begin(), tune.trackerSamples.end());
	series.slam.assign(tune.slamSamples.begin(), tune.slamSamples.end());
	series.trackerLin.assign(tune.trackerLinSamples.begin(), tune.trackerLinSamples.end());
	series.slamLin.assign(tune.slamLinSamples.begin(), tune.slamLinSamples.end());

	auto estimate = calibration::estimatePredictionLagFrames(series, 45, 4);
	if (!estimate.valid)
		return;

	tune.estimateValid = true;
	tune.estimatedLagFrames = (float)estimate.lagFrames;
	const float hz = tune.displayHz > 1.0f ? tune.displayHz : 1.0f;
	tune.estimatedLagMs = tune.estimatedLagFrames * 1000.0f / hz;

	const float target = std::clamp(tune.estimatedLagFrames + WirelessStreamBiasFrames, 0.0f, MaxPredictionFrames);
	tune.appliedPredictionFrames += (target - tune.appliedPredictionFrames) * 0.25f;
	hmdTracker.predictionTime = tune.appliedPredictionFrames;
}

protocol::PredictionTelemetry ServerTrackedDeviceProvider::GetPredictionTelemetry() const
{
	std::lock_guard<std::mutex> lock(stateMutex);
	protocol::PredictionTelemetry telemetry{};
	telemetry.valid = predictionTune.estimateValid;
	telemetry.estimatedLagMs = predictionTune.estimatedLagMs;
	telemetry.estimatedLagFrames = predictionTune.estimatedLagFrames;
	telemetry.appliedPredictionFrames = EffectivePredictionFrames();
	telemetry.displayHz = predictionTune.displayHz;
	return telemetry;
}

namespace
{
	float QuaternionYawDegrees(const vr::HmdQuaternion_t& q)
	{
		const double yaw = std::atan2(
			2.0 * (q.w * q.z + q.x * q.y),
			1.0 - 2.0 * (q.y * q.y + q.z * q.z));
		return (float)(yaw * 180.0 / 3.14159265358979323846);
	}
}

protocol::DriverTelemetry ServerTrackedDeviceProvider::GetDriverTelemetry() const
{
	std::lock_guard<std::mutex> lock(stateMutex);
	protocol::DriverTelemetry telemetry{};
	telemetry.valid = true;
	telemetry.poseHooksInstalled = PoseHooksInstalled();
	telemetry.overrideEnabled = hmdTracker.enabled;
	telemetry.trackerValid = trackerHealth.poseValid;
	telemetry.trackerBlendActive = trackerBlend.active;
	telemetry.appliedPredictionFrames = EffectivePredictionFrames();
	telemetry.displayHz = predictionTune.displayHz;
	telemetry.driftValid = drift.valid;
	telemetry.headFilterEnabled = headFilter.enabled;
	telemetry.slamSyncActive = AnySlamSyncEnabled();

	if (!telemetry.poseHooksInstalled)
		telemetry.overrideInactiveReason = protocol::DriverOverrideHooksMissing;
	else if (!hmdTracker.enabled)
		telemetry.overrideInactiveReason = protocol::DriverOverrideDisabled;
	else if (!IsValidDeviceIndex(hmdTracker.trackerID))
		telemetry.overrideInactiveReason = protocol::DriverOverrideTrackerInvalid;
	else if (!trackerHealth.poseValid)
		telemetry.overrideInactiveReason = protocol::DriverOverrideTrackerLost;
	else
		telemetry.overrideInactiveReason = protocol::DriverOverrideActive;

	telemetry.overrideActive = telemetry.overrideInactiveReason == protocol::DriverOverrideActive
		|| trackerBlend.active;

	if (drift.valid)
	{
		telemetry.driftYawDeg = QuaternionYawDegrees(drift.rotation);
		const double tx = drift.translation.v[0];
		const double ty = drift.translation.v[1];
		const double tz = drift.translation.v[2];
		telemetry.driftTranslationMm = (float)(std::sqrt(tx * tx + ty * ty + tz * tz) * 1000.0);
	}

	if (hmdTracker.enabled && !trackerHealth.poseValid && trackerHealth.lastValidQpc > 0.0)
	{
		const double now = QpcNowSeconds();
		const double lostSec = now - trackerHealth.lastValidQpc;
		telemetry.trackerLostSeconds = (float)(lostSec > 0.0 ? lostSec : 0.0);
	}

	return telemetry;
}

void ServerTrackedDeviceProvider::UpdateDrift(const vr::HmdQuaternion_t& correctedRotation, const double(&correctedPosition)[3],
	const vr::HmdQuaternion_t& rawRotation, const double(&rawPosition)[3])
{
	auto correctedQuat = VrQuatToEigen(quaternionNormalize(correctedRotation));
	auto rawQuat = VrQuatToEigen(quaternionNormalize(rawRotation));
	Eigen::Vector3d correctedPos(correctedPosition[0], correctedPosition[1], correctedPosition[2]);
	Eigen::Vector3d rawPos(rawPosition[0], rawPosition[1], rawPosition[2]);

	auto sample = calibration::computeInstantaneousDrift(correctedQuat, correctedPos, rawQuat, rawPos);
	if (!calibration::isDriftSampleWithinLimits(sample, {}))
		return;

	sample.rotation = calibration::clampRotationStep(sample.rotation, 5.0);

	double dt = FilterStep(drift.lastUpdate, drift.valid);

	drift.rotation = drift.rotationFilter.filter(EigenQuatToVr(sample.rotation), dt);
	drift.translation = drift.translationFilter.filter(
		{ sample.translation.x(), sample.translation.y(), sample.translation.z() }, dt);
	drift.valid = true;
}

void ServerTrackedDeviceProvider::BeginTrackerBlend(
	TrackerBlendState::Mode mode,
	const HmdPoseSample& fromPose,
	const HmdPoseSample& toPose,
	double durationSec)
{
	trackerBlend.active = true;
	trackerBlend.mode = mode;
	trackerBlend.fromPose = fromPose;
	trackerBlend.toPose = toPose;
	trackerBlend.durationSec = durationSec;
	trackerBlend.timePrimed = false;
	trackerBlend.startTime = {};
}

ServerTrackedDeviceProvider::HmdPoseSample ServerTrackedDeviceProvider::ResolveBlendedHmdPose(
	bool trackerValid,
	const HmdPoseSample& trackerPose,
	const HmdPoseSample& slamPose,
	bool slamPoseValid)
{
	if (trackerValid != trackerBlend.trackerWasValid)
	{
		if (trackerValid && trackerPose.valid)
		{
			HmdPoseSample from = trackerBlend.hasEmittedPose ? trackerBlend.emittedPose : slamPose;
			if (!from.valid)
				from = trackerPose;
			BeginTrackerBlend(TrackerBlendState::Mode::ToTracker, from, trackerPose, BlendToTrackerSec);
		}
		else
		{
			HmdPoseSample from = trackerBlend.hasEmittedPose ? trackerBlend.emittedPose : trackerPose;
			HmdPoseSample to = {};
			if (hmdTracker.slamFallback && slamPoseValid)
				to = slamPose;
			BeginTrackerBlend(TrackerBlendState::Mode::ToSlam, from, to, BlendToSlamSec);
		}
	}

	trackerBlend.trackerWasValid = trackerValid;

	HmdPoseSample result;

	if (trackerBlend.active)
	{
		if (trackerBlend.mode == TrackerBlendState::Mode::ToTracker && trackerValid && trackerPose.valid)
			trackerBlend.toPose = trackerPose;
		else if (trackerBlend.mode == TrackerBlendState::Mode::ToSlam && slamPoseValid)
			trackerBlend.toPose = slamPose;

		double elapsed = ElapsedSeconds(trackerBlend.startTime, trackerBlend.timePrimed);
		double t = trackerBlend.durationSec > 0.0 ? elapsed / trackerBlend.durationSec : 1.0;

		if (t >= 1.0)
		{
			trackerBlend.active = false;
			result = trackerBlend.toPose.valid ? trackerBlend.toPose : trackerBlend.fromPose;
			result.valid = trackerBlend.toPose.valid;
		}
		else if (trackerBlend.fromPose.valid && trackerBlend.toPose.valid)
		{
			Eigen::Quaterniond fromQ = VrQuatToEigen(trackerBlend.fromPose.rotation);
			Eigen::Quaterniond toQ = VrQuatToEigen(trackerBlend.toPose.rotation);
			Eigen::Vector3d fromP(trackerBlend.fromPose.position[0], trackerBlend.fromPose.position[1], trackerBlend.fromPose.position[2]);
			Eigen::Vector3d toP(trackerBlend.toPose.position[0], trackerBlend.toPose.position[1], trackerBlend.toPose.position[2]);

			auto blendedQ = calibration::slerpQuaternion(fromQ, toQ, t);
			auto blendedP = calibration::lerpVector3(fromP, toP, t);

			result.valid = true;
			result.rotation = EigenQuatToVr(blendedQ);
			result.position[0] = blendedP.x();
			result.position[1] = blendedP.y();
			result.position[2] = blendedP.z();
		}
		else
		{
			result = trackerBlend.fromPose;
		}
	}
	else if (trackerValid && trackerPose.valid)
	{
		result = trackerPose;
	}
	else if (slamPoseValid && hmdTracker.slamFallback)
	{
		result = slamPose;
	}
	else
	{
		result.valid = false;
	}

	if (result.valid)
	{
		trackerBlend.emittedPose = result;
		trackerBlend.hasEmittedPose = true;
	}

	return result;
}

ServerTrackedDeviceProvider::HmdPoseSample ServerTrackedDeviceProvider::BuildTrackerHmdPose(
	const vr::TrackedDevicePose_t& tp) const
{
	HmdPoseSample sample;
	if (!tp.bPoseIsValid)
		return sample;

	vr::HmdQuaternion_t trackerQuat = HmdQuaternion_FromMatrix(tp.mDeviceToAbsoluteTracking);

	double trackerPos[3] = {
		tp.mDeviceToAbsoluteTracking.m[0][3],
		tp.mDeviceToAbsoluteTracking.m[1][3],
		tp.mDeviceToAbsoluteTracking.m[2][3]
	};

	const bool trackerPlayspaceTransformActive = IsValidDeviceIndex(hmdTracker.trackerID)
		&& transforms[hmdTracker.trackerID].enabled;

	vr::HmdQuaternion_t trackerRefRotation;
	vr::HmdVector3d_t trackerRefPosition;
	if (trackerPlayspaceTransformActive)
	{
		trackerRefRotation = trackerQuat;
		trackerRefPosition.v[0] = trackerPos[0];
		trackerRefPosition.v[1] = trackerPos[1];
		trackerRefPosition.v[2] = trackerPos[2];
	}
	else
	{
		trackerRefRotation = quaternionNormalize(hmdTracker.calibrationRotation * trackerQuat);
		trackerRefPosition = quaternionRotateVector(hmdTracker.calibrationRotation, trackerPos);
		trackerRefPosition.v[0] += hmdTracker.calibrationTranslation.v[0];
		trackerRefPosition.v[1] += hmdTracker.calibrationTranslation.v[1];
		trackerRefPosition.v[2] += hmdTracker.calibrationTranslation.v[2];
	}

	vr::HmdQuaternion_t hmdRotation = quaternionNormalize(
		hmdTracker.native ? trackerQuat * hmdTracker.offsetRotation : trackerRefRotation * hmdTracker.offsetRotation);
	vr::HmdVector3d_t offset = quaternionRotateVector(
		hmdTracker.native ? trackerQuat : trackerRefRotation, hmdTracker.offsetTranslation.v);

	sample.rotation = hmdRotation;
	if (hmdTracker.native)
	{
		sample.position[0] = trackerPos[0] + offset.v[0];
		sample.position[1] = trackerPos[1] + offset.v[1];
		sample.position[2] = trackerPos[2] + offset.v[2];
	}
	else
	{
		sample.position[0] = trackerRefPosition.v[0] + offset.v[0];
		sample.position[1] = trackerRefPosition.v[1] + offset.v[1];
		sample.position[2] = trackerRefPosition.v[2] + offset.v[2];
	}

	sample.valid = IsFiniteQuat(sample.rotation)
		&& IsFinite(sample.position[0])
		&& IsFinite(sample.position[1])
		&& IsFinite(sample.position[2]);
	return sample;
}

ServerTrackedDeviceProvider::HmdPoseSample ServerTrackedDeviceProvider::BuildSlamHmdPose(
	const vr::DriverPose_t& pose,
	bool valid) const
{
	HmdPoseSample sample;
	if (!valid)
		return sample;

	vr::DriverPose_t slamPose = pose;
	if (drift.valid)
		ApplyDrift(slamPose);

	sample.rotation = quaternionNormalize(
		slamPose.qWorldFromDriverRotation * slamPose.qRotation * slamPose.qDriverFromHeadRotation);

	vr::HmdVector3d_t headLocal = quaternionRotateVector(slamPose.qRotation, slamPose.vecDriverFromHeadTranslation);
	double driverLocal[3] = {
		slamPose.vecPosition[0] + headLocal.v[0],
		slamPose.vecPosition[1] + headLocal.v[1],
		slamPose.vecPosition[2] + headLocal.v[2]
	};
	vr::HmdVector3d_t world = quaternionRotateVector(slamPose.qWorldFromDriverRotation, driverLocal);
	sample.position[0] = world.v[0] + slamPose.vecWorldFromDriverTranslation[0];
	sample.position[1] = world.v[1] + slamPose.vecWorldFromDriverTranslation[1];
	sample.position[2] = world.v[2] + slamPose.vecWorldFromDriverTranslation[2];
	sample.valid = IsFiniteQuat(sample.rotation)
		&& IsFinite(sample.position[0])
		&& IsFinite(sample.position[1])
		&& IsFinite(sample.position[2]);
	return sample;
}

bool ServerTrackedDeviceProvider::IsFiniteHmdPoseSample(const HmdPoseSample& sample) const
{
	return sample.valid
		&& IsFiniteQuat(sample.rotation)
		&& IsFinite(sample.position[0])
		&& IsFinite(sample.position[1])
		&& IsFinite(sample.position[2]);
}

bool ServerTrackedDeviceProvider::FetchTrackerPose(uint32_t trackerID, vr::TrackedDevicePose_t& outPose, bool& outValid)
{
	outPose = {};
	outValid = false;

	if (!IsValidDeviceIndex(trackerID))
		return false;

	const uint32_t poseCount = (std::min)(trackerID + 1, vr::k_unMaxTrackedDeviceCount);
	vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount]{};
	vr::VRServerDriverHost()->GetRawTrackedDevicePoses(0.0f, poses, poseCount);

	outPose = poses[trackerID];
	outValid = outPose.bPoseIsValid;
	return true;
}

void ServerTrackedDeviceProvider::WriteHmdPoseToDriver(
	vr::DriverPose_t& pose,
	const HmdPoseSample& hmdPose,
	const vr::TrackedDevicePose_t* trackerPose)
{
	if (!hmdPose.valid)
	{
		headVel.reset();
		if (!hmdTracker.slamFallback)
		{
			if (hmdTracker.native)
			{
				pose.qWorldFromDriverRotation = { 1, 0, 0, 0 };
				pose.vecWorldFromDriverTranslation[0] = 0;
				pose.vecWorldFromDriverTranslation[1] = 0;
				pose.vecWorldFromDriverTranslation[2] = 0;
			}
			else
			{
				pose.qWorldFromDriverRotation = hmdTracker.calibrationRotation;
				pose.vecWorldFromDriverTranslation[0] = hmdTracker.calibrationTranslation.v[0];
				pose.vecWorldFromDriverTranslation[1] = hmdTracker.calibrationTranslation.v[1];
				pose.vecWorldFromDriverTranslation[2] = hmdTracker.calibrationTranslation.v[2];
			}

			pose.qDriverFromHeadRotation = { 1, 0, 0, 0 };
			pose.vecDriverFromHeadTranslation[0] = 0;
			pose.vecDriverFromHeadTranslation[1] = 0;
			pose.vecDriverFromHeadTranslation[2] = 0;

			pose.qRotation = { 1, 0, 0, 0 };
			pose.vecPosition[0] = 0;
			pose.vecPosition[1] = 0;
			pose.vecPosition[2] = 0;

			for (int i = 0; i < 3; i++)
			{
				pose.vecVelocity[i] = 0;
				pose.vecAngularVelocity[i] = 0;
			}

			pose.poseIsValid = false;
			pose.deviceIsConnected = true;
			pose.result = vr::TrackingResult_Running_OutOfRange;
			pose.shouldApplyHeadModel = false;
			pose.poseTimeOffset = 0;
		}
		return;
	}

	pose.qWorldFromDriverRotation = { 1, 0, 0, 0 };
	pose.vecWorldFromDriverTranslation[0] = 0;
	pose.vecWorldFromDriverTranslation[1] = 0;
	pose.vecWorldFromDriverTranslation[2] = 0;

	pose.qDriverFromHeadRotation = { 1, 0, 0, 0 };
	pose.vecDriverFromHeadTranslation[0] = 0;
	pose.vecDriverFromHeadTranslation[1] = 0;
	pose.vecDriverFromHeadTranslation[2] = 0;

	pose.qRotation = hmdPose.rotation;
	pose.vecPosition[0] = hmdPose.position[0];
	pose.vecPosition[1] = hmdPose.position[1];
	pose.vecPosition[2] = hmdPose.position[2];

	if (headFilter.enabled && trackerPose && trackerPose->bPoseIsValid)
	{
		double dt = FilterStep(headFilter.lastUpdate, headFilter.valid);
		headFilter.valid = true;

		pose.qRotation = headFilter.rotationFilter.filter(pose.qRotation, dt);

		vr::HmdVector3d_t headPos = headFilter.translationFilter.filter(
			{ pose.vecPosition[0], pose.vecPosition[1], pose.vecPosition[2] }, dt);
		pose.vecPosition[0] = headPos.v[0];
		pose.vecPosition[1] = headPos.v[1];
		pose.vecPosition[2] = headPos.v[2];
	}

	if (trackerPose && trackerPose->bPoseIsValid)
	{
		double trackerVel[3] = {
			trackerPose->vVelocity.v[0],
			trackerPose->vVelocity.v[1],
			trackerPose->vVelocity.v[2]
		};

		vr::HmdQuaternion_t trackerQuat = HmdQuaternion_FromMatrix(trackerPose->mDeviceToAbsoluteTracking);
		const bool trackerPlayspaceTransformActive = hmdTracker.enabled
			&& IsValidDeviceIndex(hmdTracker.trackerID)
			&& transforms[hmdTracker.trackerID].enabled;
		vr::HmdQuaternion_t refRot = trackerPlayspaceTransformActive
			? trackerQuat
			: quaternionNormalize(hmdTracker.calibrationRotation * trackerQuat);
		vr::HmdVector3d_t offset = quaternionRotateVector(
			hmdTracker.native ? trackerQuat : refRot,
			hmdTracker.offsetTranslation.v);

		vr::HmdVector3d_t vel = quaternionRotateVector(hmdTracker.calibrationRotation, trackerVel);
		const bool useRawTrackerLinearVel = hmdTracker.native || trackerPlayspaceTransformActive;

		double dtAng = FilterStep(headVel.lastUpdate, headVel.valid);
		vr::HmdVector3d_t headAngVel = { 0, 0, 0 };
		if (headVel.valid)
		{
			headAngVel = headVel.filter.filter(
				quaternionAngularVelocity(pose.qRotation, headVel.prevRotation, dtAng), dtAng);
		}
		else if (hmdTracker.enableAngularVelocity)
		{
			double trackerAngVel[3] = {
				trackerPose->vAngularVelocity.v[0],
				trackerPose->vAngularVelocity.v[1],
				trackerPose->vAngularVelocity.v[2]
			};
			headAngVel = useRawTrackerLinearVel
				? vr::HmdVector3d_t{ trackerAngVel[0], trackerAngVel[1], trackerAngVel[2] }
				: quaternionRotateVector(hmdTracker.calibrationRotation, trackerAngVel);
		}
		headVel.prevRotation = pose.qRotation;
		headVel.valid = true;

		vr::HmdVector3d_t tangential = {
			headAngVel.v[1] * offset.v[2] - headAngVel.v[2] * offset.v[1],
			headAngVel.v[2] * offset.v[0] - headAngVel.v[0] * offset.v[2],
			headAngVel.v[0] * offset.v[1] - headAngVel.v[1] * offset.v[0]
		};

		for (int i = 0; i < 3; i++)
		{
			double baseVel = useRawTrackerLinearVel ? trackerVel[i] : vel.v[i];
			pose.vecVelocity[i] = baseVel + tangential.v[i];
			pose.vecAngularVelocity[i] = hmdTracker.enableAngularVelocity ? headAngVel.v[i] : 0.0;
		}
	}
	else
	{
		headVel.reset();
	}

	pose.poseIsValid = true;
	pose.deviceIsConnected = true;
	pose.result = vr::TrackingResult_Running_OK;
	pose.shouldApplyHeadModel = false;
	pose.poseTimeOffset = 0;
}

void ServerTrackedDeviceProvider::ApplyDrift(vr::DriverPose_t& pose) const
{
	pose.qWorldFromDriverRotation = quaternionNormalize(drift.rotation * pose.qWorldFromDriverRotation);

	vr::HmdVector3d_t rotatedTranslation = quaternionRotateVector(drift.rotation, pose.vecWorldFromDriverTranslation);
	pose.vecWorldFromDriverTranslation[0] = rotatedTranslation.v[0] + drift.translation.v[0];
	pose.vecWorldFromDriverTranslation[1] = rotatedTranslation.v[1] + drift.translation.v[1];
	pose.vecWorldFromDriverTranslation[2] = rotatedTranslation.v[2] + drift.translation.v[2];
}

bool ServerTrackedDeviceProvider::HandleDevicePoseUpdated(uint32_t openVRID, vr::DriverPose_t& pose)
{
	if (IsDriverShuttingDown())
		return true;

	if (!poseWorkActive.load(std::memory_order_acquire))
		return true;

	if (!IsValidDeviceIndex(openVRID))
		return true;

	bool processHmdOverride = false;
	uint32_t trackerID = vr::k_unTrackedDeviceIndexInvalid;
	bool rawValid = false;
	vr::HmdQuaternion_t rawRotation = { 1, 0, 0, 0 };
	double rawPosition[3] = { 0, 0, 0 };

	{
		std::lock_guard<std::mutex> lock(stateMutex);

		auto& tf = transforms[openVRID];
		if (tf.enabled && !hmdTracker.native)
		{
			pose.qWorldFromDriverRotation = tf.rotation * pose.qWorldFromDriverRotation;

			pose.vecPosition[0] *= tf.scale;
			pose.vecPosition[1] *= tf.scale;
			pose.vecPosition[2] *= tf.scale;

			vr::HmdVector3d_t rotatedTranslation = quaternionRotateVector(tf.rotation, pose.vecWorldFromDriverTranslation);
			pose.vecWorldFromDriverTranslation[0] = rotatedTranslation.v[0] + tf.translation.v[0];
			pose.vecWorldFromDriverTranslation[1] = rotatedTranslation.v[1] + tf.translation.v[1];
			pose.vecWorldFromDriverTranslation[2] = rotatedTranslation.v[2] + tf.translation.v[2];
		}

		if (!hmdTracker.enabled || !IsValidDeviceIndex(hmdTracker.trackerID))
		{
			if (slamSync[openVRID] && drift.valid)
				ApplyDrift(pose);
			return true;
		}

		if (openVRID != hmdTracker.hmdID)
		{
			if (slamSync[openVRID] && drift.valid)
				ApplyDrift(pose);
			return true;
		}

		processHmdOverride = true;
		trackerID = hmdTracker.trackerID;

		rawValid = pose.poseIsValid && pose.deviceIsConnected && pose.result == vr::TrackingResult_Running_OK;
		if (rawValid)
		{
			rawRotation = quaternionNormalize(pose.qWorldFromDriverRotation * pose.qRotation * pose.qDriverFromHeadRotation);

			vr::HmdVector3d_t headLocal = quaternionRotateVector(pose.qRotation, pose.vecDriverFromHeadTranslation);
			double driverLocal[3] = {
				pose.vecPosition[0] + headLocal.v[0],
				pose.vecPosition[1] + headLocal.v[1],
				pose.vecPosition[2] + headLocal.v[2]
			};
			vr::HmdVector3d_t world = quaternionRotateVector(pose.qWorldFromDriverRotation, driverLocal);
			rawPosition[0] = world.v[0] + pose.vecWorldFromDriverTranslation[0];
			rawPosition[1] = world.v[1] + pose.vecWorldFromDriverTranslation[1];
			rawPosition[2] = world.v[2] + pose.vecWorldFromDriverTranslation[2];
		}
	}

	if (!processHmdOverride)
		return true;

	vr::TrackedDevicePose_t trackerPoseRaw{};
	bool trackerValid = false;
	if (!FetchTrackerPose(trackerID, trackerPoseRaw, trackerValid))
		return true;

	std::lock_guard<std::mutex> lock(stateMutex);

	const HmdPoseSample trackerPose = BuildTrackerHmdPose(trackerPoseRaw);
	const HmdPoseSample slamPose = BuildSlamHmdPose(pose, rawValid);

	HmdPoseSample blendedPose = ResolveBlendedHmdPose(
		trackerValid,
		trackerPose,
		slamPose,
		rawValid);

	if (!IsFiniteHmdPoseSample(blendedPose))
	{
		LOG("Dropping non-finite blended HMD pose");
		blendedPose.valid = false;
	}

	const bool trackerHealthy = trackerValid && trackerPose.valid;
	if (trackerHealthy)
	{
		trackerHealth.poseValid = true;
		trackerHealth.lastValidQpc = QpcNowSeconds();
	}
	else if (hmdTracker.enabled)
	{
		trackerHealth.poseValid = false;
	}

	if (hmdTracker.predictionAuto && rawValid && trackerValid && trackerPose.valid && !trackerBlend.active)
	{
		double slamAngVel[3] = {
			pose.vecAngularVelocity[0],
			pose.vecAngularVelocity[1],
			pose.vecAngularVelocity[2]
		};
		double slamLinVel[3] = {
			pose.vecVelocity[0],
			pose.vecVelocity[1],
			pose.vecVelocity[2]
		};

		double trackerAngVel[3] = {
			trackerPoseRaw.vAngularVelocity.v[0],
			trackerPoseRaw.vAngularVelocity.v[1],
			trackerPoseRaw.vAngularVelocity.v[2]
		};
		double trackerLinVel[3] = {
			trackerPoseRaw.vVelocity.v[0],
			trackerPoseRaw.vVelocity.v[1],
			trackerPoseRaw.vVelocity.v[2]
		};

		vr::HmdVector3d_t calibratedAngVel = quaternionRotateVector(hmdTracker.calibrationRotation, trackerAngVel);
		vr::HmdVector3d_t calibratedLinVel = quaternionRotateVector(hmdTracker.calibrationRotation, trackerLinVel);
		const double* trackerAngVelSample = hmdTracker.native ? trackerAngVel : calibratedAngVel.v;
		const double* trackerLinVelSample = hmdTracker.native ? trackerLinVel : calibratedLinVel.v;

		const float slamAngSpeed = AngSpeedMagnitude(slamAngVel);
		const float trackerAngSpeed = AngSpeedMagnitude(trackerAngVelSample);
		const float slamLinSpeed = AngSpeedMagnitude(slamLinVel);
		const float trackerLinSpeed = AngSpeedMagnitude(trackerLinVelSample);

		const bool angReady = slamAngSpeed >= PredictionAutoTune::MinAngSpeed
			&& trackerAngSpeed >= PredictionAutoTune::MinAngSpeed;
		const bool linReady = slamLinSpeed >= PredictionAutoTune::MinLinSpeed
			&& trackerLinSpeed >= PredictionAutoTune::MinLinSpeed;

		if (angReady || linReady)
		{
			RecordPredictionSample(
				QpcNowSeconds(),
				trackerAngSpeed,
				slamAngSpeed,
				trackerLinSpeed,
				slamLinSpeed);
		}
	}

	if (AnySlamSyncEnabled() && trackerValid && trackerPose.valid && rawValid && !trackerBlend.active)
	{
		double trackerVel[3] = {
			trackerPoseRaw.vVelocity.v[0],
			trackerPoseRaw.vVelocity.v[1],
			trackerPoseRaw.vVelocity.v[2]
		};

		double trackerAngVel[3] = {
			trackerPoseRaw.vAngularVelocity.v[0],
			trackerPoseRaw.vAngularVelocity.v[1],
			trackerPoseRaw.vAngularVelocity.v[2]
		};

		const double linSpeed = sqrt(
			trackerVel[0] * trackerVel[0] +
			trackerVel[1] * trackerVel[1] +
			trackerVel[2] * trackerVel[2]);

		const double angSpeed = sqrt(
			trackerAngVel[0] * trackerAngVel[0] +
			trackerAngVel[1] * trackerAngVel[1] +
			trackerAngVel[2] * trackerAngVel[2]);

		const double maxLinSpeed = 2.75;
		const double maxAngSpeed = 3.5;

		if (!drift.valid || (linSpeed < maxLinSpeed && angSpeed < maxAngSpeed))
		{
			UpdateDrift(
				trackerPose.rotation,
				trackerPose.position,
				rawRotation,
				rawPosition);
		}
	}

	WriteHmdPoseToDriver(
		pose,
		blendedPose,
		trackerValid ? &trackerPoseRaw : nullptr);

	return true;
}