// SPDX-License-Identifier: AGPL-3.0-only

#include "ServerTrackedDeviceProvider.h"
#include "Logging.h"
#include "InterfaceHookInjector.h"

#include "Version.h"

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
	double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
	if (n > 0.0) {
		q.w /= n; q.x /= n; q.y /= n; q.z /= n;
	}
	return q;
}

inline vr::HmdQuaternion_t quaternionConjugate(const vr::HmdQuaternion_t& q) {
	return { q.w, -q.x, -q.y, -q.z };
}

inline vr::HmdQuaternion_t quaternionProjectYaw(const vr::HmdQuaternion_t& q) {
	double n = std::sqrt(q.w * q.w + q.y * q.y);
	if (n < 1e-9)
		return { 1, 0, 0, 0 };
	return { q.w / n, 0.0, q.y / n, 0.0 };
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
	LARGE_INTEGER now, freq;
	QueryPerformanceCounter(&now);
	QueryPerformanceFrequency(&freq);

	double dt = primed ? (now.QuadPart - lastUpdate.QuadPart) / (double)freq.QuadPart : 0.0;
	lastUpdate = now;
	if (dt <= 0.0 || isnan(dt)) dt = 1.0 / 90.0;
	if (dt > 0.1) dt = 0.1;
	return dt;
}

vr::EVRInitError ServerTrackedDeviceProvider::Init(vr::IVRDriverContext* pDriverContext)
{
	TRACE("ServerTrackedDeviceProvider::Init()");
	VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);

	OpenLogFile();
	LOG("OpenVR-SpaceOverride " SPACECAL_VERSION_STRING " loaded");
	LOG("auto-log session: %s", GetSessionLogPath()[0] ? GetSessionLogPath() : "(cwd fallback)");
	LOG("auto-log: events=tracker_ok/bad, speed_reject, HMD_JUMP/HOLD, last_good_hold, 60s heartbeat");
	LOG("gates: isfinite, pre-publish jump, last-good<=150ms, tracker-hook-cache");

	memset(transforms, 0, vr::k_unMaxTrackedDeviceCount * sizeof(DeviceTransform));
	memset(slamSync, 0, sizeof slamSync);
	diag = SessionDiag{};
	lastGoodHmd = LastGoodHmd{};
	cachedTracker = CachedTrackerPose{};
	displayHzQueried = false;
	cachedDisplayHz = 90.0;

	drift.rotationFilter.params = { 3.0, 1.3, 0.6 };
	drift.translationFilter.params = { 3.0, 1.3, 0.6 };
	headFilter.rotationFilter.params = { 5.0, 0.8, 1.0 };
	headFilter.translationFilter.params = { 5.0, 0.8, 1.0 };
	headVel.filter.params = { 8.0, 1.0, 1.0 };

	trackerFilter.translation.SetQ(2.5e-7);
  	trackerFilter.translation.SetR(1.0e-5);
	trackerFilter.translation.SetAdaptiveGain(4.0);

	InjectHooks(pDriverContext);
	server.Run();

	return vr::VRInitError_None;
}

void ServerTrackedDeviceProvider::Cleanup()
{
	LOG("session stats: frames=%llu ok=%llu bad=%llu jumps=%llu jump_holds=%llu last_good_holds=%llu speed_rej=%llu fallback=%llu nonfinite=%llu play_primed=%d",
		(unsigned long long)diag.frames,
		(unsigned long long)diag.trackerOkFrames,
		(unsigned long long)diag.trackerBadFrames,
		(unsigned long long)diag.jumpEvents,
		(unsigned long long)diag.jumpHolds,
		(unsigned long long)diag.lastGoodHolds,
		(unsigned long long)diag.speedRejects,
		(unsigned long long)diag.fallbackFrames,
		(unsigned long long)diag.nonFiniteDrops,
		diag.playPrimed ? 1 : 0);
	LOG("OpenVR-SpaceOverride unloading");
	SetDriverShuttingDown(true);
	server.Stop();
	DisableHooks();
	VR_CLEANUP_SERVER_DRIVER_CONTEXT();
	LOG("OpenVR-SpaceOverride unloaded");
	CloseLogFile();
}

void ServerTrackedDeviceProvider::SetDeviceTransform(const protocol::SetDeviceTransform& newTransform)
{
	if (newTransform.openVRID >= vr::k_unMaxTrackedDeviceCount)
		return;

	auto& tf = transforms[newTransform.openVRID];
	tf.enabled = newTransform.enabled;

	if (newTransform.updateTranslation)
		tf.translation = newTransform.translation;

	if (newTransform.updateRotation)
		tf.rotation = newTransform.rotation;

	if (newTransform.updateScale)
		tf.scale = newTransform.scale;
}

void ServerTrackedDeviceProvider::SetHmdTracker(const protocol::SetHmdTracker& cmd)
{
	if (cmd.hmdID >= vr::k_unMaxTrackedDeviceCount)
		return;

	// Disable uses k_unTrackedDeviceIndexInvalid; only validate trackerID when enabling.
	if (cmd.enabled && cmd.trackerID >= vr::k_unMaxTrackedDeviceCount)
		return;

	const bool trackerChanged = hmdTracker.trackerID != cmd.trackerID;

	const double newScale = cmd.calibrationScale > 0.0 ? cmd.calibrationScale : 1.0;
	const double newHmdScale = cmd.hmdScale > 0.0 ? cmd.hmdScale : 1.0;

	const bool changed =
		hmdTracker.enabled != cmd.enabled
		|| hmdTracker.native != cmd.native
		|| hmdTracker.slamFallback != cmd.slamFallback
		|| hmdTracker.enableAngularVelocity != cmd.enableAngularVelocity
		|| hmdTracker.predictionTime != cmd.predictionTime
		|| hmdTracker.hmdID != cmd.hmdID
		|| hmdTracker.trackerID != cmd.trackerID
		|| hmdTracker.calibrationScale != newScale
		|| hmdTracker.hmdScale != newHmdScale
		|| hmdTracker.offsetRotation.w != cmd.offsetRotation.w
		|| hmdTracker.offsetRotation.x != cmd.offsetRotation.x
		|| hmdTracker.offsetRotation.y != cmd.offsetRotation.y
		|| hmdTracker.offsetRotation.z != cmd.offsetRotation.z
		|| hmdTracker.offsetTranslation.v[0] != cmd.offsetTranslation.v[0]
		|| hmdTracker.offsetTranslation.v[1] != cmd.offsetTranslation.v[1]
		|| hmdTracker.offsetTranslation.v[2] != cmd.offsetTranslation.v[2]
		|| hmdTracker.calibrationRotation.w != cmd.calibrationRotation.w
		|| hmdTracker.calibrationRotation.x != cmd.calibrationRotation.x
		|| hmdTracker.calibrationRotation.y != cmd.calibrationRotation.y
		|| hmdTracker.calibrationRotation.z != cmd.calibrationRotation.z
		|| hmdTracker.calibrationTranslation.v[0] != cmd.calibrationTranslation.v[0]
		|| hmdTracker.calibrationTranslation.v[1] != cmd.calibrationTranslation.v[1]
		|| hmdTracker.calibrationTranslation.v[2] != cmd.calibrationTranslation.v[2];

	hmdTracker.enabled = cmd.enabled;
	hmdTracker.native = cmd.native;
	hmdTracker.slamFallback = cmd.slamFallback;
	hmdTracker.enableAngularVelocity = cmd.enableAngularVelocity;
	hmdTracker.predictionTime = cmd.predictionTime;
	hmdTracker.hmdID = cmd.hmdID;
	hmdTracker.trackerID = cmd.trackerID;
	hmdTracker.offsetRotation = cmd.offsetRotation;
	hmdTracker.offsetTranslation = cmd.offsetTranslation;
	hmdTracker.calibrationRotation = cmd.calibrationRotation;
	hmdTracker.calibrationTranslation = cmd.calibrationTranslation;
	hmdTracker.calibrationScale = newScale;
	hmdTracker.hmdScale = newHmdScale;

	// Log only real changes (overlay scan used to spam this every second).
	if (changed)
	{
		LOG("SetHmdTracker enabled=%d native=%d slamFallback=%d pred=%.2f hmd=%u tracker=%u scale=%.5f hmdScale=%.5f eAngVel=%d",
			cmd.enabled ? 1 : 0,
			cmd.native ? 1 : 0,
			cmd.slamFallback ? 1 : 0,
			cmd.predictionTime,
			cmd.hmdID,
			cmd.trackerID,
			hmdTracker.calibrationScale,
			hmdTracker.hmdScale,
			cmd.enableAngularVelocity ? 1 : 0);
	}

	if (!cmd.enabled)
	{
		drift.valid = false;
		drift.rotationFilter.reset();
		drift.translationFilter.reset();
		headFilter.reset();
		headVel.reset();
		trackerFilter.reset();
		memset(slamSync, 0, sizeof slamSync);
		diag.haveLastHmdPos = false;
		diag.trackerStateKnown = false;
		diag.playPrimed = false;
		lastGoodHmd.valid = false;
		cachedTracker.valid = false;
	}
	else if (trackerChanged)
	{
		cachedTracker.valid = false;
		lastGoodHmd.valid = false;
		diag.haveLastHmdPos = false;
	}
}

void ServerTrackedDeviceProvider::SetSlamSync(const protocol::SetSlamSync& cmd)
{
	if (cmd.openVRID < vr::k_unMaxTrackedDeviceCount)
		slamSync[cmd.openVRID] = cmd.enabled;
}

void ServerTrackedDeviceProvider::SetOneEuro(const protocol::SetOneEuro& cmd)
{
	auto toParams = [](const protocol::OneEuroParams& p) {
		oneeuro::Params out;
		out.minCutoff = p.minCutoff < 0.01 ? 0.01 : p.minCutoff;
		out.beta = p.beta < 0.0 ? 0.0 : p.beta;
		out.dCutoff = p.dCutoff < 0.01 ? 0.01 : p.dCutoff;
		return out;
	};

	headFilter.rotationFilter.params = toParams(cmd.head);
	headFilter.translationFilter.params = toParams(cmd.head);
	drift.rotationFilter.params = toParams(cmd.drift);
	drift.translationFilter.params = toParams(cmd.drift);

	if (headFilter.enabled && !cmd.headEnabled)
		headFilter.reset();
	headFilter.enabled = cmd.headEnabled;
}

static double SoftGateWeight(double speed, double softMax, double hardMax)
{
	if (hardMax <= softMax)
		return speed <= softMax ? 1.0 : 0.0;
	if (speed <= softMax)
		return 1.0;
	if (speed >= hardMax)
		return 0.0;
	double t = (speed - softMax) / (hardMax - softMax);
	// Smoothstep so the gate eases out instead of clipping.
	t = t * t * (3.0 - 2.0 * t);
	return 1.0 - t;
}

static vr::HmdQuaternion_t NlerpQuat(const vr::HmdQuaternion_t& a, const vr::HmdQuaternion_t& b, double t)
{
	double dot = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
	vr::HmdQuaternion_t bb = b;
	if (dot < 0.0)
	{
		bb.w = -bb.w; bb.x = -bb.x; bb.y = -bb.y; bb.z = -bb.z;
		dot = -dot;
	}
	double u = 1.0 - t;
	vr::HmdQuaternion_t r = {
		u * a.w + t * bb.w,
		u * a.x + t * bb.x,
		u * a.y + t * bb.y,
		u * a.z + t * bb.z
	};
	return quaternionNormalize(r);
}

void ServerTrackedDeviceProvider::UpdateDrift(const vr::HmdQuaternion_t& correctedRotation, const double(&correctedPosition)[3],
	const vr::HmdQuaternion_t& rawRotation, const double(&rawPosition)[3], double weight)
{
	if (weight <= 0.0)
		return;
	if (weight > 1.0)
		weight = 1.0;

	vr::HmdQuaternion_t instRot = quaternionProjectYaw(quaternionNormalize(correctedRotation * quaternionConjugate(rawRotation)));
	vr::HmdVector3d_t instRotatedRaw = quaternionRotateVector(instRot, rawPosition);

	double slamScale = SlamToCorrectedScale();
	vr::HmdVector3d_t instTrans = {
		correctedPosition[0] - instRotatedRaw.v[0] * slamScale,
		correctedPosition[1] - instRotatedRaw.v[1] * slamScale,
		correctedPosition[2] - instRotatedRaw.v[2] * slamScale
	};

	// Soft gate: blend new measurement toward the current drift estimate when moving fast.
	if (weight < 1.0 && drift.valid)
	{
		instRot = NlerpQuat(drift.rotation, instRot, weight);
		instTrans.v[0] = drift.translation.v[0] + (instTrans.v[0] - drift.translation.v[0]) * weight;
		instTrans.v[1] = drift.translation.v[1] + (instTrans.v[1] - drift.translation.v[1]) * weight;
		instTrans.v[2] = drift.translation.v[2] + (instTrans.v[2] - drift.translation.v[2]) * weight;
	}

	double dt = FilterStep(drift.lastUpdate, drift.valid);

	drift.rotation = drift.rotationFilter.filter(instRot, dt);
	drift.translation = drift.translationFilter.filter(instTrans, dt);
	drift.valid = true;
}

void ServerTrackedDeviceProvider::ApplyDrift(vr::DriverPose_t& pose) const
{
	double slamScale = SlamToCorrectedScale();

	pose.qWorldFromDriverRotation = quaternionNormalize(drift.rotation * pose.qWorldFromDriverRotation);

	pose.vecPosition[0] *= slamScale;
	pose.vecPosition[1] *= slamScale;
	pose.vecPosition[2] *= slamScale;

	double scaledTranslation[3] = {
		pose.vecWorldFromDriverTranslation[0] * slamScale,
		pose.vecWorldFromDriverTranslation[1] * slamScale,
		pose.vecWorldFromDriverTranslation[2] * slamScale
	};
	vr::HmdVector3d_t rotatedTranslation = quaternionRotateVector(drift.rotation, scaledTranslation);
	pose.vecWorldFromDriverTranslation[0] = rotatedTranslation.v[0] + drift.translation.v[0];
	pose.vecWorldFromDriverTranslation[1] = rotatedTranslation.v[1] + drift.translation.v[1];
	pose.vecWorldFromDriverTranslation[2] = rotatedTranslation.v[2] + drift.translation.v[2];
}

bool ServerTrackedDeviceProvider::IsFiniteVec3(const double v[3])
{
	return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

bool ServerTrackedDeviceProvider::IsFiniteQuat(const vr::HmdQuaternion_t& q)
{
	return std::isfinite(q.w) && std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z);
}

double ServerTrackedDeviceProvider::GetCachedDisplayHz(uint32_t hmdOpenVRID)
{
	LARGE_INTEGER now{}, freq{};
	QueryPerformanceCounter(&now);
	QueryPerformanceFrequency(&freq);

	bool need = !displayHzQueried;
	if (displayHzQueried)
	{
		double elapsed = (now.QuadPart - displayHzLastQuery.QuadPart) / (double)freq.QuadPart;
		if (elapsed >= 1.0)
			need = true;
	}

	if (need && hmdOpenVRID < vr::k_unMaxTrackedDeviceCount)
	{
		vr::PropertyContainerHandle_t container = vr::VRProperties()->TrackedDeviceToPropertyContainer(hmdOpenVRID);
		double hz = vr::VRProperties()->GetFloatProperty(container, vr::Prop_DisplayFrequency_Float);
		if (hz >= 1.0)
			cachedDisplayHz = hz;
		else if (!displayHzQueried)
			cachedDisplayHz = 90.0;
		displayHzLastQuery = now;
		displayHzQueried = true;
	}

	return cachedDisplayHz > 1.0 ? cachedDisplayHz : 90.0;
}

void ServerTrackedDeviceProvider::CacheTrackerWorldPose(const vr::DriverPose_t& pose)
{
	if (!pose.poseIsValid || !pose.deviceIsConnected || pose.result != vr::TrackingResult_Running_OK)
		return;

	vr::HmdQuaternion_t worldRot = quaternionNormalize(
		pose.qWorldFromDriverRotation * pose.qRotation * pose.qDriverFromHeadRotation);
	if (!IsFiniteQuat(worldRot))
		return;

	vr::HmdVector3d_t headLocal = quaternionRotateVector(pose.qRotation, pose.vecDriverFromHeadTranslation);
	double driverLocal[3] = {
		pose.vecPosition[0] + headLocal.v[0],
		pose.vecPosition[1] + headLocal.v[1],
		pose.vecPosition[2] + headLocal.v[2]
	};
	vr::HmdVector3d_t world = quaternionRotateVector(pose.qWorldFromDriverRotation, driverLocal);
	double pos[3] = {
		world.v[0] + pose.vecWorldFromDriverTranslation[0],
		world.v[1] + pose.vecWorldFromDriverTranslation[1],
		world.v[2] + pose.vecWorldFromDriverTranslation[2]
	};
	if (!IsFiniteVec3(pos))
		return;

	vr::HmdVector3d_t worldVel = quaternionRotateVector(pose.qWorldFromDriverRotation, pose.vecVelocity);
	vr::HmdVector3d_t worldAng = quaternionRotateVector(
		quaternionNormalize(pose.qWorldFromDriverRotation * pose.qRotation),
		pose.vecAngularVelocity);

	cachedTracker.rotation = worldRot;
	cachedTracker.position[0] = pos[0];
	cachedTracker.position[1] = pos[1];
	cachedTracker.position[2] = pos[2];
	cachedTracker.velocity[0] = worldVel.v[0];
	cachedTracker.velocity[1] = worldVel.v[1];
	cachedTracker.velocity[2] = worldVel.v[2];
	cachedTracker.angularVelocity[0] = worldAng.v[0];
	cachedTracker.angularVelocity[1] = worldAng.v[1];
	cachedTracker.angularVelocity[2] = worldAng.v[2];
	QueryPerformanceCounter(&cachedTracker.timestamp);
	cachedTracker.valid = true;
}

bool ServerTrackedDeviceProvider::FetchTrackerSample(
	vr::HmdQuaternion_t& outRot, double outPos[3],
	double outVel[3], double outAngVel[3], double& outLinSpeed)
{
	LARGE_INTEGER now{}, freq{};
	QueryPerformanceCounter(&now);
	QueryPerformanceFrequency(&freq);

	// Prefer hook cache if fresh (<=100 ms).
	if (cachedTracker.valid)
	{
		double age = (now.QuadPart - cachedTracker.timestamp.QuadPart) / (double)freq.QuadPart;
		if (age >= 0.0 && age <= 0.10)
		{
			outRot = cachedTracker.rotation;
			for (int i = 0; i < 3; i++)
			{
				outPos[i] = cachedTracker.position[i];
				outVel[i] = cachedTracker.velocity[i];
				outAngVel[i] = cachedTracker.angularVelocity[i];
			}
			outLinSpeed = sqrt(outVel[0] * outVel[0] + outVel[1] * outVel[1] + outVel[2] * outVel[2]);
			const double maxAcceptLinSpeed = 8.0;
			if (outLinSpeed <= maxAcceptLinSpeed && IsFiniteQuat(outRot) && IsFiniteVec3(outPos))
				return true;
		}
	}

	// Fallback: unpredicted GetRaw (compositor uses our velocities for reprojection).
	vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount];
	vr::VRServerDriverHost()->GetRawTrackedDevicePoses(0.0f, poses, vr::k_unMaxTrackedDeviceCount);

	if (hmdTracker.trackerID >= vr::k_unMaxTrackedDeviceCount)
		return false;

	const auto& tp = poses[hmdTracker.trackerID];
	outVel[0] = tp.vVelocity.v[0];
	outVel[1] = tp.vVelocity.v[1];
	outVel[2] = tp.vVelocity.v[2];
	outAngVel[0] = tp.vAngularVelocity.v[0];
	outAngVel[1] = tp.vAngularVelocity.v[1];
	outAngVel[2] = tp.vAngularVelocity.v[2];
	outLinSpeed = sqrt(outVel[0] * outVel[0] + outVel[1] * outVel[1] + outVel[2] * outVel[2]);

	const double maxAcceptLinSpeed = 8.0;
	if (!tp.bPoseIsValid || !tp.bDeviceIsConnected
		|| tp.eTrackingResult != vr::TrackingResult_Running_OK
		|| outLinSpeed > maxAcceptLinSpeed)
	{
		return false;
	}

	outRot = HmdQuaternion_FromMatrix(tp.mDeviceToAbsoluteTracking);
	outPos[0] = tp.mDeviceToAbsoluteTracking.m[0][3];
	outPos[1] = tp.mDeviceToAbsoluteTracking.m[1][3];
	outPos[2] = tp.mDeviceToAbsoluteTracking.m[2][3];
	return IsFiniteQuat(outRot) && IsFiniteVec3(outPos);
}

void ServerTrackedDeviceProvider::StoreLastGoodHmd(const vr::DriverPose_t& pose)
{
	lastGoodHmd.rotation = pose.qRotation;
	lastGoodHmd.position[0] = pose.vecPosition[0];
	lastGoodHmd.position[1] = pose.vecPosition[1];
	lastGoodHmd.position[2] = pose.vecPosition[2];
	lastGoodHmd.velocity[0] = pose.vecVelocity[0];
	lastGoodHmd.velocity[1] = pose.vecVelocity[1];
	lastGoodHmd.velocity[2] = pose.vecVelocity[2];
	lastGoodHmd.angularVelocity[0] = pose.vecAngularVelocity[0];
	lastGoodHmd.angularVelocity[1] = pose.vecAngularVelocity[1];
	lastGoodHmd.angularVelocity[2] = pose.vecAngularVelocity[2];
	QueryPerformanceCounter(&lastGoodHmd.timestamp);
	lastGoodHmd.valid = true;
}

bool ServerTrackedDeviceProvider::ApplyLastGoodHmd(vr::DriverPose_t& pose, double maxAgeSec) const
{
	if (!lastGoodHmd.valid)
		return false;

	LARGE_INTEGER now{}, freq{};
	QueryPerformanceCounter(&now);
	QueryPerformanceFrequency(&freq);
	double age = (now.QuadPart - lastGoodHmd.timestamp.QuadPart) / (double)freq.QuadPart;
	if (age < 0.0 || age > maxAgeSec)
		return false;

	pose.qWorldFromDriverRotation = { 1, 0, 0, 0 };
	pose.vecWorldFromDriverTranslation[0] = 0;
	pose.vecWorldFromDriverTranslation[1] = 0;
	pose.vecWorldFromDriverTranslation[2] = 0;
	pose.qDriverFromHeadRotation = { 1, 0, 0, 0 };
	pose.vecDriverFromHeadTranslation[0] = 0;
	pose.vecDriverFromHeadTranslation[1] = 0;
	pose.vecDriverFromHeadTranslation[2] = 0;
	pose.qRotation = lastGoodHmd.rotation;
	pose.vecPosition[0] = lastGoodHmd.position[0];
	pose.vecPosition[1] = lastGoodHmd.position[1];
	pose.vecPosition[2] = lastGoodHmd.position[2];
	for (int i = 0; i < 3; i++)
	{
		pose.vecVelocity[i] = lastGoodHmd.velocity[i];
		pose.vecAngularVelocity[i] = lastGoodHmd.angularVelocity[i];
	}
	pose.poseIsValid = true;
	pose.deviceIsConnected = true;
	pose.result = vr::TrackingResult_Running_OK;
	pose.shouldApplyHeadModel = false;
	pose.poseTimeOffset = 0;
	return true;
}

bool ServerTrackedDeviceProvider::ShouldHoldForJump(const double newPos[3], double dtSec) const
{
	if (!lastGoodHmd.valid)
		return false; // nothing to compare — accept first solid frame
	if (!IsFiniteVec3(newPos))
		return true;

	const double dx = newPos[0] - lastGoodHmd.position[0];
	const double dy = newPos[1] - lastGoodHmd.position[1];
	const double dz = newPos[2] - lastGoodHmd.position[2];
	const double dist = sqrt(dx * dx + dy * dy + dz * dz);

	// Absolute snap (single frame teleport) or impossible speed.
	const double jumpThreshM = 0.35;
	const double maxSpeedMps = 8.0;
	if (dist > jumpThreshM)
		return true;
	if (dtSec > 1e-4 && (dist / dtSec) > maxSpeedMps)
		return true;
	return false;
}

bool ServerTrackedDeviceProvider::HandleDevicePoseUpdated(uint32_t openVRID, vr::DriverPose_t& pose)
{
	if (openVRID >= vr::k_unMaxTrackedDeviceCount)
		return true;

	// P1-b: capture head tracker world pose from its own hook (before any transforms).
	if (hmdTracker.enabled
		&& hmdTracker.trackerID < vr::k_unMaxTrackedDeviceCount
		&& openVRID == hmdTracker.trackerID)
	{
		CacheTrackerWorldPose(pose);
	}

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

	if (hmdTracker.enabled)
	{
		if (openVRID == hmdTracker.hmdID)
		{
			bool rawValid = pose.poseIsValid && pose.deviceIsConnected && pose.result == vr::TrackingResult_Running_OK;
			vr::HmdQuaternion_t rawRotation = { 1, 0, 0, 0 };
			double rawPosition[3] = { 0, 0, 0 };
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

			double displayHz = GetCachedDisplayHz(openVRID);

			vr::HmdQuaternion_t trackerQuat = { 1, 0, 0, 0 };
			double rawTrackerPos[3] = { 0, 0, 0 };
			double trackerVel[3] = { 0, 0, 0 };
			double trackerAngVel[3] = { 0, 0, 0 };
			double linSpeed = 0.0;
			const bool trackerPoseOk = FetchTrackerSample(trackerQuat, rawTrackerPos, trackerVel, trackerAngVel, linSpeed);
			const bool speedReject = !trackerPoseOk && linSpeed > 8.0;

			// P2-b: first solid OK starts "play" counters (ignore pre-cal BAD noise).
			if (trackerPoseOk && !diag.playPrimed)
			{
				diag.playPrimed = true;
				diag.frames = 0;
				diag.trackerOkFrames = 0;
				diag.trackerBadFrames = 0;
				diag.jumpEvents = 0;
				diag.jumpHolds = 0;
				diag.lastGoodHolds = 0;
				diag.speedRejects = 0;
				diag.fallbackFrames = 0;
				diag.nonFiniteDrops = 0;
				LOG("play primed (first solid tracker OK) — diag counters reset");
			}

			++diag.frames;
			if (trackerPoseOk)
				++diag.trackerOkFrames;
			else
				++diag.trackerBadFrames;
			if (speedReject)
				++diag.speedRejects;

			// Rate-limited state transitions.
			{
				LARGE_INTEGER now{}, freq{};
				QueryPerformanceCounter(&now);
				QueryPerformanceFrequency(&freq);

				if (!diag.trackerStateKnown || diag.lastTrackerOk != trackerPoseOk)
				{
					bool allowLog = true;
					if (diag.lastBadLog.QuadPart != 0)
					{
						double since = (now.QuadPart - diag.lastBadLog.QuadPart) / (double)freq.QuadPart;
						if (since < 0.25)
							allowLog = false;
					}
					if (allowLog)
					{
						LOG("tracker %s speed=%.2f reject_speed=%d cache=%d",
							trackerPoseOk ? "OK" : "BAD",
							linSpeed,
							speedReject ? 1 : 0,
							cachedTracker.valid ? 1 : 0);
						diag.lastBadLog = now;
					}
					diag.lastTrackerOk = trackerPoseOk;
					diag.trackerStateKnown = true;
				}

				if (diag.lastHeartbeat.QuadPart == 0)
					diag.lastHeartbeat = now;
				double hb = (now.QuadPart - diag.lastHeartbeat.QuadPart) / (double)freq.QuadPart;
				if (hb >= 60.0)
				{
					LOG("heartbeat frames=%llu ok=%llu bad=%llu jumps=%llu holds=%llu lg_holds=%llu speed_rej=%llu fallback=%llu nonfinite=%llu enabled=%d",
						(unsigned long long)diag.frames,
						(unsigned long long)diag.trackerOkFrames,
						(unsigned long long)diag.trackerBadFrames,
						(unsigned long long)diag.jumpEvents,
						(unsigned long long)diag.jumpHolds,
						(unsigned long long)diag.lastGoodHolds,
						(unsigned long long)diag.speedRejects,
						(unsigned long long)diag.fallbackFrames,
						(unsigned long long)diag.nonFiniteDrops,
						hmdTracker.enabled ? 1 : 0);
					diag.lastHeartbeat = now;
				}
			}

			if (trackerPoseOk)
			{
				vr::HmdQuaternion_t trackerRefRotation = quaternionNormalize(hmdTracker.calibrationRotation * trackerQuat);

				// Local prediction from velocity (honors predictionTime without GetRaw predict).
				float predFrames = hmdTracker.predictionTime;
				if (predFrames < 0.0f) predFrames = 0.0f;
				if (predFrames > 10.0f) predFrames = 10.0f;
				const double predSec = (1.0 / displayHz) * (double)predFrames;
				if (predSec > 0.0)
				{
					rawTrackerPos[0] += trackerVel[0] * predSec;
					rawTrackerPos[1] += trackerVel[1] * predSec;
					rawTrackerPos[2] += trackerVel[2] * predSec;
				}

				vr::HmdVector3d_t filteredTrackerPos = trackerFilter.translation.update({
					rawTrackerPos[0],
					rawTrackerPos[1],
					rawTrackerPos[2]
				});
				double trackerPos[3] = {
					filteredTrackerPos.v[0],
					filteredTrackerPos.v[1],
					filteredTrackerPos.v[2]
				};

				vr::HmdVector3d_t trackerRefPosition = quaternionRotateVector(hmdTracker.calibrationRotation, trackerPos);

				trackerRefPosition.v[0] += hmdTracker.calibrationTranslation.v[0];
				trackerRefPosition.v[1] += hmdTracker.calibrationTranslation.v[1];
				trackerRefPosition.v[2] += hmdTracker.calibrationTranslation.v[2];

				vr::HmdQuaternion_t hmdRotation = quaternionNormalize(hmdTracker.native ? trackerQuat * hmdTracker.offsetRotation : trackerRefRotation * hmdTracker.offsetRotation);
				vr::HmdVector3d_t offset = quaternionRotateVector(hmdTracker.native ? trackerQuat : trackerRefRotation, hmdTracker.offsetTranslation.v);

				pose.qWorldFromDriverRotation = { 1, 0, 0, 0 };
				pose.vecWorldFromDriverTranslation[0] = 0;
				pose.vecWorldFromDriverTranslation[1] = 0;
				pose.vecWorldFromDriverTranslation[2] = 0;

				pose.qDriverFromHeadRotation = { 1, 0, 0, 0 };
				pose.vecDriverFromHeadTranslation[0] = 0;
				pose.vecDriverFromHeadTranslation[1] = 0;
				pose.vecDriverFromHeadTranslation[2] = 0;

				if (hmdTracker.native) {
					pose.qRotation = hmdRotation;
					pose.vecPosition[0] = trackerPos[0] + offset.v[0];
					pose.vecPosition[1] = trackerPos[1] + offset.v[1];
					pose.vecPosition[2] = trackerPos[2] + offset.v[2];
				}
				else {
					pose.qRotation = hmdRotation;
					pose.vecPosition[0] = trackerRefPosition.v[0] + offset.v[0];
					pose.vecPosition[1] = trackerRefPosition.v[1] + offset.v[1];
					pose.vecPosition[2] = trackerRefPosition.v[2] + offset.v[2];
				}

				if (headFilter.enabled)
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

				vr::HmdVector3d_t vel = quaternionRotateVector(hmdTracker.calibrationRotation, trackerVel);
				vel.v[0] *= hmdTracker.calibrationScale;
				vel.v[1] *= hmdTracker.calibrationScale;
				vel.v[2] *= hmdTracker.calibrationScale;

				double dtAng = FilterStep(headVel.lastUpdate, headVel.valid);
				vr::HmdVector3d_t headAngVel = { 0, 0, 0 };
				if (headVel.valid)
					headAngVel = headVel.filter.filter(quaternionAngularVelocity(pose.qRotation, headVel.prevRotation, dtAng), dtAng);
				headVel.prevRotation = pose.qRotation;
				headVel.valid = true;

				vr::HmdVector3d_t tangential = {
					headAngVel.v[1] * offset.v[2] - headAngVel.v[2] * offset.v[1],
					headAngVel.v[2] * offset.v[0] - headAngVel.v[0] * offset.v[2],
					headAngVel.v[0] * offset.v[1] - headAngVel.v[1] * offset.v[0]
				};

				for (int i = 0; i < 3; i++)
				{
					double baseVel = hmdTracker.native ? trackerVel[i] : vel.v[i];
					pose.vecVelocity[i] = baseVel + tangential.v[i];
					pose.vecAngularVelocity[i] = hmdTracker.enableAngularVelocity ? headAngVel.v[i] : 0.0;
				}

				pose.poseIsValid = true;
				pose.deviceIsConnected = true;
				pose.result = vr::TrackingResult_Running_OK;
				pose.shouldApplyHeadModel = false;
				pose.poseTimeOffset = 0;

				// P0-b: non-finite → hold last good or drop.
				const double candPos[3] = { pose.vecPosition[0], pose.vecPosition[1], pose.vecPosition[2] };
				if (!IsFiniteQuat(pose.qRotation) || !IsFiniteVec3(candPos))
				{
					++diag.nonFiniteDrops;
					if (ApplyLastGoodHmd(pose, 0.15))
					{
						++diag.lastGoodHolds;
						return true;
					}
					pose.poseIsValid = false;
					pose.result = vr::TrackingResult_Running_OutOfRange;
					return true;
				}

				// P0-a: pre-publish jump gate — do not emit teleports.
				double dtPos = 1.0 / displayHz;
				if (diag.haveLastHmdPos && diag.lastHmdPosTime.QuadPart != 0)
				{
					LARGE_INTEGER now{}, freq{};
					QueryPerformanceCounter(&now);
					QueryPerformanceFrequency(&freq);
					double dt = (now.QuadPart - diag.lastHmdPosTime.QuadPart) / (double)freq.QuadPart;
					if (dt > 1e-4 && dt < 0.1)
						dtPos = dt;
				}

				if (ShouldHoldForJump(candPos, dtPos))
				{
					const double dx = candPos[0] - lastGoodHmd.position[0];
					const double dy = candPos[1] - lastGoodHmd.position[1];
					const double dz = candPos[2] - lastGoodHmd.position[2];
					const double dist = lastGoodHmd.valid ? sqrt(dx * dx + dy * dy + dz * dz) : 0.0;
					++diag.jumpEvents;
					++diag.jumpHolds;
					LARGE_INTEGER now{}, freq{};
					QueryPerformanceCounter(&now);
					QueryPerformanceFrequency(&freq);
					bool allow = true;
					if (diag.lastJumpLog.QuadPart != 0)
					{
						double since = (now.QuadPart - diag.lastJumpLog.QuadPart) / (double)freq.QuadPart;
						if (since < 0.5)
							allow = false;
					}
					if (allow)
					{
						LOG("HMD_JUMP_HOLD dist=%.3fm dt=%.4f speed=%.2f (held last good)",
							dist, dtPos, linSpeed);
						diag.lastJumpLog = now;
					}
					if (ApplyLastGoodHmd(pose, 0.25))
						return true;
					// No last good yet — fall through and accept first frame.
				}

				StoreLastGoodHmd(pose);
				diag.lastHmdPos[0] = pose.vecPosition[0];
				diag.lastHmdPos[1] = pose.vecPosition[1];
				diag.lastHmdPos[2] = pose.vecPosition[2];
				QueryPerformanceCounter(&diag.lastHmdPosTime);
				diag.haveLastHmdPos = true;

				if (rawValid)
				{
					double angSpeed = sqrt(
						trackerAngVel[0] * trackerAngVel[0] +
						trackerAngVel[1] * trackerAngVel[1] +
						trackerAngVel[2] * trackerAngVel[2]);

					const double softLin = 1.5, hardLin = 3.5;
					const double softAng = 2.0, hardAng = 5.0;
					double weight = SoftGateWeight(linSpeed, softLin, hardLin)
						* SoftGateWeight(angSpeed, softAng, hardAng);

					if (!drift.valid)
						UpdateDrift(pose.qRotation, pose.vecPosition, rawRotation, rawPosition, 1.0);
					else if (weight > 0.01)
						UpdateDrift(pose.qRotation, pose.vecPosition, rawRotation, rawPosition, weight);
				}
			}
			else {
				// P1-a: brief LOS — hold last good up to 150 ms before slam fallback.
				headVel.reset();
				if (ApplyLastGoodHmd(pose, 0.15))
				{
					++diag.lastGoodHolds;
					return true;
				}

				if (hmdTracker.slamFallback && drift.valid)
					++diag.fallbackFrames;
				if (!hmdTracker.slamFallback) {
					if (hmdTracker.native) {
						pose.qWorldFromDriverRotation = { 1, 0, 0, 0 };
						pose.vecWorldFromDriverTranslation[0] = 0;
						pose.vecWorldFromDriverTranslation[1] = 0;
						pose.vecWorldFromDriverTranslation[2] = 0;
					}
					else {
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
				else if (drift.valid) {
					ApplyDrift(pose);
				}
			}
		}
		else if (slamSync[openVRID] && drift.valid)
		{
			ApplyDrift(pose);
		}
	}

	return true;
}