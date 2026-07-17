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
	LOG("auto-log: events=tracker_ok/bad, speed_reject, hmd_jump>0.35m, enable/disable, 60s heartbeat");

	memset(transforms, 0, vr::k_unMaxTrackedDeviceCount * sizeof(DeviceTransform));
	memset(slamSync, 0, sizeof slamSync);
	diag = SessionDiag{};

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
	LOG("session stats: frames=%llu tracker_ok=%llu tracker_bad=%llu jumps=%llu speed_reject=%llu fallback=%llu",
		(unsigned long long)diag.frames,
		(unsigned long long)diag.trackerOkFrames,
		(unsigned long long)diag.trackerBadFrames,
		(unsigned long long)diag.jumpEvents,
		(unsigned long long)diag.speedRejects,
		(unsigned long long)diag.fallbackFrames);
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
	hmdTracker.calibrationScale = cmd.calibrationScale > 0.0 ? cmd.calibrationScale : 1.0;
	hmdTracker.hmdScale = cmd.hmdScale > 0.0 ? cmd.hmdScale : 1.0;

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

bool ServerTrackedDeviceProvider::HandleDevicePoseUpdated(uint32_t openVRID, vr::DriverPose_t& pose)
{
	if (openVRID >= vr::k_unMaxTrackedDeviceCount)
		return true;

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

			vr::PropertyContainerHandle_t container = vr::VRProperties()->TrackedDeviceToPropertyContainer(openVRID);

			double displayHz = vr::VRProperties()->GetFloatProperty(container, vr::Prop_DisplayFrequency_Float);
			if (displayHz < 1.0)
				displayHz = 90.0;

			// Unpredicted fetch: predicted GetRaw can teleport on a bad velocity sample.
			// Compositor reprojection uses the velocities we write on the HMD pose.
			vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount];
			vr::VRServerDriverHost()->GetRawTrackedDevicePoses(0.0f, poses, vr::k_unMaxTrackedDeviceCount);

			static const vr::TrackedDevicePose_t invalidTrackerPose{};
			const auto& tp = (hmdTracker.trackerID < vr::k_unMaxTrackedDeviceCount)
				? poses[hmdTracker.trackerID]
				: invalidTrackerPose;

			double trackerVel[3] = { tp.vVelocity.v[0], tp.vVelocity.v[1], tp.vVelocity.v[2] };
			double trackerAngVel[3] = { tp.vAngularVelocity.v[0], tp.vAngularVelocity.v[1], tp.vAngularVelocity.v[2] };
			const double linSpeed = sqrt(
				trackerVel[0] * trackerVel[0] +
				trackerVel[1] * trackerVel[1] +
				trackerVel[2] * trackerVel[2]);

			// bPoseIsValid alone is not enough; reject impossible kinematics (optical glitches).
			const double maxAcceptLinSpeed = 8.0;
			const bool speedReject = tp.bPoseIsValid && tp.bDeviceIsConnected
				&& tp.eTrackingResult == vr::TrackingResult_Running_OK
				&& linSpeed > maxAcceptLinSpeed;
			const bool trackerPoseOk = tp.bPoseIsValid
				&& tp.bDeviceIsConnected
				&& tp.eTrackingResult == vr::TrackingResult_Running_OK
				&& linSpeed <= maxAcceptLinSpeed;

			++diag.frames;
			if (trackerPoseOk)
				++diag.trackerOkFrames;
			else
				++diag.trackerBadFrames;
			if (speedReject)
				++diag.speedRejects;

			// Rate-limited state transitions (auto-log, no user action).
			{
				LARGE_INTEGER now{}, freq{};
				QueryPerformanceCounter(&now);
				QueryPerformanceFrequency(&freq);
				const double nowSec = now.QuadPart / (double)freq.QuadPart;

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
						LOG("tracker %s valid=%d connected=%d result=%d speed=%.2f reject_speed=%d",
							trackerPoseOk ? "OK" : "BAD",
							tp.bPoseIsValid ? 1 : 0,
							tp.bDeviceIsConnected ? 1 : 0,
							(int)tp.eTrackingResult,
							linSpeed,
							speedReject ? 1 : 0);
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
					LOG("heartbeat frames=%llu ok=%llu bad=%llu jumps=%llu speed_rej=%llu fallback=%llu enabled=%d",
						(unsigned long long)diag.frames,
						(unsigned long long)diag.trackerOkFrames,
						(unsigned long long)diag.trackerBadFrames,
						(unsigned long long)diag.jumpEvents,
						(unsigned long long)diag.speedRejects,
						(unsigned long long)diag.fallbackFrames,
						hmdTracker.enabled ? 1 : 0);
					diag.lastHeartbeat = now;
				}
				(void)nowSec;
			}

			if (trackerPoseOk)
			{
				vr::HmdQuaternion_t trackerQuat = HmdQuaternion_FromMatrix(tp.mDeviceToAbsoluteTracking);

				vr::HmdQuaternion_t trackerRefRotation = quaternionNormalize(hmdTracker.calibrationRotation * trackerQuat);

				double rawTrackerPos[3] = {
					tp.mDeviceToAbsoluteTracking.m[0][3],
					tp.mDeviceToAbsoluteTracking.m[1][3],
					tp.mDeviceToAbsoluteTracking.m[2][3]
				};

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

				// Detect large HMD world jumps (meters) after override write.
				{
					const double px = pose.vecPosition[0];
					const double py = pose.vecPosition[1];
					const double pz = pose.vecPosition[2];
					if (diag.haveLastHmdPos)
					{
						const double dx = px - diag.lastHmdPos[0];
						const double dy = py - diag.lastHmdPos[1];
						const double dz = pz - diag.lastHmdPos[2];
						const double dist = sqrt(dx * dx + dy * dy + dz * dz);
						const double jumpThresh = 0.35; // 35 cm in one pose update is not human
						if (dist > jumpThresh)
						{
							++diag.jumpEvents;
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
								LOG("HMD_JUMP dist=%.3fm pos=(%.2f,%.2f,%.2f) prev=(%.2f,%.2f,%.2f) speed=%.2f pred=%.2f",
									dist, px, py, pz,
									diag.lastHmdPos[0], diag.lastHmdPos[1], diag.lastHmdPos[2],
									linSpeed, hmdTracker.predictionTime);
								diag.lastJumpLog = now;
							}
						}
					}
					diag.lastHmdPos[0] = px;
					diag.lastHmdPos[1] = py;
					diag.lastHmdPos[2] = pz;
					diag.haveLastHmdPos = true;
				}

				if (rawValid)
				{
					double angSpeed = sqrt(
						trackerAngVel[0] * trackerAngVel[0] +
						trackerAngVel[1] * trackerAngVel[1] +
						trackerAngVel[2] * trackerAngVel[2]);

					// Soft gate: full weight below soft caps, zero past hard caps, blend between.
					// Avoids hard lock→jump when turning quickly through the old 2.75/3.5 cliffs.
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
				// Brief LOS/glitch: keep Kalman so reacquire does not snap from a reset filter.
				headVel.reset();
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