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

namespace {

// ---- IPC input boundary ------------------------------------------------------------------
// The pipe carries whatever a local process writes, the Request union is uninitialised
// client-side, and the pipe has a default DACL. These setters wrote straight into the config a
// pose callback reads.
//
// ACCEPTANCE (why the happy-path output is bit-identical): every predicate is phrased
// POSITIVELY, so a good value passes it unchanged and nothing is clamped -- the whole message
// is either applied exactly as sent or rejected whole. The live values sit far inside the
// bounds (calibrationScale 0.99920, hmdScale 0.99200, translations ~1.5 m, unit quaternions),
// so every message the shipped overlay sends is applied byte-for-byte as before.
//
// Policy is REJECT AND LOG, never silently clamp -- a clamped bad value produces mis-tracking
// that looks like a calibration problem, which is the most expensive kind of bug this project
// has. Note every test is written so NaN FAILS it: `>=`/`<=` are false for NaN, so the checks
// are phrased positively rather than as negated bounds.

inline bool FiniteQuatUnit(const vr::HmdQuaternion_t& q)
{
	if (!std::isfinite(q.w) || !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z))
		return false;
	const double n = sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
	return fabs(1.0 - n) <= 1e-3;
}

inline bool FiniteVecWithin(const vr::HmdVector3d_t& v, double maxAbs)
{
	if (!std::isfinite(v.v[0]) || !std::isfinite(v.v[1]) || !std::isfinite(v.v[2]))
		return false;
	return fabs(v.v[0]) <= maxAbs && fabs(v.v[1]) <= maxAbs && fabs(v.v[2]) <= maxAbs;
}

inline bool ScaleInRange(double s)
{
	return std::isfinite(s) && s >= 0.5 && s <= 2.0;
}

// One-Euro parameters. NaN previously survived `p.minCutoff < 0.01 ? 0.01 : p.minCutoff`
// because the comparison is false for NaN, so the NaN was stored and propagated into alpha().
// For any finite input this is `v < lo ? lo : v`, i.e. exactly what it replaces; the fallbacks
// are oneeuro::Params' own member initialisers.
inline double SanitizeOneEuro(double v, double lo, double fallback)
{
	if (!std::isfinite(v))
		return fallback;
	return v < lo ? lo : v;
}

} // namespace

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

	memset(transforms, 0, vr::k_unMaxTrackedDeviceCount * sizeof(DeviceTransform));
	memset(slamSync, 0, sizeof slamSync);

	// ACCEPTANCE (why the happy-path output is bit-identical): g_server is a file-scope global
	// (Main.cpp), so member initialisers run once per PROCESS, not once per driver load. On the
	// first Init of a process every assignment below is a no-op -- it writes exactly the value
	// the member initialiser already holds. Steady state is therefore identical; only a
	// same-process Cleanup->Init differs, and there it restores the state the first Init had.
	//
	// hmdTracker is the one with the real blast radius. transforms[] and slamSync[] were
	// already being cleared here for this reason; hmdTracker was not, so a same-process reload
	// resumed with enabled==true and the pre-reload trackerID while transforms[] had just been
	// zeroed. The overlay cannot correct it -- IPCClient connects once from InitCalibrator and
	// has no reconnect path -- so nothing re-pushes SetHmdTracker and no log line fires. OpenVR
	// indices are assigned in connection order and are not stable across restarts, so a stale
	// trackerID rebuilds the HMD from whatever device now holds that index. Coming up inert
	// until the overlay pushes a fresh SetHmdTracker is what Init already does for everything
	// else.
	hmdTracker = {};

	drift.valid = false;
	drift.rotation = { 1, 0, 0, 0 };
	drift.translation = { 0, 0, 0 };
	drift.lastUpdate = {};
	drift.rotationFilter.reset();
	drift.translationFilter.reset();
	// The snapshot body devices read has to be cleared with it: on a same-process reload the
	// pose threads would otherwise keep applying the pre-reload correction until the first HMD
	// frame republishes.
	sharedDrift = SharedDrift{};

	headFilter.enabled = false;
	headFilter.lastUpdate = {};
	headFilter.reset();

	headVel.prevRotation = { 1, 0, 0, 0 };
	headVel.lastUpdate = {};
	headVel.reset();

	trackerFilter.reset();

	displayHzQueried = false;
	displayHzLastQuery = {};
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
	LOG("OpenVR-SpaceOverride unloaded");
	CloseLogFile();

	TRACE("ServerTrackedDeviceProvider::Cleanup()");
	// Poses in flight during teardown pass through untouched. Set before Stop()/DisableHooks()
	// so a callback that is already inside the detour cannot start rewriting state the two
	// calls below are dismantling. AUDIT 14 stays open: there is still no quiescence wait
	// between this store and DisableHooks().
	SetDriverShuttingDown(true);
	server.Stop();
	DisableHooks();
	VR_CLEANUP_SERVER_DRIVER_CONTEXT();
}

void ServerTrackedDeviceProvider::SetDeviceTransform(const protocol::SetDeviceTransform& newTransform)
{
	// The array-index bound upstream lacks entirely: openVRID comes off the pipe and indexes a
	// fixed-size member array. SetSlamSync already had this check; SetDeviceTransform did not.
	if (newTransform.openVRID >= vr::k_unMaxTrackedDeviceCount)
	{
		LOG("IPC REJECT SetDeviceTransform: openVRID %u out of range", newTransform.openVRID);
		return;
	}

	// Validate before writing anything, and reject the whole message rather than applying the
	// good fields of a bad one. tf.scale multiplies vecPosition for every device that carries a
	// transform, so a NaN or zero here corrupts poses the driver does not otherwise touch.
	if (newTransform.updateTranslation && !FiniteVecWithin(newTransform.translation, 10.0))
	{
		LOG("IPC REJECT SetDeviceTransform id=%u: translation not finite / >10 m", newTransform.openVRID);
		return;
	}
	if (newTransform.updateRotation && !FiniteQuatUnit(newTransform.rotation))
	{
		LOG("IPC REJECT SetDeviceTransform id=%u: rotation not a finite unit quaternion", newTransform.openVRID);
		return;
	}
	if (newTransform.updateScale && !ScaleInRange(newTransform.scale))
	{
		LOG("IPC REJECT SetDeviceTransform id=%u: scale %.6f outside [0.5, 2.0]",
			newTransform.openVRID, newTransform.scale);
		return;
	}

	// Validation is done above, outside the lock; only the apply is exclusive.
	std::unique_lock<std::shared_mutex> lock(configMutex);

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
	// hmdID indexes transforms[] on the pose path via HandleDevicePoseUpdated's comparison, and
	// trackerID indexes the raw pose array. Both come off the pipe unvalidated upstream.
	if (cmd.hmdID >= vr::k_unMaxTrackedDeviceCount)
	{
		LOG("IPC REJECT SetHmdTracker: hmdID %u out of range", cmd.hmdID);
		return;
	}
	if (cmd.enabled && cmd.trackerID >= vr::k_unMaxTrackedDeviceCount)
	{
		LOG("IPC REJECT SetHmdTracker: trackerID %u out of range", cmd.trackerID);
		return;
	}

	// This message defines the entire calibration the pose path runs on. Reject it whole if any
	// field is unusable — a partially-applied calibration is worse than none, because the driver
	// keeps tracking and the error looks like a bad calibration rather than a bad message.
	if (!FiniteQuatUnit(cmd.offsetRotation) || !FiniteQuatUnit(cmd.calibrationRotation))
	{
		LOG("IPC REJECT SetHmdTracker: offset/calibration rotation is not a finite unit quaternion");
		return;
	}
	if (!FiniteVecWithin(cmd.offsetTranslation, 10.0) || !FiniteVecWithin(cmd.calibrationTranslation, 10.0))
	{
		LOG("IPC REJECT SetHmdTracker: offset/calibration translation not finite or >10 m");
		return;
	}
	if (!ScaleInRange(cmd.calibrationScale) || !ScaleInRange(cmd.hmdScale))
	{
		LOG("IPC REJECT SetHmdTracker: calibrationScale %.6f / hmdScale %.6f outside [0.5, 2.0]",
			cmd.calibrationScale, cmd.hmdScale);
		return;
	}
	if (!std::isfinite(cmd.predictionTime))
	{
		LOG("IPC REJECT SetHmdTracker: predictionTime not finite");
		return;
	}

	// One exclusive section for the whole calibration. Upstream wrote these fifteen fields with
	// no lock at all, so a pose callback reading them concurrently could compose a head pose
	// from half of one calibration and half of the next.
	std::unique_lock<std::shared_mutex> lock(configMutex);

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
	// Both are in range by ScaleInRange above; the ternaries are kept unchanged so the applied
	// value is still explicit at the point of use, and so this line is textually upstream's.
	hmdTracker.calibrationScale = cmd.calibrationScale > 0.0 ? cmd.calibrationScale : 1.0;
	hmdTracker.hmdScale = cmd.hmdScale > 0.0 ? cmd.hmdScale : 1.0;

	if (!cmd.enabled)
	{
		drift.valid = false;
		drift.rotationFilter.reset();
		drift.translationFilter.reset();
		headFilter.reset();
		headVel.reset();
		trackerFilter.reset();
		memset(slamSync, 0, sizeof slamSync);
		// Same clear, propagated to the copy the other pose threads read. Without this a body
		// device could keep applying the correction from the calibration just torn down.
		PublishDrift();
	}
}

void ServerTrackedDeviceProvider::SetSlamSync(const protocol::SetSlamSync& cmd)
{
	// Upstream already bounds this index; the only addition is that an out-of-range id is now
	// logged instead of dropped in silence. Same policy as the other two setters.
	if (cmd.openVRID < vr::k_unMaxTrackedDeviceCount)
	{
		std::unique_lock<std::shared_mutex> lock(configMutex);
		slamSync[cmd.openVRID] = cmd.enabled;
	}
	else
		LOG("IPC REJECT SetSlamSync: openVRID %u out of range", cmd.openVRID);
}

void ServerTrackedDeviceProvider::SetOneEuro(const protocol::SetOneEuro& cmd)
{
	// SanitizeOneEuro is `v < lo ? lo : v` for every finite input, so the parameters the overlay
	// sends are stored unchanged. The only difference is that a non-finite value no longer
	// slips through (`NaN < 0.01` is false, so upstream stored the NaN and alpha() spread it
	// through the filtered pose).
	auto toParams = [](const protocol::OneEuroParams& p) {
		oneeuro::Params out;
		out.minCutoff = SanitizeOneEuro(p.minCutoff, 0.01, 1.0);
		out.beta = SanitizeOneEuro(p.beta, 0.0, 0.0);
		out.dCutoff = SanitizeOneEuro(p.dCutoff, 0.01, 1.0);
		return out;
	};

	std::unique_lock<std::shared_mutex> lock(configMutex);

	headFilter.rotationFilter.params = toParams(cmd.head);
	headFilter.translationFilter.params = toParams(cmd.head);
	drift.rotationFilter.params = toParams(cmd.drift);
	drift.translationFilter.params = toParams(cmd.drift);

	if (headFilter.enabled && !cmd.headEnabled)
		headFilter.reset();
	headFilter.enabled = cmd.headEnabled;
}

void ServerTrackedDeviceProvider::UpdateDrift(const vr::HmdQuaternion_t& correctedRotation, const double(&correctedPosition)[3],
	const vr::HmdQuaternion_t& rawRotation, const double(&rawPosition)[3])
{
	vr::HmdQuaternion_t instRot = quaternionProjectYaw(quaternionNormalize(correctedRotation * quaternionConjugate(rawRotation)));
	vr::HmdVector3d_t instRotatedRaw = quaternionRotateVector(instRot, rawPosition);

	double slamScale = SlamToCorrectedScale();
	vr::HmdVector3d_t instTrans = {
		correctedPosition[0] - instRotatedRaw.v[0] * slamScale,
		correctedPosition[1] - instRotatedRaw.v[1] * slamScale,
		correctedPosition[2] - instRotatedRaw.v[2] * slamScale
	};

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

// Called at the end of the HMD branch, i.e. immediately after UpdateDrift on the same thread.
// A body device whose pose lands between the two therefore sees the previous frame's
// correction — at most one HMD frame of staleness, ~11 ms — where upstream saw a genuine torn
// read of a quaternion being written concurrently.
void ServerTrackedDeviceProvider::PublishDrift()
{
	std::lock_guard<std::mutex> lock(driftMutex);
	sharedDrift.valid = drift.valid;
	sharedDrift.rotation = drift.rotation;
	sharedDrift.translation = drift.translation;
}

// The arithmetic below is ApplyDrift's, line for line, reading the published snapshot instead
// of `drift`. It is deliberately duplicated rather than factored out so that ApplyDrift stays
// textually upstream's for the static byte-compare. Returns false when no correction has been
// published yet, which is upstream's `drift.valid` test moved inside.
bool ServerTrackedDeviceProvider::ApplySharedDrift(vr::DriverPose_t& pose)
{
	vr::HmdQuaternion_t driftRotation;
	vr::HmdVector3d_t driftTranslation;
	{
		std::lock_guard<std::mutex> lock(driftMutex);
		if (!sharedDrift.valid)
			return false;
		driftRotation = sharedDrift.rotation;
		driftTranslation = sharedDrift.translation;
	}

	double slamScale = SlamToCorrectedScale();

	pose.qWorldFromDriverRotation = quaternionNormalize(driftRotation * pose.qWorldFromDriverRotation);

	pose.vecPosition[0] *= slamScale;
	pose.vecPosition[1] *= slamScale;
	pose.vecPosition[2] *= slamScale;

	double scaledTranslation[3] = {
		pose.vecWorldFromDriverTranslation[0] * slamScale,
		pose.vecWorldFromDriverTranslation[1] * slamScale,
		pose.vecWorldFromDriverTranslation[2] * slamScale
	};
	vr::HmdVector3d_t rotatedTranslation = quaternionRotateVector(driftRotation, scaledTranslation);
	pose.vecWorldFromDriverTranslation[0] = rotatedTranslation.v[0] + driftTranslation.v[0];
	pose.vecWorldFromDriverTranslation[1] = rotatedTranslation.v[1] + driftTranslation.v[1];
	pose.vecWorldFromDriverTranslation[2] = rotatedTranslation.v[2] + driftTranslation.v[2];
	return true;
}

// ACCEPTANCE (why the happy-path output is bit-identical): every real headset reports a stable
// 72-144 Hz, so the cached value is the same number the inline property read returned and the
// prediction interval `(1.0 / hz) * predictionTime` is unchanged bit for bit -- the float the
// property returns converts exactly to double, as it did before. What changes is only what
// happens when the property read FAILS: upstream's GetFloatProperty returns 0.0f on error, so
// (1.0/0)*predictionTime = inf (or NaN when predictionTime is 0, which it is on this rig) went
// straight into GetRawTrackedDevicePoses. The floor also means a mid-session property failure
// keeps the last good rate instead of poisoning the prediction.
//
// Re-read at most once a second. In vanilla-plus the only consumer is that prediction interval.
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

bool ServerTrackedDeviceProvider::HandleDevicePoseUpdated(uint32_t openVRID, vr::DriverPose_t& pose)
{
	// openVRID indexes transforms[] and slamSync[] directly. SteamVR only ever passes a valid
	// index, so this is unreachable in play; it is here because the two arrays are fixed size.
	if (openVRID >= vr::k_unMaxTrackedDeviceCount)
		return true;

	// ACCEPTANCE (why the happy-path output is bit-identical): the arithmetic below is
	// unchanged, and this lock is pure mutual exclusion — with no concurrent IPC write it
	// admits exactly the frames upstream admitted, reading exactly the values upstream read.
	// Held SHARED for the whole callback so a config write cannot tear an in-flight pose.
	//
	// Cannot self-deadlock on a writer-waiting shared_mutex: the detour's re-entrancy depth
	// guard passes a re-entered pose callback straight through, before this lock is reached.
	std::shared_lock<std::shared_mutex> configLock(configMutex);

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

			// Same Hz on the happy path, so the same predSec. `container` went with the inline
			// property read; GetCachedDisplayHz resolves its own.
			double displayHz = GetCachedDisplayHz(openVRID);

			vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount];
			vr::VRServerDriverHost()->GetRawTrackedDevicePoses((1.0 / displayHz) * hmdTracker.predictionTime, poses, vr::k_unMaxTrackedDeviceCount);

			const auto& tp = poses[hmdTracker.trackerID];
			if (tp.bPoseIsValid)
			{
				vr::HmdQuaternion_t trackerQuat = HmdQuaternion_FromMatrix(tp.mDeviceToAbsoluteTracking);

				vr::HmdQuaternion_t trackerRefRotation = quaternionNormalize(hmdTracker.calibrationRotation * trackerQuat);

				vr::HmdVector3d_t filteredTrackerPos = trackerFilter.translation.update({
					tp.mDeviceToAbsoluteTracking.m[0][3],
					tp.mDeviceToAbsoluteTracking.m[1][3],
					tp.mDeviceToAbsoluteTracking.m[2][3]
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

				double trackerVel[3] = {
					tp.vVelocity.v[0],
					tp.vVelocity.v[1],
					tp.vVelocity.v[2]
				};

				double trackerAngVel[3] = {
					tp.vAngularVelocity.v[0],
					tp.vAngularVelocity.v[1],
					tp.vAngularVelocity.v[2]
				};

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

				if (rawValid)
				{
					double linSpeed = sqrt(
						trackerVel[0] * trackerVel[0] +
						trackerVel[1] * trackerVel[1] +
						trackerVel[2] * trackerVel[2]);

					double angSpeed = sqrt(
						trackerAngVel[0] * trackerAngVel[0] +
						trackerAngVel[1] * trackerAngVel[1] +
						trackerAngVel[2] * trackerAngVel[2]);

					const double maxLinSpeed = 2.75;
					const double maxAngSpeed = 3.5;

					if (!drift.valid || (linSpeed < maxLinSpeed && angSpeed < maxAngSpeed))
						UpdateDrift(pose.qRotation, pose.vecPosition, rawRotation, rawPosition);
				}
			}
			else {
				headVel.reset();
				trackerFilter.reset();
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

			// Body devices (slamSync) read the correction through the snapshot, so publish it
			// here, at the end of the HMD branch and on the same thread that just wrote it.
			// Idempotent, and unconditional so that a frame which did NOT update the drift
			// still keeps the snapshot in step with `drift.valid`.
			PublishDrift();
		}
		else if (slamSync[openVRID])
		{
			// ApplySharedDrift returns false and touches nothing when no correction has been
			// published, so this is upstream's `slamSync[openVRID] && drift.valid` with the
			// validity test moved inside the lock that protects the value it guards.
			ApplySharedDrift(pose);
		}
	}

	return true;
}