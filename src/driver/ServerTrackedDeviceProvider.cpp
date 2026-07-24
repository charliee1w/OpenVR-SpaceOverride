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
	LOG("gates: isfinite, pre-publish jump + reconverge<=3m/s, last-good<=150ms, tracker-hook-cache (no GetRaw while quash)");

	memset(transforms, 0, vr::k_unMaxTrackedDeviceCount * sizeof(DeviceTransform));
	memset(slamSync, 0, sizeof slamSync);
	diag = SessionDiag{};
	lastGoodHmd = LastGoodHmd{};
	cachedTracker = CachedTrackerPose{};
	fusion = FusionState{};
	ekf = FusionEkf{};
	displayHzQueried = false;
	cachedDisplayHz = 90.0;

	{
		vr::EVRSettingsError serr = vr::VRSettingsError_None;
		bool f = vr::VRSettings()->GetBool("driver_spaceoverride", "fusionMode", &serr);
		fusionMode = (serr == vr::VRSettingsError_None) ? f : false;
		serr = vr::VRSettingsError_None;
		bool d = vr::VRSettings()->GetBool("driver_spaceoverride", "fusionDiag", &serr);
		fusionDiag = (serr == vr::VRSettingsError_None) ? d : false;
	}
	LOG("mode: %s", fusionMode
		? "FUSION (SLAM source, tracker observes correction, slam-step cancel)"
		: "OVERRIDE (tracker source, classic)");

	if (fusionMode && fusionDiag)
	{
		OpenDiagCsv();
		QueryPerformanceCounter(&diagStart);
		diagLastWrite = LARGE_INTEGER{};
		LOG("fusion per-frame diagnostic CSV enabled: %s", DiagCsvOpen() ? "open" : "FAILED to open");
	}

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

void ServerTrackedDeviceProvider::RunFrame()
{
	// Live driver settings, polled ~1 Hz. The overlay writes them via IVRSettings,
	// so mode switching needs no protocol change and no SteamVR restart.
	LARGE_INTEGER now{}, freq{};
	QueryPerformanceCounter(&now);
	QueryPerformanceFrequency(&freq);
	if (settingsLastPoll.QuadPart != 0)
	{
		double since = (now.QuadPart - settingsLastPoll.QuadPart) / (double)freq.QuadPart;
		if (since < 1.0)
			return;
	}
	settingsLastPoll = now;

	vr::EVRSettingsError serr = vr::VRSettingsError_None;
	const bool wantFusion = vr::VRSettings()->GetBool("driver_spaceoverride", "fusionMode", &serr);
	if (serr == vr::VRSettingsError_None && wantFusion != fusionMode)
	{
		{
			std::unique_lock<std::shared_mutex> lock(configMutex);
			fusionMode = wantFusion;
			// Whichever estimator takes over re-converges from scratch; the V1 publish
			// gate bounds the visual transition to <= 3 m/s, so the switch is a slew.
			ekf = FusionEkf{};
			fusion = FusionState{};
			drift.valid = false;
			drift.rotationFilter.reset();
			drift.translationFilter.reset();
		}
		LOG("mode switched -> %s (live, via overlay)", fusionMode ? "FUSION" : "OVERRIDE");
	}

	serr = vr::VRSettingsError_None;
	const bool wantDiag = vr::VRSettings()->GetBool("driver_spaceoverride", "fusionDiag", &serr);
	if (serr == vr::VRSettingsError_None && wantDiag != fusionDiag)
	{
		fusionDiag = wantDiag;
		if (fusionDiag)
		{
			OpenDiagCsv();
			QueryPerformanceCounter(&diagStart);
			diagLastWrite = LARGE_INTEGER{};
		}
		else
		{
			CloseDiagCsv();
		}
		LOG("fusion diagnostic CSV -> %s", fusionDiag ? "on" : "off");
	}
}

void ServerTrackedDeviceProvider::Cleanup()
{
	LOG("session stats: frames=%llu ok=%llu bad=%llu jumps=%llu jump_holds=%llu last_good_holds=%llu speed_rej=%llu fallback=%llu nonfinite=%llu play_primed=%d mode=%c slam_steps=%llu",
		(unsigned long long)diag.frames,
		(unsigned long long)diag.trackerOkFrames,
		(unsigned long long)diag.trackerBadFrames,
		(unsigned long long)diag.jumpEvents,
		(unsigned long long)diag.jumpHolds,
		(unsigned long long)diag.lastGoodHolds,
		(unsigned long long)diag.speedRejects,
		(unsigned long long)diag.fallbackFrames,
		(unsigned long long)diag.nonFiniteDrops,
		diag.playPrimed ? 1 : 0,
		fusionMode ? 'F' : 'O',
		(unsigned long long)diag.slamSteps);
	LOG("OpenVR-SpaceOverride unloading");
	SetDriverShuttingDown(true);
	server.Stop();
	DisableHooks();
	VR_CLEANUP_SERVER_DRIVER_CONTEXT();
	LOG("OpenVR-SpaceOverride unloaded");
	CloseDiagCsv();
	CloseLogFile();
}

void ServerTrackedDeviceProvider::SetDeviceTransform(const protocol::SetDeviceTransform& newTransform)
{
	if (newTransform.openVRID >= vr::k_unMaxTrackedDeviceCount)
		return;

	// V2-b: exclusive vs pose threads.
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
	if (cmd.hmdID >= vr::k_unMaxTrackedDeviceCount)
		return;

	// Disable uses k_unTrackedDeviceIndexInvalid; only validate trackerID when enabling.
	if (cmd.enabled && cmd.trackerID >= vr::k_unMaxTrackedDeviceCount)
		return;

	const double newScale = cmd.calibrationScale > 0.0 ? cmd.calibrationScale : 1.0;
	const double newHmdScale = cmd.hmdScale > 0.0 ? cmd.hmdScale : 1.0;

	// V2-b: exclusive vs pose threads (which hold configMutex shared for the whole
	// pose callback) — config writes and filter resets can't tear an in-flight pose.
	std::unique_lock<std::shared_mutex> lock(configMutex);

	const bool trackerChanged = hmdTracker.trackerID != cmd.trackerID;

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
		reconverging = false;
		fusion = FusionState{};
		// Reopen the EKF: covariance wide, correction re-learned from scratch.
		ekf.valid = false;
		ekf.Pt = 1.0;
		ekf.Ptheta = 1.0;
		ekf.outlierRun = 0;
		std::lock_guard<std::mutex> cacheLock(trackerCacheMutex);
		cachedTracker.valid = false;
	}
	else if (trackerChanged)
	{
		lastGoodHmd.valid = false;
		diag.haveLastHmdPos = false;
		reconverging = false;
		fusion = FusionState{};
		ekf.valid = false;
		ekf.Pt = 1.0;
		ekf.Ptheta = 1.0;
		ekf.outlierRun = 0;
		std::lock_guard<std::mutex> cacheLock(trackerCacheMutex);
		cachedTracker.valid = false;
	}

	lock.unlock();

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
			newScale,
			newHmdScale,
			cmd.enableAngularVelocity ? 1 : 0);
	}
}

void ServerTrackedDeviceProvider::SetSlamSync(const protocol::SetSlamSync& cmd)
{
	if (cmd.openVRID < vr::k_unMaxTrackedDeviceCount)
	{
		// V2-b: exclusive vs pose threads.
		std::unique_lock<std::shared_mutex> lock(configMutex);
		slamSync[cmd.openVRID] = cmd.enabled;
	}
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

	// V2-b: exclusive vs pose threads (filter params are read mid-frame).
	std::unique_lock<std::shared_mutex> lock(configMutex);

	headFilter.rotationFilter.params = toParams(cmd.head);
	headFilter.translationFilter.params = toParams(cmd.head);
	// Note: in fusion mode the EKF writes the correction directly; these One-Euro
	// drift filters are only used by the classic override path.
	drift.rotationFilter.params = toParams(cmd.drift);
	drift.translationFilter.params = toParams(cmd.drift);

	if (headFilter.enabled && !cmd.headEnabled)
		headFilter.reset();
	headFilter.enabled = cmd.headEnabled;
}

inline vr::HmdQuaternion_t yawQuaternion(double angle)
{
	return { cos(angle * 0.5), 0.0, sin(angle * 0.5), 0.0 };
}

bool ServerTrackedDeviceProvider::FusionEkfUpdate(
	const vr::HmdQuaternion_t& obsRot, const double obsPos[3],
	const vr::HmdQuaternion_t& rawRot, const double rawPos[3],
	double linSpeed, double angSpeed)
{
	// Random-walk process noise (per second): SLAM drifts on the order of mm/s
	// and fractions of a degree/min against lighthouse.
	const double Qtheta = 3e-6;   // rad^2/s
	const double Qt = 4e-6;       // m^2/s
	// Measurement noise at rest (post tracker-Kalman): ~4 mm, ~0.5 deg.
	const double Rtheta0 = 7.6e-5;
	const double Rt0 = 1.6e-5;
	// Motion inflation: tracker-vs-SLAM latency skew shows up as position/angle
	// error proportional to speed. Absorbing it into R (rather than correcting a
	// time-offset) is what makes the estimator robust during motion.
	const double kv = 0.008;      // s of position error per m/s
	const double kw = 0.010;      // s of angle error per rad/s
	const double gateT = 16.0;    // ~chi^2(3) 99.9%
	const double gateTheta = 9.0; // ~chi^2(1) 99.7%

	double dt = FilterStep(ekf.lastUpdate, ekf.valid);
	if (ekf.valid)
	{
		ekf.Ptheta += Qtheta * dt;
		ekf.Pt += Qt * dt;
	}

	const double slamScale = SlamToCorrectedScale();
	const double scaledRaw[3] = { rawPos[0] * slamScale, rawPos[1] * slamScale, rawPos[2] * slamScale };

	// Pre-update residuals for gating.
	vr::HmdVector3d_t pred0 = quaternionRotateVector(ekf.yawCorr, scaledRaw);
	const double r0[3] = {
		obsPos[0] - (pred0.v[0] + ekf.trans[0]),
		obsPos[1] - (pred0.v[1] + ekf.trans[1]),
		obsPos[2] - (pred0.v[2] + ekf.trans[2])
	};
	const double r0sq = r0[0] * r0[0] + r0[1] * r0[1] + r0[2] * r0[2];

	vr::HmdQuaternion_t qPred = quaternionNormalize(ekf.yawCorr * rawRot);
	vr::HmdQuaternion_t qDelta = quaternionProjectYaw(
		quaternionNormalize(obsRot * quaternionConjugate(qPred)));
	double rTheta = 2.0 * atan2(qDelta.y, qDelta.w);
	if (rTheta > 3.14159265) rTheta -= 2.0 * 3.14159265358979;
	if (rTheta < -3.14159265) rTheta += 2.0 * 3.14159265358979;

	const double Rt = Rt0 + (kv * linSpeed) * (kv * linSpeed);
	const double Rtheta = Rtheta0 + (kw * angSpeed) * (kw * angSpeed);
	const double St = ekf.Pt + Rt;
	const double Stheta = ekf.Ptheta + Rtheta;

	// Innovation gate (translation drives outlier bookkeeping).
	if (ekf.valid && (r0sq / St) > gateT)
	{
		bool reset = false;
		if (++ekf.outlierRun >= 5)
		{
			// Persistent disagreement (missed step / recenter fade): reopen the
			// covariance so the next accepted samples re-anchor in a few frames.
			LOG("FUSION EKF covariance reset (residual %.3fm after %d gated frames)",
				sqrt(r0sq), ekf.outlierRun);
			ekf.Pt = 1.0;
			ekf.Ptheta = 0.5;
			ekf.outlierRun = 0;
			reset = true;
		}
		// Diagnostic: gated frames are rare and important — always log.
		if (DiagCsvOpen())
		{
			LARGE_INTEGER now{}, freq{};
			QueryPerformanceCounter(&now);
			QueryPerformanceFrequency(&freq);
			double tms = (now.QuadPart - diagStart.QuadPart) * 1000.0 / (double)freq.QuadPart;
			LogDiagCsv("%.1f,%.3f,%.3f,%.2f,%.4f,%.3f,1,%s",
				tms, linSpeed, angSpeed, sqrt(r0sq) * 100.0, 0.0, sqrt(ekf.Pt) * 100.0,
				reset ? "reset" : "gate");
		}
		return false;
	}
	ekf.outlierRun = 0;

	// Yaw update first (bootstrap: translation must snap with the new yaw).
	if (!ekf.valid || (rTheta * rTheta / Stheta) <= gateTheta)
	{
		const double Ktheta = ekf.Ptheta / Stheta;
		ekf.yawCorr = quaternionProjectYaw(quaternionNormalize(
			yawQuaternion(Ktheta * rTheta) * ekf.yawCorr));
		ekf.Ptheta *= (1.0 - Ktheta);
	}

	// Translation residual with the updated yaw, then update.
	vr::HmdVector3d_t pred1 = quaternionRotateVector(ekf.yawCorr, scaledRaw);
	const double r1[3] = {
		obsPos[0] - (pred1.v[0] + ekf.trans[0]),
		obsPos[1] - (pred1.v[1] + ekf.trans[1]),
		obsPos[2] - (pred1.v[2] + ekf.trans[2])
	};
	const double Kt = ekf.Pt / St;
	ekf.trans[0] += Kt * r1[0];
	ekf.trans[1] += Kt * r1[1];
	ekf.trans[2] += Kt * r1[2];
	ekf.Pt *= (1.0 - Kt);

	ekf.valid = true;

	// Disparity stats for the heartbeat.
	const double rCm = sqrt(r1[0] * r1[0] + r1[1] * r1[1] + r1[2] * r1[2]) * 100.0;
	{
		diag.dispSumCm += rCm;
		if (rCm > diag.dispMaxCm)
			diag.dispMaxCm = rCm;
		++diag.dispN;
	}

	// Diagnostic: accepted frames at <=20 Hz. Plot disp_cm vs speed_mps colored by
	// Kt — if Kt collapses at high speed while disp stays low, the filter is coasting
	// on SLAM (not proof of alignment); if Kt holds and disp stays low, it's real.
	if (DiagCsvOpen())
	{
		LARGE_INTEGER now{}, freq{};
		QueryPerformanceCounter(&now);
		QueryPerformanceFrequency(&freq);
		double sinceWrite = diagLastWrite.QuadPart
			? (now.QuadPart - diagLastWrite.QuadPart) / (double)freq.QuadPart : 1.0;
		if (sinceWrite >= 0.05)
		{
			double tms = (now.QuadPart - diagStart.QuadPart) * 1000.0 / (double)freq.QuadPart;
			LogDiagCsv("%.1f,%.3f,%.3f,%.2f,%.4f,%.3f,0,",
				tms, linSpeed, angSpeed, rCm, Kt, sqrt(ekf.Pt) * 100.0);
			diagLastWrite = now;
		}
	}

	// Publish into the shared correction container (ApplyDrift / slamSync / fallback).
	drift.rotation = ekf.yawCorr;
	drift.translation.v[0] = ekf.trans[0];
	drift.translation.v[1] = ekf.trans[1];
	drift.translation.v[2] = ekf.trans[2];
	drift.valid = true;
	return true;
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

	// Disparity stats for the heartbeat (unblended residual vs current correction).
	if (drift.valid)
	{
		const double rx = instTrans.v[0] - drift.translation.v[0];
		const double ry = instTrans.v[1] - drift.translation.v[1];
		const double rz = instTrans.v[2] - drift.translation.v[2];
		const double rCm = sqrt(rx * rx + ry * ry + rz * rz) * 100.0;
		diag.dispSumCm += rCm;
		if (rCm > diag.dispMaxCm)
			diag.dispMaxCm = rCm;
		++diag.dispN;
	}

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

	// Never cache the published quash parking spot (~Y+9001). Raw is cached before quash;
	// this is a belt if call order ever changes.
	if (fabs(pos[1]) > 100.0)
		return;

	vr::HmdVector3d_t worldVel = quaternionRotateVector(pose.qWorldFromDriverRotation, pose.vecVelocity);
	vr::HmdVector3d_t worldAng = quaternionRotateVector(
		quaternionNormalize(pose.qWorldFromDriverRotation * pose.qRotation),
		pose.vecAngularVelocity);

	// V2-a: build the sample fully, then publish under the cache lock — the HMD pose
	// thread reads this concurrently with the tracker pose thread writing it.
	CachedTrackerPose next;
	next.rotation = worldRot;
	next.position[0] = pos[0];
	next.position[1] = pos[1];
	next.position[2] = pos[2];
	next.velocity[0] = worldVel.v[0];
	next.velocity[1] = worldVel.v[1];
	next.velocity[2] = worldVel.v[2];
	next.angularVelocity[0] = worldAng.v[0];
	next.angularVelocity[1] = worldAng.v[1];
	next.angularVelocity[2] = worldAng.v[2];
	QueryPerformanceCounter(&next.timestamp);
	next.valid = true;

	std::lock_guard<std::mutex> lock(trackerCacheMutex);
	cachedTracker = next;
}

bool ServerTrackedDeviceProvider::FetchTrackerSample(
	vr::HmdQuaternion_t& outRot, double outPos[3],
	double outVel[3], double outAngVel[3], double& outLinSpeed)
{
	LARGE_INTEGER now{}, freq{};
	QueryPerformanceCounter(&now);
	QueryPerformanceFrequency(&freq);

	// Hook cache only while override is on.
	// We quash the published head-tracker pose (~Y+9001) so Standable/VRC ignore it.
	// GetRawTrackedDevicePoses returns that published pose — never use it for HMD rebuild
	// (session_20260717_231753: HMD_JUMP_HOLD dist~9000m speed=0 from poisoned GetRaw).
	// Max age 150 ms (aligned with P1-a last-good hold); position dead-reckoned by age.
	const double kCacheMaxAgeSec = 0.15;
	const double maxAcceptLinSpeed = 8.0;

	outLinSpeed = 0.0;

	// V2-a: tracker cache is written on the tracker device's pose thread; copy under lock.
	CachedTrackerPose cache;
	{
		std::lock_guard<std::mutex> lock(trackerCacheMutex);
		cache = cachedTracker;
	}

	if (!cache.valid)
		return false;

	double age = (now.QuadPart - cache.timestamp.QuadPart) / (double)freq.QuadPart;
	if (age < 0.0 || age > kCacheMaxAgeSec)
		return false;

	outRot = cache.rotation;
	for (int i = 0; i < 3; i++)
	{
		// V0-b: dead-reckon continuously from the sample timestamp — no step at a
		// freshness boundary; predictionTime applies on top later.
		outPos[i] = cache.position[i] + cache.velocity[i] * age;
		outVel[i] = cache.velocity[i];
		outAngVel[i] = cache.angularVelocity[i];
	}
	outLinSpeed = sqrt(outVel[0] * outVel[0] + outVel[1] * outVel[1] + outVel[2] * outVel[2]);

	// V0-a: keep the measured speed on rejection so speed_rej diags stay truthful.
	if (outLinSpeed > maxAcceptLinSpeed
		|| !IsFiniteQuat(outRot)
		|| !IsFiniteVec3(outPos)
		|| fabs(outPos[1]) > 100.0)
	{
		return false;
	}

	return true;
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

bool ServerTrackedDeviceProvider::GatePublish(vr::DriverPose_t& pose, double displayHz, double linSpeed)
{
	// P0-b: non-finite → hold last good or drop.
	const double candPos[3] = { pose.vecPosition[0], pose.vecPosition[1], pose.vecPosition[2] };
	if (!IsFiniteQuat(pose.qRotation) || !IsFiniteVec3(candPos))
	{
		++diag.nonFiniteDrops;
		if (ApplyLastGoodHmd(pose, 0.15))
		{
			++diag.lastGoodHolds;
			return false;
		}
		pose.poseIsValid = false;
		pose.result = vr::TrackingResult_Running_OutOfRange;
		return false;
	}

	// P0-a: pre-publish jump gate — do not emit teleports.
	double dtPos = 1.0 / displayHz;
	if (diag.haveLastHmdPos && diag.lastHmdPosTime.QuadPart != 0)
	{
		LARGE_INTEGER now{}, freq{};
		QueryPerformanceCounter(&now);
		QueryPerformanceFrequency(&freq);
		double dt = (now.QuadPart - diag.lastHmdPosTime.QuadPart) / (double)freq.QuadPart;
		// V0-c: real elapsed time since the last accepted frame, so recovery after a
		// hold/BAD gap doesn't divide real motion by one frame. Floored at half a
		// display frame: back-to-back pose callbacks otherwise divide a harmless
		// millimetre-scale step by a sub-millisecond gap and manufacture an
		// impossible speed (observed: 1.5cm over 1.2ms read as 12.5 m/s).
		const double dtFloor = 0.5 / displayHz;
		if (dt > 1e-4)
			dtPos = dt < 0.3 ? (dt > dtFloor ? dt : dtFloor) : 0.3;
	}

	// V1: bounded reconvergence — a far candidate is approached at
	// <= kMaxCatchupSpeed instead of freeze-then-snap after a 250 ms hold.
	// Invariant: the published pose never moves more than kMaxCatchupSpeed*dt
	// per frame. Transient spikes cost at most one bounded step each way.
	const double kMaxCatchupSpeed = 3.0;   // m/s
	const double kReconvergeRotTau = 0.10; // s
	if (lastGoodHmd.valid && (reconverging || ShouldHoldForJump(candPos, dtPos)))
	{
		const double dx = candPos[0] - lastGoodHmd.position[0];
		const double dy = candPos[1] - lastGoodHmd.position[1];
		const double dz = candPos[2] - lastGoodHmd.position[2];
		const double dist = sqrt(dx * dx + dy * dy + dz * dz);
		const double maxStep = kMaxCatchupSpeed * dtPos;

		if (dist > maxStep)
		{
			if (!reconverging)
				++diag.jumpEvents;
			++diag.jumpHolds;
			reconverging = true;

			const double s = maxStep / dist;
			pose.vecPosition[0] = lastGoodHmd.position[0] + dx * s;
			pose.vecPosition[1] = lastGoodHmd.position[1] + dy * s;
			pose.vecPosition[2] = lastGoodHmd.position[2] + dz * s;
			pose.qRotation = NlerpQuat(lastGoodHmd.rotation, pose.qRotation,
				dtPos / (dtPos + kReconvergeRotTau));
			for (int i = 0; i < 3; i++)
			{
				pose.vecVelocity[i] = (pose.vecPosition[i] - lastGoodHmd.position[i]) / dtPos;
				pose.vecAngularVelocity[i] = 0.0;
			}

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
				LOG("HMD_JUMP_HOLD dist=%.3fm dt=%.4f speed=%.2f (reconverging <=%.1fm/s)",
					dist, dtPos, linSpeed, kMaxCatchupSpeed);
				diag.lastJumpLog = now;
			}
		}
		else
		{
			// Candidate within one bounded step — accept it as-is; slew done.
			if (reconverging)
				LOG("HMD reconverge complete (residual %.3fm)", dist);
			reconverging = false;
		}
	}
	else
	{
		reconverging = false;
	}

	StoreLastGoodHmd(pose);
	diag.lastHmdPos[0] = pose.vecPosition[0];
	diag.lastHmdPos[1] = pose.vecPosition[1];
	diag.lastHmdPos[2] = pose.vecPosition[2];
	QueryPerformanceCounter(&diag.lastHmdPosTime);
	diag.haveLastHmdPos = true;
	return true;
}

bool ServerTrackedDeviceProvider::HandleDevicePoseUpdated(uint32_t openVRID, vr::DriverPose_t& pose)
{
	if (openVRID >= vr::k_unMaxTrackedDeviceCount)
		return true;

	// V2-b: pose threads hold configMutex shared for the whole callback; IPC setters
	// take it exclusive, so config writes and filter resets can't tear an in-flight
	// pose. Pose threads never contend with each other on a shared lock.
	std::shared_lock<std::shared_mutex> configLock(configMutex);

	// Head-mounted lighthouse tracker while override is active:
	// 1) Cache raw pose for HMD rebuild (P1-b).
	// 2) Quash the *published* tracker pose so Standable / VRC / FBT IK ignore it.
	//    Same technique as OpenVR-SpaceCalibrator continuous-cal "Hide tracker"
	//    (park ~9001 m above origin). Device still enumerates; apps won't use it as a body joint.
	if (hmdTracker.enabled
		&& hmdTracker.trackerID < vr::k_unMaxTrackedDeviceCount
		&& openVRID == hmdTracker.trackerID)
	{
		CacheTrackerWorldPose(pose);

		// Cancel world-from-driver so final world pos ≈ (0, 9001, 0) regardless of seated zero.
		pose.vecPosition[0] = -pose.vecWorldFromDriverTranslation[0];
		pose.vecPosition[1] = -pose.vecWorldFromDriverTranslation[1] + 9001.0;
		pose.vecPosition[2] = -pose.vecWorldFromDriverTranslation[2];
		for (int i = 0; i < 3; i++)
		{
			pose.vecVelocity[i] = 0;
			pose.vecAngularVelocity[i] = 0;
		}
		// Still "valid" so SteamVR doesn't thrash connection; position is useless for IK.
		pose.poseIsValid = true;
		pose.deviceIsConnected = true;
		pose.result = vr::TrackingResult_Running_OK;
		return true;
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
				diag.slamSteps = 0;
				diag.dispSumCm = 0;
				diag.dispMaxCm = 0;
				diag.dispN = 0;
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
					LOG("heartbeat frames=%llu ok=%llu bad=%llu jumps=%llu holds=%llu lg_holds=%llu speed_rej=%llu fallback=%llu nonfinite=%llu enabled=%d mode=%c disp_avg=%.1fcm disp_max=%.1fcm slam_steps=%llu sig=%.2fcm",
						(unsigned long long)diag.frames,
						(unsigned long long)diag.trackerOkFrames,
						(unsigned long long)diag.trackerBadFrames,
						(unsigned long long)diag.jumpEvents,
						(unsigned long long)diag.jumpHolds,
						(unsigned long long)diag.lastGoodHolds,
						(unsigned long long)diag.speedRejects,
						(unsigned long long)diag.fallbackFrames,
						(unsigned long long)diag.nonFiniteDrops,
						hmdTracker.enabled ? 1 : 0,
						fusionMode ? 'F' : 'O',
						diag.dispN ? diag.dispSumCm / (double)diag.dispN : 0.0,
						diag.dispMaxCm,
						(unsigned long long)diag.slamSteps,
						sqrt(ekf.Pt) * 100.0);
					diag.dispSumCm = 0;
					diag.dispMaxCm = 0;
					diag.dispN = 0;
					diag.lastHeartbeat = now;
				}
			}

			// FUSION: SLAM is the source; the tracker only observes the SLAM→lighthouse
			// correction. LOS loss is a non-event (correction freezes). SLAM relocation
			// steps are attributed against the tracker and cancelled in the same frame.
			// Falls through to the classic override path when SLAM itself is invalid.
			if (fusionMode && !hmdTracker.native && rawValid)
			{
				if (trackerPoseOk)
				{
					// Observed head pose from the tracker (same math as override, not published).
					vr::HmdQuaternion_t trackerRef = quaternionNormalize(hmdTracker.calibrationRotation * trackerQuat);
					vr::HmdVector3d_t filteredTrackerPos = trackerFilter.translation.update({
						rawTrackerPos[0], rawTrackerPos[1], rawTrackerPos[2] });
					vr::HmdVector3d_t refPos = quaternionRotateVector(hmdTracker.calibrationRotation, filteredTrackerPos.v);
					refPos.v[0] += hmdTracker.calibrationTranslation.v[0];
					refPos.v[1] += hmdTracker.calibrationTranslation.v[1];
					refPos.v[2] += hmdTracker.calibrationTranslation.v[2];
					vr::HmdQuaternion_t obsRot = quaternionNormalize(trackerRef * hmdTracker.offsetRotation);
					vr::HmdVector3d_t off = quaternionRotateVector(trackerRef, hmdTracker.offsetTranslation.v);
					double obsPos[3] = {
						refPos.v[0] + off.v[0],
						refPos.v[1] + off.v[1],
						refPos.v[2] + off.v[2]
					};

					// Stale prev samples (mode fallback / long gap) must not fake a step.
					{
						LARGE_INTEGER now{}, freq{};
						QueryPerformanceCounter(&now);
						QueryPerformanceFrequency(&freq);
						if (fusion.havePrev)
						{
							double age = (now.QuadPart - fusion.lastFrame.QuadPart) / (double)freq.QuadPart;
							if (age > 0.2)
								fusion.havePrev = false;
						}
						fusion.lastFrame = now;
					}

					// Step attribution: a jump in SLAM the tracker didn't see is a SLAM
					// relocation → cancel it; a jump in the tracker SLAM didn't see is a
					// tracker glitch → don't let it steer the correction.
					bool skipUpdate = false;
					if (fusion.havePrev)
					{
						const double dsx = rawPosition[0] - fusion.prevRawPos[0];
						const double dsy = rawPosition[1] - fusion.prevRawPos[1];
						const double dsz = rawPosition[2] - fusion.prevRawPos[2];
						const double slamStep = sqrt(dsx * dsx + dsy * dsy + dsz * dsz);
						const double dox = obsPos[0] - fusion.prevObsPos[0];
						const double doy = obsPos[1] - fusion.prevObsPos[1];
						const double doz = obsPos[2] - fusion.prevObsPos[2];
						const double obsStep = sqrt(dox * dox + doy * doy + doz * doz);

						// Arm the canceller only when the tracker says the head is quiet:
						// that's the only regime where a SLAM jump is unambiguously a
						// relocation. During fast motion, latency skew makes SLAM lead the
						// tracker by centimeters per frame (session_20260723: cancels fired
						// with tracker step ~1.9cm during dancing — real motion eaten).
						// An uncancelled 2-3cm SLAM hiccup mid-motion is masked anyway.
						const bool headQuiet = linSpeed < 0.75;
						// Cooldown: a real relocation is one event; repeated cancels within
						// 250 ms indicate misattribution/oscillation (02:47:24 burst) and
						// also throttles log I/O on the pose thread.
						bool cancelReady = true;
						{
							LARGE_INTEGER nowC{}, freqC{};
							QueryPerformanceCounter(&nowC);
							QueryPerformanceFrequency(&freqC);
							if (fusion.lastCancel.QuadPart != 0)
							{
								double since = (nowC.QuadPart - fusion.lastCancel.QuadPart) / (double)freqC.QuadPart;
								if (since < 0.25)
									cancelReady = false;
							}
						}
						if (drift.valid && cancelReady && headQuiet && slamStep > 0.025 && obsStep < 0.01)
						{
							// Keep the published pose fixed while SLAM steps under it.
							const double slamScale = SlamToCorrectedScale();
							const double scaled[3] = { dsx * slamScale, dsy * slamScale, dsz * slamScale };
							vr::HmdVector3d_t comp = quaternionRotateVector(drift.rotation, scaled);
							for (int i = 0; i < 3; i++)
							{
								ekf.trans[i] -= comp.v[i];
								drift.translation.v[i] -= comp.v[i];
							}
							++diag.slamSteps;
							skipUpdate = true; // mid-step disparity is garbage; resume next frame
							QueryPerformanceCounter(&fusion.lastCancel);
							LOG("SLAM_STEP cancelled |d|=%.3fm (tracker step %.3fm)", slamStep, obsStep);
						}
						else if (drift.valid && obsStep > 0.025 && slamStep < 0.01)
						{
							skipUpdate = true; // tracker glitch (SLAM didn't see the jump)
						}
					}
					fusion.prevRawPos[0] = rawPosition[0];
					fusion.prevRawPos[1] = rawPosition[1];
					fusion.prevRawPos[2] = rawPosition[2];
					fusion.prevObsPos[0] = obsPos[0];
					fusion.prevObsPos[1] = obsPos[1];
					fusion.prevObsPos[2] = obsPos[2];
					fusion.havePrev = true;

					if (!skipUpdate)
					{
						const double angSpeed = sqrt(
							trackerAngVel[0] * trackerAngVel[0] +
							trackerAngVel[1] * trackerAngVel[1] +
							trackerAngVel[2] * trackerAngVel[2]);
						// Error-state KF: Mahalanobis gating replaces fixed thresholds,
						// covariance reset replaces the re-anchor; latency skew is absorbed
						// into motion-inflated R.
						FusionEkfUpdate(obsRot, obsPos, rawRotation, rawPosition,
							linSpeed, angSpeed);
					}
				}
				else
				{
					// Tracker BAD in fusion: a non-event — SLAM carries, correction frozen.
					fusion.havePrev = false;
				}

				if (drift.valid)
					ApplyDrift(pose);

				GatePublish(pose, displayHz, linSpeed);
				return true;
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

				if (!GatePublish(pose, displayHz, linSpeed))
					return true;

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