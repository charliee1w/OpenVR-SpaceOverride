// SPDX-License-Identifier: AGPL-3.0-only

#include "PoseEstimator.h"
#include "Logging.h"

#include <cmath>

using namespace pose_math;

namespace pose_est {
namespace {

void StoreLastGoodHmd(PoseState& state, const PoseClock& clock,
	const vr::HmdQuaternion_t& worldRot, const double worldPos[3],
	const double velocity[3], const double angularVelocity[3])
{
	state.lastGoodHmd.rotation = worldRot;
	for (int i = 0; i < 3; i++)
	{
		state.lastGoodHmd.position[i] = worldPos[i];
		state.lastGoodHmd.velocity[i] = velocity[i];
		state.lastGoodHmd.angularVelocity[i] = angularVelocity[i];
	}
	state.lastGoodHmd.timestamp = clock.now;
	state.lastGoodHmd.valid = true;
}

bool ApplyLastGoodHmd(vr::DriverPose_t& pose, const PoseState& state, const PoseClock& clock, double maxAgeSec)
{
	if (!state.lastGoodHmd.valid)
		return false;

	double age = clock.AgeSec(state.lastGoodHmd.timestamp);
	if (age < 0.0 || age > maxAgeSec)
		return false;

	WriteWorldPose(pose, state.lastGoodHmd.rotation, state.lastGoodHmd.position);
	for (int i = 0; i < 3; i++)
	{
		pose.vecVelocity[i] = state.lastGoodHmd.velocity[i];
		pose.vecAngularVelocity[i] = state.lastGoodHmd.angularVelocity[i];
	}
	pose.poseIsValid = true;
	pose.deviceIsConnected = true;
	pose.result = vr::TrackingResult_Running_OK;
	pose.shouldApplyHeadModel = false;
	pose.poseTimeOffset = 0;
	return true;
}

bool ShouldHoldForJump(const PoseState& state, const double newPos[3], double dtSec)
{
	if (!state.lastGoodHmd.valid)
		return false;
	if (!IsFiniteVec3(newPos))
		return true;

	const double dx = newPos[0] - state.lastGoodHmd.position[0];
	const double dy = newPos[1] - state.lastGoodHmd.position[1];
	const double dz = newPos[2] - state.lastGoodHmd.position[2];
	const double dist = sqrt(dx * dx + dy * dy + dz * dz);

	const double jumpThreshM = 0.35;
	if (dist > jumpThreshM)
		return true;
	if (dtSec > 1e-4 && (dist / dtSec) > kMaxPlausibleHeadSpeed)
		return true;
	return false;
}

// Returns false when the pose was replaced by a last-good hold / slew (caller
// skips drift bookkeeping for this frame).
bool GatePublish(vr::DriverPose_t& pose, const PoseConfig& cfg, double displayHz,
	double linSpeed, PoseState& state, PoseDiag& diag, const PoseClock& clock)
{
	vr::HmdQuaternion_t candRot;
	double candPos[3];
	ComposeWorldPose(pose, candRot, candPos);

	if (!IsFiniteQuat(candRot) || !IsFiniteVec3(candPos))
	{
		++diag.nonFiniteDrops;
		if (ApplyLastGoodHmd(pose, state, clock, 0.15))
		{
			++diag.lastGoodHolds;
			return false;
		}
		pose.poseIsValid = false;
		pose.result = vr::TrackingResult_Running_OutOfRange;
		return false;
	}

	double dtPos = 1.0 / displayHz;
	if (diag.haveLastHmdPos && diag.lastHmdPosTime != 0)
	{
		double dt = clock.AgeSec(diag.lastHmdPosTime);
		const double dtFloor = 0.5 / displayHz;
		if (dt > 1e-4)
			dtPos = dt < 0.3 ? (dt > dtFloor ? dt : dtFloor) : 0.3;
	}

	const double dtSlew = dtPos < 2.0 / displayHz ? dtPos : 2.0 / displayHz;

	const double kMaxCatchupSpeed = 3.0;
	const double kMaxCatchupAngSpeed = 3.0;
	const double kReconvergeRotTau = 0.10;
	// cfg.publishSlewEnabled off means a distant candidate is published as-is rather than
	// approached. That is a snap, i.e. exactly the teleport this gate exists to prevent -- it is
	// a safety bound, not smoothing, and it costs nothing in steady state because it only
	// engages while reconverging or holding for a jump.
	if (cfg.publishSlewEnabled && state.lastGoodHmd.valid
		&& (state.reconverging || ShouldHoldForJump(state, candPos, dtPos)))
	{
		const double dx = candPos[0] - state.lastGoodHmd.position[0];
		const double dy = candPos[1] - state.lastGoodHmd.position[1];
		const double dz = candPos[2] - state.lastGoodHmd.position[2];
		const double dist = sqrt(dx * dx + dy * dy + dz * dz);
		const double maxStep = kMaxCatchupSpeed * dtSlew;

		if (dist > maxStep)
		{
			if (!state.reconverging)
				++diag.jumpEvents;
			++diag.jumpHolds;
			state.reconverging = true;

			const double s = maxStep / dist;
			const double stepPos[3] = {
				state.lastGoodHmd.position[0] + dx * s,
				state.lastGoodHmd.position[1] + dy * s,
				state.lastGoodHmd.position[2] + dz * s
			};
			double tRot = dtSlew / (dtSlew + kReconvergeRotTau);
			{
				double dot = state.lastGoodHmd.rotation.w * candRot.w + state.lastGoodHmd.rotation.x * candRot.x
					+ state.lastGoodHmd.rotation.y * candRot.y + state.lastGoodHmd.rotation.z * candRot.z;
				dot = fabs(dot);
				if (dot > 1.0)
					dot = 1.0;
				const double angle = 2.0 * acos(dot);
				const double maxAngStep = kMaxCatchupAngSpeed * dtSlew;
				if (angle > 1e-9 && angle * tRot > maxAngStep)
					tRot = maxAngStep / angle;
			}
			const vr::HmdQuaternion_t stepRot = NlerpQuat(state.lastGoodHmd.rotation, candRot, tRot);

			WriteWorldPose(pose, stepRot, stepPos);
			const double dtVel = dtSlew > 1e-6 ? dtSlew : dtPos;
			double stepVel[3];
			for (int i = 0; i < 3; i++)
			{
				stepVel[i] = (stepPos[i] - state.lastGoodHmd.position[i]) / dtVel;
				pose.vecVelocity[i] = stepVel[i];
				pose.vecAngularVelocity[i] = 0.0;
			}
			SanitizeVec3Velocity(pose.vecVelocity, kMaxCatchupSpeed * 1.5);
			SanitizeVec3Velocity(pose.vecAngularVelocity, kMaxCatchupAngSpeed * 1.5);

			bool allow = true;
			if (diag.lastJumpLog != 0)
			{
				double since = clock.AgeSec(diag.lastJumpLog);
				if (since < 0.5)
					allow = false;
			}
			if (allow)
			{
				LOG("HMD_JUMP_HOLD dist=%.3fm dt=%.4f speed=%.2f (reconverging <=%.1fm/s)",
					dist, dtPos, linSpeed, kMaxCatchupSpeed);
				diag.lastJumpLog = clock.now;
			}

			StoreLastGoodHmd(state, clock, stepRot, stepPos, stepVel, pose.vecAngularVelocity);
			for (int i = 0; i < 3; i++)
				diag.lastHmdPos[i] = stepPos[i];
			diag.lastHmdPosTime = clock.now;
			diag.haveLastHmdPos = true;
			return false;
		}
		else
		{
			if (state.reconverging)
				LOG("HMD reconverge complete (residual %.3fm)", dist);
			state.reconverging = false;
		}
	}
	else
	{
		state.reconverging = false;
	}

	SanitizeVec3Velocity(pose.vecVelocity, kMaxPlausibleHeadSpeed);
	SanitizeVec3Velocity(pose.vecAngularVelocity, 20.0);
	vr::HmdVector3d_t worldVel = quaternionRotateVector(pose.qWorldFromDriverRotation, pose.vecVelocity);
	vr::HmdVector3d_t worldAngVel = quaternionRotateVector(pose.qWorldFromDriverRotation, pose.vecAngularVelocity);
	StoreLastGoodHmd(state, clock, candRot, candPos, worldVel.v, worldAngVel.v);
	for (int i = 0; i < 3; i++)
		diag.lastHmdPos[i] = candPos[i];
	diag.lastHmdPosTime = clock.now;
	diag.haveLastHmdPos = true;
	return true;
}

bool FusionEkfUpdate(PoseState& state, PoseDiag& diag, const PoseConfig& cfg, const PoseClock& clock,
	const vr::HmdQuaternion_t& obsRot, const double obsPos[3],
	const vr::HmdQuaternion_t& rawRot, const double rawPos[3],
	double linSpeed, double angSpeed, double measAgeSec,
	vr::ETrackingResult trackerResult)
{
	const double Qtheta = 3e-6;
	const double Qt = 4e-6;
	const double Rtheta0 = 7.6e-5;
	const double Rt0 = 1.6e-5;
	const double kv = 0.008;
	const double kw = 0.010;
	const double sigmaV = 0.30;
	const double aMax = 20.0;
	const double sigmaW = 0.50;
	const double gateT = 16.0;
	const double gateTheta = 9.0;

	auto& ekf = state.ekf;
	double dt = FilterStep(ekf.lastUpdate, ekf.valid, clock.now, clock.freq);
	if (ekf.valid)
	{
		ekf.Ptheta += Qtheta * dt;
		ekf.Pt += Qt * dt;
	}

	const double slamScale = cfg.SlamScale();
	const double scaledRaw[3] = { rawPos[0] * slamScale, rawPos[1] * slamScale, rawPos[2] * slamScale };

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
	if (rTheta > 3.14159265)
		rTheta -= 2.0 * 3.14159265358979;
	if (rTheta < -3.14159265)
		rTheta += 2.0 * 3.14159265358979;

	const double age = measAgeSec > 0.0 ? measAgeSec : 0.0;
	const double agePos = (sigmaV * age) * (sigmaV * age)
		+ (0.5 * aMax * age * age) * (0.5 * aMax * age * age);
	const double ageAng = (sigmaW * age) * (sigmaW * age);
	const double quality = (trackerResult == vr::TrackingResult_Running_OK) ? 1.0 : 25.0;
	const double Rt = (Rt0 + (kv * linSpeed) * (kv * linSpeed) + agePos) * quality;
	const double Rtheta = (Rtheta0 + (kw * angSpeed) * (kw * angSpeed) + ageAng) * quality;
	const double St = ekf.Pt + Rt;
	const double Stheta = ekf.Ptheta + Rtheta;

	// The Mahalanobis gate FAILS OPEN on a non-finite innovation: NaN > gateT is false, so a
	// NaN observation was ACCEPTED, latched into ekf.trans/yawCorr, and stayed there for the
	// session — every isfinite check downstream guards the published pose, not the filter
	// state, so the filter never recovered. No such input has been observed on this rig (the
	// hook cache and prediction paths are finite-checked), so this is hardening against an
	// external garbage sample, not an active defect — but the latch is permanent, and the
	// cost of rejecting is one skipped update. Skip WITHOUT counting toward the outlier run:
	// five NaNs must not trigger a covariance reset that re-anchors from the sixth.
	if (!std::isfinite(r0sq) || !std::isfinite(rTheta) || !std::isfinite(St) || !std::isfinite(Stheta))
		return false;

	if (ekf.valid && (r0sq / St) > gateT)
	{
		bool reset = false;
		if (++ekf.outlierRun >= 5)
		{
			LOG("FUSION EKF covariance reset (residual %.3fm after %d gated frames)",
				sqrt(r0sq), ekf.outlierRun);
			ekf.Pt = 1.0;
			ekf.Ptheta = 0.5;
			ekf.outlierRun = 0;
			reset = true;
		}
		if (DiagCsvOpen())
		{
			double tms = state.diagStart
				? (clock.now - state.diagStart) * 1000.0 / (double)clock.freq
				: 0.0;
			double corrCm = sqrt(ekf.trans[0] * ekf.trans[0] + ekf.trans[1] * ekf.trans[1] + ekf.trans[2] * ekf.trans[2]) * 100.0;
			double corrYaw = 2.0 * atan2(ekf.yawCorr.y, ekf.yawCorr.w) * 180.0 / 3.14159265358979;
			LogDiagCsv("%.1f,F,%.3f,%.3f,%.2f,%.4f,%.3f,%.2f,%.2f,%.4f,%.4f,%.4f,1,%s",
				tms, linSpeed, angSpeed, sqrt(r0sq) * 100.0, 0.0, sqrt(ekf.Pt) * 100.0,
				corrCm, corrYaw, obsPos[0], obsPos[1], obsPos[2],
				reset ? "reset" : "gate");
		}
		return false;
	}
	ekf.outlierRun = 0;

	// N1-h: bound how fast the CORRECTION itself may move.
	//
	// Real head motion does not pass through the correction — the head's own rotation is in
	// `rawRot` and its own translation in `scaledRaw` — so a rate limit here cannot add latency
	// to genuine motion. That is why the bound lives here and NOT as a rotation limb in
	// GatePublish: the publish gate sees `yawCorr * rawRot` and cannot distinguish a 650 deg/s
	// artifact from a 650 deg/s head turn, so a limb there would either miss this or slew real
	// turns.
	//
	// What it bounds, measured in session_20260806_200502: five covariance resets in 7.8 s, each
	// re-anchoring yaw from a SINGLE sample, published as one-frame view-yaw steps of
	// 6.17 / 13.02 / 14.39 / 17.87 deg while the head moved 0.4-5.2 mm.
	//
	// Retuning the reset covariance cannot fix that. Rtheta0 = 7.6e-5 rad^2 is sigma = 0.5 deg,
	// so ANY reset Ptheta meaningfully above the measurement noise gives Ktheta ~ 1 — the
	// observed 0.99985. What is actually wrong is that Rtheta claims 0.5 deg while the
	// single-sample yaw estimates during that burst disagreed by 28 deg. Until the measurement
	// model is right, this is what makes the failure survivable rather than visible.
	//
	// Deliberately NOT applied to the slam-step cancellation below: that subtracts real SLAM
	// motion, and slewing it would reintroduce the jump it exists to remove.
	const double kMaxCorrYawRate = 0.26;    // rad/s, ~15 deg/s
	const double kMaxCorrTransRate = 0.50;  // m/s
	// Bypassable, but this is a SAFETY BOUND rather than smoothing: no real head motion passes
	// through the correction channel, so the limit adds no latency to anything the user does.
	// Turning it off restores the measured 17.87-degree single-frame view yaw step on covariance
	// reset (N1-h). Cold-start anchor is exempt either way.
	const bool limitSlew = cfg.corrRateLimitEnabled && ekf.valid && dt > 0.0;

	if (!ekf.valid || (rTheta * rTheta / Stheta) <= gateTheta)
	{
		const double Ktheta = ekf.Ptheta / Stheta;
		double dYaw = Ktheta * rTheta;
		// Covariance must contract by the gain actually APPLIED, not the gain the filter wanted.
		// Contracting by the full Ktheta while the clamp holds dYaw back leaves the filter
		// believing a correction it never made: after a reset (Ptheta=0.5, Ktheta~0.9998) the
		// first accepted frame would collapse Ptheta to ~1e-4 while the correction moved ~0.17 deg
		// of a ~20 deg residual — every following frame then fails the innovation gate, outlierRun
		// re-fires the reset every ~6 frames, and the re-anchor duty-cycles instead of slewing.
		double KthetaEff = Ktheta;
		if (limitSlew)
		{
			const double maxYaw = kMaxCorrYawRate * dt;
			if (dYaw > maxYaw)
			{
				dYaw = maxYaw;
				++diag.corrSlewLimited;
			}
			else if (dYaw < -maxYaw)
			{
				dYaw = -maxYaw;
				++diag.corrSlewLimited;
			}
			if (fabs(rTheta) > 1e-12)
				KthetaEff = dYaw / rTheta;   // == Ktheta when the clamp did not engage
		}
		ekf.yawCorr = quaternionProjectYaw(quaternionNormalize(
			yawQuaternion(dYaw) * ekf.yawCorr));
		ekf.Ptheta *= (1.0 - KthetaEff);
	}

	vr::HmdVector3d_t pred1 = quaternionRotateVector(ekf.yawCorr, scaledRaw);
	const double r1[3] = {
		obsPos[0] - (pred1.v[0] + ekf.trans[0]),
		obsPos[1] - (pred1.v[1] + ekf.trans[1]),
		obsPos[2] - (pred1.v[2] + ekf.trans[2])
	};
	const double Kt = ekf.Pt / St;
	double dTrans[3] = { Kt * r1[0], Kt * r1[1], Kt * r1[2] };
	// Same rule as the yaw limb: contract Pt by the effective gain (Kt scaled by the clamp
	// factor), so a clamped update leaves the covariance honest about the residual still owed.
	double KtEff = Kt;
	if (limitSlew)
	{
		const double maxStep = kMaxCorrTransRate * dt;
		const double mag = sqrt(dTrans[0] * dTrans[0] + dTrans[1] * dTrans[1] + dTrans[2] * dTrans[2]);
		if (mag > maxStep && mag > 1e-12)
		{
			const double s = maxStep / mag;
			dTrans[0] *= s;
			dTrans[1] *= s;
			dTrans[2] *= s;
			KtEff = Kt * s;
			++diag.corrSlewLimited;
		}
	}
	ekf.trans[0] += dTrans[0];
	ekf.trans[1] += dTrans[1];
	ekf.trans[2] += dTrans[2];
	ekf.Pt *= (1.0 - KtEff);

	ekf.valid = true;

	const double rCm = sqrt(r1[0] * r1[0] + r1[1] * r1[1] + r1[2] * r1[2]) * 100.0;
	{
		diag.dispSumCm += rCm;
		if (rCm > diag.dispMaxCm)
			diag.dispMaxCm = rCm;
		++diag.dispN;
	}

	if (DiagCsvOpen())
	{
		double sinceWrite = state.diagLastWrite
			? clock.AgeSec(state.diagLastWrite) : 1.0;
		if (sinceWrite >= 0.05)
		{
			double tms = state.diagStart
				? (clock.now - state.diagStart) * 1000.0 / (double)clock.freq
				: 0.0;
			double corrCm = sqrt(ekf.trans[0] * ekf.trans[0] + ekf.trans[1] * ekf.trans[1] + ekf.trans[2] * ekf.trans[2]) * 100.0;
			double corrYaw = 2.0 * atan2(ekf.yawCorr.y, ekf.yawCorr.w) * 180.0 / 3.14159265358979;
			LogDiagCsv("%.1f,F,%.3f,%.3f,%.2f,%.4f,%.3f,%.2f,%.2f,%.4f,%.4f,%.4f,0,",
				tms, linSpeed, angSpeed, rCm, Kt, sqrt(ekf.Pt) * 100.0,
				corrCm, corrYaw, obsPos[0], obsPos[1], obsPos[2]);
			state.diagLastWrite = clock.now;
		}
	}

	// No mutex: pure state write. Live provider copies to shared drift after the frame.
	state.drift.rotation = ekf.yawCorr;
	state.drift.translation.v[0] = ekf.trans[0];
	state.drift.translation.v[1] = ekf.trans[1];
	state.drift.translation.v[2] = ekf.trans[2];
	state.drift.valid = true;
	return true;
}

void UpdateDrift(PoseState& state, PoseDiag& diag, const PoseConfig& cfg, const PoseClock& clock,
	const vr::HmdQuaternion_t& correctedRotation, const double (&correctedPosition)[3],
	const vr::HmdQuaternion_t& rawRotation, const double (&rawPosition)[3], double weight)
{
	if (weight <= 0.0)
		return;
	if (weight > 1.0)
		weight = 1.0;

	vr::HmdQuaternion_t instRot = quaternionProjectYaw(quaternionNormalize(correctedRotation * quaternionConjugate(rawRotation)));
	vr::HmdVector3d_t instRotatedRaw = quaternionRotateVector(instRot, rawPosition);

	double slamScale = cfg.SlamScale();
	vr::HmdVector3d_t instTrans = {
		correctedPosition[0] - instRotatedRaw.v[0] * slamScale,
		correctedPosition[1] - instRotatedRaw.v[1] * slamScale,
		correctedPosition[2] - instRotatedRaw.v[2] * slamScale
	};

	if (state.drift.valid)
	{
		const double rx = instTrans.v[0] - state.drift.translation.v[0];
		const double ry = instTrans.v[1] - state.drift.translation.v[1];
		const double rz = instTrans.v[2] - state.drift.translation.v[2];
		const double rCm = sqrt(rx * rx + ry * ry + rz * rz) * 100.0;
		diag.dispSumCm += rCm;
		if (rCm > diag.dispMaxCm)
			diag.dispMaxCm = rCm;
		++diag.dispN;
	}

	if (weight < 1.0 && state.drift.valid)
	{
		instRot = NlerpQuat(state.drift.rotation, instRot, weight);
		instTrans.v[0] = state.drift.translation.v[0] + (instTrans.v[0] - state.drift.translation.v[0]) * weight;
		instTrans.v[1] = state.drift.translation.v[1] + (instTrans.v[1] - state.drift.translation.v[1]) * weight;
		instTrans.v[2] = state.drift.translation.v[2] + (instTrans.v[2] - state.drift.translation.v[2]) * weight;
	}

	double dt = FilterStep(state.drift.lastUpdate, state.drift.valid, clock.now, clock.freq);

	// One-Euro on the correction channel. Bypassable: this smooths the drift transform, not head
	// motion (the head's own rotation is in rawRot and its translation in scaledRaw), so turning
	// it off makes the correction track its observation more closely rather than making the view
	// laggier -- but it also lets observation noise straight into the published transform.
	vr::HmdQuaternion_t newRot = cfg.driftFilterEnabled
		? state.drift.rotationFilter.filter(instRot, dt)
		: instRot;
	vr::HmdVector3d_t newTrans = cfg.driftFilterEnabled
		? state.drift.translationFilter.filter(instTrans, dt)
		: instTrans;
	state.drift.rotation = newRot;
	state.drift.translation = newTrans;
	state.drift.valid = true;
}

} // namespace

bool ApplyDrift(vr::DriverPose_t& pose, const PoseConfig& cfg, const PoseState& state)
{
	if (!state.drift.valid)
		return false;

	const vr::HmdQuaternion_t driftRotation = state.drift.rotation;
	const vr::HmdVector3d_t driftTranslation = state.drift.translation;
	double slamScale = cfg.SlamScale();

	pose.qWorldFromDriverRotation = quaternionNormalize(driftRotation * pose.qWorldFromDriverRotation);

	pose.vecPosition[0] *= slamScale;
	pose.vecPosition[1] *= slamScale;
	pose.vecPosition[2] *= slamScale;

	pose.vecDriverFromHeadTranslation[0] *= slamScale;
	pose.vecDriverFromHeadTranslation[1] *= slamScale;
	pose.vecDriverFromHeadTranslation[2] *= slamScale;

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

bool ProcessHmdFrame(
	const PoseConfig& cfg,
	const PoseClock& clock,
	double displayHz,
	const HmdInput& hmd,
	const TrackerInput& tracker,
	vr::DriverPose_t& pose,
	PoseState& state,
	PoseDiag& diag)
{
	// Keep headFilter.enabled in sync with config snapshot for this frame.
	state.headFilter.enabled = cfg.headFilterEnabled;

	const bool rawValid = hmd.rawValid;
	const vr::HmdQuaternion_t& rawRotation = hmd.rotation;
	const double (&rawPosition)[3] = hmd.position;

	const bool trackerPoseOk = tracker.poseOk;
	const bool trackerOkForOverride = tracker.okForOverride;
	const bool speedReject = tracker.speedReject;
	double linSpeed = tracker.linSpeed;

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
		diag.slamStepSumM = 0;
		diag.corrSlewLimited = 0;
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

	// Rate-limited tracker state + heartbeat (uses injected clock).
	{
		if (!diag.trackerStateKnown || diag.lastTrackerOk != trackerPoseOk)
		{
			bool allowLog = true;
			if (diag.lastBadLog != 0)
			{
				double since = clock.AgeSec(diag.lastBadLog);
				if (since < 0.25)
					allowLog = false;
			}
			if (allowLog)
			{
				// cache age restores (and improves on) the `cache=` field the extraction
				// dropped: a fresh fetch reads ~0, a dead-reckoned sample reads its true age,
				// which is what actually matters when reading a BAD line.
				LOG("tracker %s speed=%.2f reject_speed=%d cache_age_ms=%.0f",
					trackerPoseOk ? "OK" : "BAD",
					linSpeed,
					speedReject ? 1 : 0,
					tracker.ageSec * 1000.0);
				diag.lastBadLog = clock.now;
			}
			diag.lastTrackerOk = trackerPoseOk;
			diag.trackerStateKnown = true;
		}

		if (diag.lastHeartbeat == 0)
			diag.lastHeartbeat = clock.now;
		double hb = clock.AgeSec(diag.lastHeartbeat);
		if (hb >= 60.0)
		{
			// `absorbed` is the running total of SLAM-origin motion folded into `corr`. Read the
			// two together: corr ≈ absorbed means the correction is explained by real steps and is
			// nominal however large it looks. Tracking health is disp/sig/jumps, never corr.
			LOG("heartbeat frames=%llu ok=%llu bad=%llu jumps=%llu holds=%llu lg_holds=%llu speed_rej=%llu fallback=%llu nonfinite=%llu enabled=1 mode=%c disp_avg=%.1fcm disp_max=%.1fcm slam_steps=%llu corr_slew=%llu sig=%.2fcm corr_yaw=%.2fdeg corr=%.1fcm absorbed=%.2fm",
				(unsigned long long)diag.frames,
				(unsigned long long)diag.trackerOkFrames,
				(unsigned long long)diag.trackerBadFrames,
				(unsigned long long)diag.jumpEvents,
				(unsigned long long)diag.jumpHolds,
				(unsigned long long)diag.lastGoodHolds,
				(unsigned long long)diag.speedRejects,
				(unsigned long long)diag.fallbackFrames,
				(unsigned long long)diag.nonFiniteDrops,
				// Gate the EKF fields on the same condition that decides whether the EKF
				// actually runs (fusionActive = fusionMode && !native), not on fusionMode
				// alone. With fusionMode=1 native=1 the filter is never touched, so the old
				// test printed mode=F sig=100.00cm — sqrt of the untouched default Pt=1.0 —
				// which reads as a wildly unconverged filter rather than an unused one. That
				// is the exact misreading the override-mode guard was added to prevent.
				(cfg.fusionMode && !cfg.native) ? 'F' : 'O',
				diag.dispN ? diag.dispSumCm / (double)diag.dispN : 0.0,
				diag.dispMaxCm,
				(unsigned long long)diag.slamSteps,
				(unsigned long long)diag.corrSlewLimited,
				(cfg.fusionMode && !cfg.native) ? sqrt(state.ekf.Pt) * 100.0 : 0.0,
				(cfg.fusionMode && !cfg.native) ? EkfYawCorrDeg(state) : 0.0,
				(cfg.fusionMode && !cfg.native) ? EkfTransMagCm(state) : 0.0,
				diag.slamStepSumM);
			diag.dispSumCm = 0;
			diag.dispMaxCm = 0;
			diag.dispN = 0;
			diag.lastHeartbeat = clock.now;
		}
	}

	const bool fusionActive = cfg.fusionMode && !cfg.native;
	// True when fusion could not use SLAM this frame and the tracker-driven rebuild below is
	// standing in for it. The rebuild is shared with real override mode, but three of its steps
	// must stay inert here — see the uses of this flag:
	//
	//   - velocity prediction, because it feeds the SAME state.trackerFilter.translation the
	//     fusion branch feeds unpredicted. Flipping the input convention frame-to-frame during
	//     a SLAM dropout injects a vel*predSec step (~1 cm at 1 m/s, one frame of prediction)
	//     into filter state, which then becomes the EKF's obsPos on the next fusion frame —
	//     where 4-sigma of the translation gate is only ~1.6 cm, so a dropout could push good
	//     samples into the innovation gate.
	//   - the One-Euro head filter, which the docs state is inert in fusion.
	//   - the headVel angular-velocity chain, which fusion frames never feed — running it here
	//     differentiates against whatever rotation the PREVIOUS dropout stored (see the reset at
	//     its use site).
	bool fusionFallthrough = false;
	if (fusionActive && !rawValid)
	{
		if (ApplyLastGoodHmd(pose, state, clock, 0.15))
		{
			++diag.lastGoodHolds;
			return true;
		}
		if (!trackerOkForOverride)
		{
			pose.poseIsValid = false;
			pose.result = vr::TrackingResult_Running_OutOfRange;
			return true;
		}
		// Fall through to the tracker-driven rebuild below.
		fusionFallthrough = true;
	}
	else if (fusionActive)
	{
		if (trackerPoseOk)
		{
			vr::HmdQuaternion_t trackerRef = quaternionNormalize(cfg.calibrationRotation * tracker.rotation);
			// Bypassed when the pre-filter is off. In fusion this feeds the EKF's observation
			// rather than the view, so the cost here is a coloured innovation (N4-e) rather than
			// visible lag -- but N4-e's step (1) wants it gone from this path too.
			const vr::HmdVector3d_t rawTrackerVec = {
				tracker.position[0], tracker.position[1], tracker.position[2] };
			vr::HmdVector3d_t filteredTrackerPos = cfg.trackerFilterEnabled
				? state.trackerFilter.translation.update(rawTrackerVec)
				: rawTrackerVec;
			vr::HmdVector3d_t refPos = quaternionRotateVector(cfg.calibrationRotation, filteredTrackerPos.v);
			refPos.v[0] += cfg.calibrationTranslation.v[0];
			refPos.v[1] += cfg.calibrationTranslation.v[1];
			refPos.v[2] += cfg.calibrationTranslation.v[2];
			vr::HmdQuaternion_t obsRot = quaternionNormalize(trackerRef * cfg.offsetRotation);
			vr::HmdVector3d_t off = quaternionRotateVector(trackerRef, cfg.offsetTranslation.v);
			double obsPos[3] = {
				refPos.v[0] + off.v[0],
				refPos.v[1] + off.v[1],
				refPos.v[2] + off.v[2]
			};

			if (state.fusion.havePrev)
			{
				double age = clock.AgeSec(state.fusion.lastFrame);
				if (age > 0.2)
					state.fusion.havePrev = false;
			}
			state.fusion.lastFrame = clock.now;

			bool skipUpdate = false;
			if (state.fusion.havePrev)
			{
				const double dsx = rawPosition[0] - state.fusion.prevRawPos[0];
				const double dsy = rawPosition[1] - state.fusion.prevRawPos[1];
				const double dsz = rawPosition[2] - state.fusion.prevRawPos[2];
				const double slamStep = sqrt(dsx * dsx + dsy * dsy + dsz * dsz);
				const double dox = obsPos[0] - state.fusion.prevObsPos[0];
				const double doy = obsPos[1] - state.fusion.prevObsPos[1];
				const double doz = obsPos[2] - state.fusion.prevObsPos[2];
				const double obsStep = sqrt(dox * dox + doy * doy + doz * doz);

				const bool headQuiet = linSpeed < 0.75;
				bool cancelReady = true;
				if (state.fusion.lastCancel != 0)
				{
					double since = clock.AgeSec(state.fusion.lastCancel);
					if (since < 0.25)
						cancelReady = false;
				}
				if (state.drift.valid && cancelReady && headQuiet && slamStep > 0.025 && obsStep < 0.01)
				{
					const double slamScale = cfg.SlamScale();
					const double scaled[3] = { dsx * slamScale, dsy * slamScale, dsz * slamScale };
					vr::HmdVector3d_t comp = quaternionRotateVector(state.drift.rotation, scaled);
					for (int i = 0; i < 3; i++)
					{
						state.ekf.trans[i] -= comp.v[i];
						state.drift.translation.v[i] -= comp.v[i];
					}
					const double kappa = 0.2;
					state.ekf.Pt += (kappa * slamStep) * (kappa * slamStep);
					++diag.slamSteps;
					diag.slamStepSumM += slamStep;
					skipUpdate = true;
					state.fusion.lastCancel = clock.now;
					LOG("SLAM_STEP cancelled |d|=%.3fm (tracker step %.3fm)", slamStep, obsStep);
				}
				else if (state.drift.valid && obsStep > 0.025 && slamStep < 0.01)
				{
					skipUpdate = true;
				}
			}
			state.fusion.prevRawPos[0] = rawPosition[0];
			state.fusion.prevRawPos[1] = rawPosition[1];
			state.fusion.prevRawPos[2] = rawPosition[2];
			state.fusion.prevObsPos[0] = obsPos[0];
			state.fusion.prevObsPos[1] = obsPos[1];
			state.fusion.prevObsPos[2] = obsPos[2];
			state.fusion.havePrev = true;

			if (!skipUpdate)
			{
				const double angSpeed = sqrt(
					tracker.angularVelocity[0] * tracker.angularVelocity[0] +
					tracker.angularVelocity[1] * tracker.angularVelocity[1] +
					tracker.angularVelocity[2] * tracker.angularVelocity[2]);
				FusionEkfUpdate(state, diag, cfg, clock, obsRot, obsPos, rawRotation, rawPosition,
					linSpeed, angSpeed, tracker.ageSec, tracker.result);
			}
		}
		else
		{
			state.fusion.havePrev = false;
			// Same reason as the override branch below: a stale pre-filter state would feed the
			// EKF an observation dragged toward the pre-dropout position, and the translation
			// innovation gate rejects at ~1.6 cm, so a few centimetres of movement during the gap
			// is enough to gate every frame on return and force a covariance reset.
			state.trackerFilter.reset();
		}

		ApplyDrift(pose, cfg, state);
		GatePublish(pose, cfg, displayHz, linSpeed, state, diag, clock);
		return true;
	}

	if (trackerOkForOverride)
	{
		vr::HmdQuaternion_t trackerQuat = tracker.rotation;
		double rawTrackerPos[3] = { tracker.position[0], tracker.position[1], tracker.position[2] };
		const double (&trackerVel)[3] = tracker.velocity;
		const double (&trackerAngVel)[3] = tracker.angularVelocity;

		vr::HmdQuaternion_t trackerRefRotation = quaternionNormalize(cfg.calibrationRotation * trackerQuat);

		float predFrames = cfg.predictionTime;
		if (predFrames < 0.0f)
			predFrames = 0.0f;
		if (predFrames > 10.0f)
			predFrames = 10.0f;
		const double predSec = (1.0 / displayHz) * (double)predFrames;
		if (predSec > 0.0 && !fusionFallthrough)
		{
			rawTrackerPos[0] += trackerVel[0] * predSec;
			rawTrackerPos[1] += trackerVel[1] * predSec;
			rawTrackerPos[2] += trackerVel[2] * predSec;
		}

		// OVERRIDE PATH: this filtered position becomes the published HEAD position, so the
		// filter's lag is view lag. With the shipped constants that is ~125 ms below a 6.3 mm
		// per-frame dead-band, which is where ordinary nodding lives. Off by default.
		const vr::HmdVector3d_t rawTrackerVec = {
			rawTrackerPos[0],
			rawTrackerPos[1],
			rawTrackerPos[2]
		};
		vr::HmdVector3d_t filteredTrackerPos = cfg.trackerFilterEnabled
			? state.trackerFilter.translation.update(rawTrackerVec)
			: rawTrackerVec;
		double trackerPos[3] = {
			filteredTrackerPos.v[0],
			filteredTrackerPos.v[1],
			filteredTrackerPos.v[2]
		};

		vr::HmdVector3d_t trackerRefPosition = quaternionRotateVector(cfg.calibrationRotation, trackerPos);
		trackerRefPosition.v[0] += cfg.calibrationTranslation.v[0];
		trackerRefPosition.v[1] += cfg.calibrationTranslation.v[1];
		trackerRefPosition.v[2] += cfg.calibrationTranslation.v[2];

		vr::HmdQuaternion_t hmdRotation = quaternionNormalize(cfg.native
			? trackerQuat * cfg.offsetRotation
			: trackerRefRotation * cfg.offsetRotation);
		vr::HmdVector3d_t offset = quaternionRotateVector(
			cfg.native ? trackerQuat : trackerRefRotation, cfg.offsetTranslation.v);

		pose.qWorldFromDriverRotation = { 1, 0, 0, 0 };
		pose.vecWorldFromDriverTranslation[0] = 0;
		pose.vecWorldFromDriverTranslation[1] = 0;
		pose.vecWorldFromDriverTranslation[2] = 0;

		pose.qDriverFromHeadRotation = { 1, 0, 0, 0 };
		pose.vecDriverFromHeadTranslation[0] = 0;
		pose.vecDriverFromHeadTranslation[1] = 0;
		pose.vecDriverFromHeadTranslation[2] = 0;

		if (cfg.native)
		{
			pose.qRotation = hmdRotation;
			pose.vecPosition[0] = trackerPos[0] + offset.v[0];
			pose.vecPosition[1] = trackerPos[1] + offset.v[1];
			pose.vecPosition[2] = trackerPos[2] + offset.v[2];
		}
		else
		{
			pose.qRotation = hmdRotation;
			pose.vecPosition[0] = trackerRefPosition.v[0] + offset.v[0];
			pose.vecPosition[1] = trackerRefPosition.v[1] + offset.v[1];
			pose.vecPosition[2] = trackerRefPosition.v[2] + offset.v[2];
		}

		if (state.headFilter.enabled && !fusionFallthrough)
		{
			double dt = FilterStep(state.headFilter.lastUpdate, state.headFilter.valid, clock.now, clock.freq);
			state.headFilter.valid = true;

			pose.qRotation = state.headFilter.rotationFilter.filter(pose.qRotation, dt);

			vr::HmdVector3d_t headPos = state.headFilter.translationFilter.filter(
				{ pose.vecPosition[0], pose.vecPosition[1], pose.vecPosition[2] }, dt);
			pose.vecPosition[0] = headPos.v[0];
			pose.vecPosition[1] = headPos.v[1];
			pose.vecPosition[2] = headPos.v[2];
		}

		// No calibrationScale here. The position path two blocks up builds trackerRefPosition as
		// R*p + t with no scale factor, so scaling the velocity made the published velocity the
		// derivative of a pose the driver never publishes. Inert while calibrationScale is 1.0
		// (it is, in every logged session), but the two would diverge the moment it is not, and
		// velocity feeds the compositor's prediction.
		vr::HmdVector3d_t vel = quaternionRotateVector(cfg.calibrationRotation, trackerVel);

		// Third fall-through inert step (with prediction and One-Euro above): the headVel chain
		// differentiates successive qRotation samples, and fusion frames never feed it. A dropout
		// that primed it (valid=true, prevRotation stored) followed minutes later by another
		// dropout would difference against that minutes-old rotation across FilterStep's clamped
		// 0.1 s dt — up to ~31 rad/s of bogus omega published at SLAM-loss onset, plus its
		// tangential lever-arm term in vecVelocity. Reset instead of skip, so no stale prevRotation
		// can survive to the next dropout; the docs' "angular-velocity passthrough is inert in
		// fusion" then holds on this path too.
		vr::HmdVector3d_t headAngVel = { 0, 0, 0 };
		if (!fusionFallthrough)
		{
			double dtAng = FilterStep(state.headVel.lastUpdate, state.headVel.valid, clock.now, clock.freq);
			if (state.headVel.valid)
			{
				const vr::HmdVector3d_t rawAngVel =
					quaternionAngularVelocity(pose.qRotation, state.headVel.prevRotation, dtAng);
				// Smooths the PUBLISHED angular velocity only -- the pose itself is untouched
				// either way. Games that extrapolate from vecAngularVelocity between frames will
				// feel this; the rendered pose will not move differently.
				headAngVel = cfg.headVelFilterEnabled
					? state.headVel.filter.filter(rawAngVel, dtAng)
					: rawAngVel;
			}
			state.headVel.prevRotation = pose.qRotation;
			state.headVel.valid = true;
		}
		else
		{
			state.headVel.reset();
		}

		vr::HmdVector3d_t tangential = {
			headAngVel.v[1] * offset.v[2] - headAngVel.v[2] * offset.v[1],
			headAngVel.v[2] * offset.v[0] - headAngVel.v[0] * offset.v[2],
			headAngVel.v[0] * offset.v[1] - headAngVel.v[1] * offset.v[0]
		};

		for (int i = 0; i < 3; i++)
		{
			double baseVel = cfg.native ? trackerVel[i] : vel.v[i];
			pose.vecVelocity[i] = baseVel + tangential.v[i];
			pose.vecAngularVelocity[i] = cfg.enableAngularVelocity ? headAngVel.v[i] : 0.0;
		}
		SanitizeVec3Velocity(pose.vecVelocity, kMaxPlausibleHeadSpeed);
		SanitizeVec3Velocity(pose.vecAngularVelocity, 20.0);

		pose.poseIsValid = true;
		pose.deviceIsConnected = true;
		pose.result = vr::TrackingResult_Running_OK;
		pose.shouldApplyHeadModel = false;
		pose.poseTimeOffset = 0;

		if (!GatePublish(pose, cfg, displayHz, linSpeed, state, diag, clock))
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

			if (!state.drift.valid)
				UpdateDrift(state, diag, cfg, clock, pose.qRotation, pose.vecPosition, rawRotation, rawPosition, 1.0);
			else if (weight > 0.01)
				UpdateDrift(state, diag, cfg, clock, pose.qRotation, pose.vecPosition, rawRotation, rawPosition, weight);
		}
	}
	else
	{
		state.headVel.reset();
		// The tracker pre-filter must go too. It keeps X/P/K across the gap, and its gain has
		// settled to K~0.146, so after a loss during which the user moved it drags the published
		// position back toward where the head was before the dropout for several frames. reset()
		// clears `primed`, so the next good sample re-primes the filter to that sample exactly.
		state.trackerFilter.reset();
		if (ApplyLastGoodHmd(pose, state, clock, 0.15))
		{
			++diag.lastGoodHolds;
			return true;
		}

		if (cfg.slamFallback)
			++diag.fallbackFrames;
		if (!cfg.slamFallback)
		{
			if (cfg.native)
			{
				pose.qWorldFromDriverRotation = { 1, 0, 0, 0 };
				pose.vecWorldFromDriverTranslation[0] = 0;
				pose.vecWorldFromDriverTranslation[1] = 0;
				pose.vecWorldFromDriverTranslation[2] = 0;
			}
			else
			{
				pose.qWorldFromDriverRotation = cfg.calibrationRotation;
				pose.vecWorldFromDriverTranslation[0] = cfg.calibrationTranslation.v[0];
				pose.vecWorldFromDriverTranslation[1] = cfg.calibrationTranslation.v[1];
				pose.vecWorldFromDriverTranslation[2] = cfg.calibrationTranslation.v[2];
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
		else
		{
			ApplyDrift(pose, cfg, state);
			GatePublish(pose, cfg, displayHz, linSpeed, state, diag, clock);
		}
	}

	return true;
}

} // namespace pose_est
