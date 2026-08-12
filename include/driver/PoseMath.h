// SPDX-License-Identifier: AGPL-3.0-only
// Pure pose math shared by the live driver and offline PoseEstimator callers.
// No SteamVR context, no locks, no I/O.

#pragma once

#include <openvr_driver.h>

#include <cmath>
#include <cstdint>

namespace pose_math {

// Fastest plausible head motion. Reject gate, jump-gate speed limb, and
// SanitizeVec3Velocity clamp for published head rates must share this value.
inline constexpr double kMaxPlausibleHeadSpeed = 8.0; // m/s

inline vr::HmdQuaternion_t operator*(const vr::HmdQuaternion_t& lhs, const vr::HmdQuaternion_t& rhs)
{
	return {
		(lhs.w * rhs.w) - (lhs.x * rhs.x) - (lhs.y * rhs.y) - (lhs.z * rhs.z),
		(lhs.w * rhs.x) + (lhs.x * rhs.w) + (lhs.y * rhs.z) - (lhs.z * rhs.y),
		(lhs.w * rhs.y) + (lhs.y * rhs.w) + (lhs.z * rhs.x) - (lhs.x * rhs.z),
		(lhs.w * rhs.z) + (lhs.z * rhs.w) + (lhs.x * rhs.y) - (lhs.y * rhs.x)
	};
}

inline vr::HmdVector3d_t quaternionRotateVector(const vr::HmdQuaternion_t& quat, const double (&vector)[3])
{
	vr::HmdQuaternion_t vectorQuat = { 0.0, vector[0], vector[1], vector[2] };
	vr::HmdQuaternion_t conjugate = { quat.w, -quat.x, -quat.y, -quat.z };
	auto rotatedVectorQuat = quat * vectorQuat * conjugate;
	return { rotatedVectorQuat.x, rotatedVectorQuat.y, rotatedVectorQuat.z };
}

inline vr::HmdQuaternion_t quaternionNormalize(vr::HmdQuaternion_t q)
{
	double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
	if (n > 0.0)
	{
		q.w /= n;
		q.x /= n;
		q.y /= n;
		q.z /= n;
	}
	return q;
}

inline vr::HmdQuaternion_t quaternionConjugate(const vr::HmdQuaternion_t& q)
{
	return { q.w, -q.x, -q.y, -q.z };
}

inline vr::HmdQuaternion_t quaternionProjectYaw(const vr::HmdQuaternion_t& q)
{
	double n = std::sqrt(q.w * q.w + q.y * q.y);
	if (n < 1e-9)
		return { 1, 0, 0, 0 };
	return { q.w / n, 0.0, q.y / n, 0.0 };
}

inline vr::HmdQuaternion_t yawQuaternion(double angle)
{
	return { std::cos(angle * 0.5), 0.0, std::sin(angle * 0.5), 0.0 };
}

inline vr::HmdVector3d_t quaternionAngularVelocity(
	const vr::HmdQuaternion_t& cur, const vr::HmdQuaternion_t& prev, double dt)
{
	if (dt <= 0.0)
		return { 0, 0, 0 };

	vr::HmdQuaternion_t d = quaternionNormalize(cur * quaternionConjugate(prev));
	if (d.w < 0.0)
	{
		d.w = -d.w;
		d.x = -d.x;
		d.y = -d.y;
		d.z = -d.z;
	}

	double s = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
	if (s < 1e-9)
		return { 0, 0, 0 };

	double scale = (2.0 * std::atan2(s, d.w)) / (s * dt);
	return { d.x * scale, d.y * scale, d.z * scale };
}

// Inverse of quaternionAngularVelocity for a world-frame omega (N1-a(ii)).
inline vr::HmdQuaternion_t quaternionIntegrateOmega(
	const vr::HmdQuaternion_t& q, const double omega[3], double dt)
{
	if (dt <= 0.0)
		return q;
	const double w = std::sqrt(omega[0] * omega[0] + omega[1] * omega[1] + omega[2] * omega[2]);
	if (w < 1e-12)
		return q;
	const double half = 0.5 * w * dt;
	const double s = std::sin(half) / w;
	const vr::HmdQuaternion_t dq = { std::cos(half), omega[0] * s, omega[1] * s, omega[2] * s };
	return quaternionNormalize(dq * q);
}

inline void SanitizeVec3Velocity(double v[3], double maxSpeed)
{
	if (!std::isfinite(v[0]) || !std::isfinite(v[1]) || !std::isfinite(v[2]))
	{
		v[0] = v[1] = v[2] = 0.0;
		return;
	}
	const double sp = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
	if (sp > maxSpeed && sp > 1e-12)
	{
		const double s = maxSpeed / sp;
		v[0] *= s;
		v[1] *= s;
		v[2] *= s;
	}
}

inline double SoftGateWeight(double speed, double softMax, double hardMax)
{
	if (hardMax <= softMax)
		return speed <= softMax ? 1.0 : 0.0;
	if (speed <= softMax)
		return 1.0;
	if (speed >= hardMax)
		return 0.0;
	double t = (speed - softMax) / (hardMax - softMax);
	t = t * t * (3.0 - 2.0 * t);
	return 1.0 - t;
}

inline vr::HmdQuaternion_t NlerpQuat(const vr::HmdQuaternion_t& a, const vr::HmdQuaternion_t& b, double t)
{
	double dot = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
	vr::HmdQuaternion_t bb = b;
	if (dot < 0.0)
	{
		bb.w = -bb.w;
		bb.x = -bb.x;
		bb.y = -bb.y;
		bb.z = -bb.z;
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

inline bool IsFiniteVec3(const double v[3])
{
	return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

inline bool IsFiniteQuat(const vr::HmdQuaternion_t& q)
{
	if (!std::isfinite(q.w) || !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z))
		return false;
	const double n2 = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
	return n2 > 1e-12;
}

// Full driver→world composition. Publish gates must run on this, not vecPosition alone.
inline void ComposeWorldPose(const vr::DriverPose_t& pose, vr::HmdQuaternion_t& outRot, double outPos[3])
{
	outRot = quaternionNormalize(pose.qWorldFromDriverRotation * pose.qRotation * pose.qDriverFromHeadRotation);

	vr::HmdVector3d_t headLocal = quaternionRotateVector(pose.qRotation, pose.vecDriverFromHeadTranslation);
	double driverLocal[3] = {
		pose.vecPosition[0] + headLocal.v[0],
		pose.vecPosition[1] + headLocal.v[1],
		pose.vecPosition[2] + headLocal.v[2]
	};
	vr::HmdVector3d_t world = quaternionRotateVector(pose.qWorldFromDriverRotation, driverLocal);
	outPos[0] = world.v[0] + pose.vecWorldFromDriverTranslation[0];
	outPos[1] = world.v[1] + pose.vecWorldFromDriverTranslation[1];
	outPos[2] = world.v[2] + pose.vecWorldFromDriverTranslation[2];
}

inline void WriteWorldPose(vr::DriverPose_t& pose, const vr::HmdQuaternion_t& worldRot, const double worldPos[3])
{
	pose.qWorldFromDriverRotation = { 1, 0, 0, 0 };
	pose.vecWorldFromDriverTranslation[0] = 0;
	pose.vecWorldFromDriverTranslation[1] = 0;
	pose.vecWorldFromDriverTranslation[2] = 0;
	pose.qDriverFromHeadRotation = { 1, 0, 0, 0 };
	pose.vecDriverFromHeadTranslation[0] = 0;
	pose.vecDriverFromHeadTranslation[1] = 0;
	pose.vecDriverFromHeadTranslation[2] = 0;
	pose.qRotation = worldRot;
	pose.vecPosition[0] = worldPos[0];
	pose.vecPosition[1] = worldPos[1];
	pose.vecPosition[2] = worldPos[2];
}

// dt from a previous tick. Does not read wall clock — caller injects now/freq (ticks).
inline double FilterStep(int64_t& lastUpdate, bool primed, int64_t now, int64_t freq)
{
	double dt = primed && freq != 0 ? (now - lastUpdate) / (double)freq : 0.0;
	lastUpdate = now;
	if (dt <= 0.0 || std::isnan(dt))
		dt = 1.0 / 90.0;
	if (dt > 0.1)
		dt = 0.1;
	return dt;
}

inline double SlamToCorrectedScale(double hmdScale, double calibrationScale, bool native)
{
	double k = hmdScale > 0.0 ? 1.0 / hmdScale : 1.0;
	return native ? k : k * calibrationScale;
}

} // namespace pose_math
