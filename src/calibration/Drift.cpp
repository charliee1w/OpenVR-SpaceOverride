// SPDX-License-Identifier: AGPL-3.0-only

#include "calibration/Drift.h"

#include <cmath>

namespace calibration
{
	static double quaternionAngleDegrees(const Eigen::Quaterniond& q)
	{
		Eigen::Quaterniond n = q.normalized();
		if (n.w() < 0.0)
			n.coeffs() = -n.coeffs();

		double dot = std::clamp(n.w(), -1.0, 1.0);
		return 2.0 * std::acos(dot) * 180.0 / EIGEN_PI;
	}

	DriftInstantaneousSample computeInstantaneousDrift(
		const Eigen::Quaterniond& correctedRotation,
		const Eigen::Vector3d& correctedPosition,
		const Eigen::Quaterniond& rawRotation,
		const Eigen::Vector3d& rawPosition)
	{
		DriftInstantaneousSample sample;
		sample.rotation = (correctedRotation * rawRotation.conjugate()).normalized();
		sample.translation = correctedPosition - sample.rotation * rawPosition;
		return sample;
	}

	bool isDriftSampleWithinLimits(const DriftInstantaneousSample& sample, const DriftSampleLimits& limits)
	{
		if (sample.translation.norm() > limits.maxTranslationMeters)
			return false;

		return quaternionAngleDegrees(sample.rotation) <= limits.maxRotationDegrees;
	}

	Eigen::Quaterniond clampRotationStep(const Eigen::Quaterniond& delta, double maxDegrees)
	{
		Eigen::Quaterniond normalized = delta.normalized();
		double angle = quaternionAngleDegrees(normalized);
		if (angle <= maxDegrees || angle < 1e-9)
			return normalized;

		double t = maxDegrees / angle;
		return Eigen::Quaterniond::Identity().slerp(t, normalized);
	}
}