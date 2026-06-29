// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include <Dense>

namespace calibration
{
	struct DriftInstantaneousSample
	{
		Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
		Eigen::Vector3d translation = Eigen::Vector3d::Zero();
	};

	struct DriftSampleLimits
	{
		double maxTranslationMeters = 0.05;
		double maxRotationDegrees = 10.0;
	};

	DriftInstantaneousSample computeInstantaneousDrift(
		const Eigen::Quaterniond& correctedRotation,
		const Eigen::Vector3d& correctedPosition,
		const Eigen::Quaterniond& rawRotation,
		const Eigen::Vector3d& rawPosition);

	bool isDriftSampleWithinLimits(const DriftInstantaneousSample& sample, const DriftSampleLimits& limits);

	Eigen::Quaterniond clampRotationStep(const Eigen::Quaterniond& delta, double maxDegrees);
}