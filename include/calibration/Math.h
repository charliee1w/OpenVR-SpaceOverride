// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include <Dense>
#include <cstddef>
#include <vector>

namespace calibration
{
	struct Pose
	{
		Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
		Eigen::Vector3d trans = Eigen::Vector3d::Zero();

		Pose() = default;
		Pose(const Eigen::Matrix3d& rot, const Eigen::Vector3d& trans) : rot(rot), trans(trans) { }
	};

	struct Sample
	{
		Pose ref;
		Pose target;
		bool valid = false;

		Sample() = default;
		Sample(Pose ref, Pose target) : ref(ref), target(target), valid(true) { }
	};

	struct DeltaRotationSample
	{
		bool valid = false;
		Eigen::Vector3d ref = Eigen::Vector3d::Zero();
		Eigen::Vector3d target = Eigen::Vector3d::Zero();
	};

	struct RotationCalibrationResult
	{
		Eigen::Vector3d eulerDegrees = Eigen::Vector3d::Zero();
		size_t sampleCount = 0;
		size_t deltaCount = 0;
	};

	struct RelativeOffsetResult
	{
		Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
		Eigen::Vector3d translation = Eigen::Vector3d::Zero();
	};

	struct GuardianShiftResult
	{
		bool suspect = false;
		double slamJumpMm = 0.0;
		double trackerJumpMm = 0.0;
	};

	// Reject calibration when head movement is too coplanar (see CalibrationTick).
	constexpr double AxisVarianceThreshold = 0.0005;

	Eigen::Vector3d rotationVector(const Eigen::Matrix3d& rot);
	double angleFromRotationMatrix3(const Eigen::Matrix3d& rot);

	DeltaRotationSample deltaRotationSamples(const Sample& s1, const Sample& s2);

	RotationCalibrationResult calibrateRotation(const std::vector<Sample>& samples);
	Eigen::Vector3d calibrateTranslation(const std::vector<Sample>& samples, const Eigen::Matrix3d& rotation);

	double secondAxisVariance(const std::vector<Sample>& samples);
	Eigen::Vector3d computeRefToTargetOffset(const std::vector<Sample>& samples, const Eigen::Matrix3d& calRot, const Eigen::Vector3d& calTrans);
	double retargetingErrorRMS(const std::vector<Sample>& samples, const Eigen::Vector3d& hmdToTargetPos, const Eigen::Matrix3d& calRot, const Eigen::Vector3d& calTrans);

	RelativeOffsetResult computeRelativeOffset(const std::vector<Sample>& samples, const Eigen::Matrix3d& calRot, const Eigen::Vector3d& calTrans);
	RelativeOffsetResult computeRelativeOffsetWeighted(const std::vector<Sample>& samples, const Eigen::Matrix3d& calRot, const Eigen::Vector3d& calTrans);

	std::vector<double> computeSampleWeights(const std::vector<Sample>& samples);

	double retargetingErrorRMSWeighted(const std::vector<Sample>& samples, const Eigen::Vector3d& hmdToTargetPos, const Eigen::Matrix3d& calRot, const Eigen::Vector3d& calTrans, const std::vector<double>& weights);
	double mountOffsetErrorRMSWeighted(const std::vector<Sample>& samples, const Eigen::Matrix3d& calRot, const Eigen::Vector3d& calTrans, const RelativeOffsetResult& mount, const std::vector<double>& weights);

	double runtimeMountResidualMm(const Pose& slamHmd, const Pose& tracker, const Eigen::Matrix3d& calRot, const Eigen::Vector3d& calTransM, const RelativeOffsetResult& mount);

	GuardianShiftResult detectGuardianShift(const Pose& prevSlam, const Pose& curSlam, const Pose& prevTracker, const Pose& curTracker,
		double slamJumpThresholdMm = 15.0, double trackerStableThresholdMm = 5.0);

	double mountOffsetErrorRMS(const std::vector<Sample>& samples, const Eigen::Matrix3d& calRot, const Eigen::Vector3d& calTrans, const RelativeOffsetResult& mount);

	// RMS of frame-to-frame tracker translation step lengths in meters (head-mount jitter indicator).
	double trackerTranslationJitterRMS(const std::vector<Sample>& samples);

	Eigen::Matrix3d eulerDegreesToRotationMatrix(const Eigen::Vector3d& eulerDegrees);
}