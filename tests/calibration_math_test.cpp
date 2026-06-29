// SPDX-License-Identifier: AGPL-3.0-only

#include <calibration/Math.h>
#include <calibration/Drift.h>
#include <calibration/PoseBlend.h>
#include <calibration/PredictionLag.h>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>

using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace
{
	calibration::Pose makeRefPose(double yaw, double pitch, double roll, const Eigen::Vector3d& trans)
	{
		Eigen::Matrix3d rot =
			(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
			 Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
			 Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
		return calibration::Pose(rot, trans);
	}

	std::vector<calibration::Sample> generateRigidSamples(
		const Eigen::Matrix3d& calRot,
		const Eigen::Vector3d& calTransM,
		size_t count)
	{
		std::vector<calibration::Sample> samples;
		samples.reserve(count);

		for (size_t i = 0; i < count; ++i)
		{
			double t = 0.35 * (double)i + 0.5;
			auto ref = makeRefPose(t, t * 0.6, t * 0.25, Eigen::Vector3d(0.1 * (double)i, 0.05 * (double)i, 1.5));
			// Reference (HMD/SLAM) ≈ calRot * target + calTrans — generate tracker poses from known calibration.
			calibration::Pose target(
				calRot.transpose() * ref.rot,
				calRot.transpose() * (ref.trans - calTransM));
			samples.emplace_back(ref, target);
		}

		return samples;
	}

	Eigen::Matrix3d mountRotation(double yaw, double pitch, double roll)
	{
		return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
			Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
			Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
	}
}

TEST_CASE("CalibrateRotation recovers known inter-system rotation", "[calibration]")
{
	const Eigen::Vector3d expectedEuler(12.0, -8.0, 5.0);
	const Eigen::Matrix3d calRot = calibration::eulerDegreesToRotationMatrix(expectedEuler);
	const Eigen::Vector3d calTransM(0.02, -0.01, 0.03);

	auto samples = generateRigidSamples(calRot, calTransM, 12);
	REQUIRE(samples.size() >= 8);

	auto result = calibration::calibrateRotation(samples);
	REQUIRE(result.deltaCount > 0);

	const Eigen::Matrix3d estimated = calibration::eulerDegreesToRotationMatrix(result.eulerDegrees);
	REQUIRE(estimated.isApprox(calRot, 1e-2));
}

TEST_CASE("CalibrateTranslation recovers known inter-system translation", "[calibration]")
{
	const Eigen::Vector3d expectedEuler(15.0, 4.0, -6.0);
	const Eigen::Matrix3d calRot = calibration::eulerDegreesToRotationMatrix(expectedEuler);
	const Eigen::Vector3d expectedTransCm(3.5, -2.0, 11.0);
	const Eigen::Vector3d calTransM = expectedTransCm * 0.01;

	auto samples = generateRigidSamples(calRot, calTransM, 12);
	auto rotResult = calibration::calibrateRotation(samples);
	REQUIRE(rotResult.deltaCount > 0);

	const Eigen::Matrix3d estimatedRot = calibration::eulerDegreesToRotationMatrix(rotResult.eulerDegrees);
	auto estimatedTransCm = calibration::calibrateTranslation(samples, estimatedRot);

	CHECK_THAT(estimatedTransCm.x(), WithinAbs(expectedTransCm.x(), 0.5));
	CHECK_THAT(estimatedTransCm.y(), WithinAbs(expectedTransCm.y(), 0.5));
	CHECK_THAT(estimatedTransCm.z(), WithinAbs(expectedTransCm.z(), 0.5));
}

TEST_CASE("ComputeRelativeOffset is stable for a fixed mount transform", "[calibration]")
{
	const Eigen::Matrix3d calRot = calibration::eulerDegreesToRotationMatrix(Eigen::Vector3d(10.0, -5.0, 3.0));
	const Eigen::Vector3d calTransM(0.01, 0.02, -0.03);
	const Eigen::Matrix3d mountRot = mountRotation(0.2, -0.15, 0.1);
	const Eigen::Vector3d mountTrans(0.04, -0.02, 0.06);

	std::vector<calibration::Sample> samples;
	for (size_t i = 0; i < 10; ++i)
	{
		double t = 0.4 * (double)i + 0.6;
		auto ref = makeRefPose(t, t * 0.5, t * 0.2, Eigen::Vector3d(0.2, 0.1, 1.6));

		Eigen::Matrix3d trackerRot = ref.rot * mountRot.transpose();
		Eigen::Vector3d trackerTrans = ref.trans - trackerRot * mountTrans;
		calibration::Pose target(
			calRot.transpose() * trackerRot,
			calRot.transpose() * (trackerTrans - calTransM));

		samples.emplace_back(ref, target);
	}

	auto first = calibration::computeRelativeOffset(samples, calRot, calTransM);
	auto second = calibration::computeRelativeOffset(samples, calRot, calTransM);

	CHECK(first.rotation.coeffs().isApprox(second.rotation.coeffs(), 1e-9));
	CHECK(first.translation.isApprox(second.translation, 1e-9));
	CHECK(first.rotation.toRotationMatrix().isApprox(mountRot, 1e-2));
	CHECK_THAT(first.translation.x(), WithinAbs(mountTrans.x(), 0.01));
	CHECK_THAT(first.translation.y(), WithinAbs(mountTrans.y(), 0.01));
	CHECK_THAT(first.translation.z(), WithinAbs(mountTrans.z(), 0.01));
}

TEST_CASE("RetargetingErrorRMS is near zero for consistent rigid samples", "[calibration]")
{
	const Eigen::Vector3d expectedEuler(7.0, -3.0, 2.0);
	const Eigen::Matrix3d calRot = calibration::eulerDegreesToRotationMatrix(expectedEuler);
	const Eigen::Vector3d calTransM(0.03, 0.01, -0.02);

	auto samples = generateRigidSamples(calRot, calTransM, 10);
	auto hmdToTarget = calibration::computeRefToTargetOffset(samples, calRot, calTransM);
	double rms = calibration::retargetingErrorRMS(samples, hmdToTarget, calRot, calTransM);

	CHECK(rms < 1e-6);
}

TEST_CASE("RetargetingErrorRMS is high when calibration transform is wrong", "[calibration]")
{
	const Eigen::Matrix3d calRot = calibration::eulerDegreesToRotationMatrix(Eigen::Vector3d(10.0, 5.0, -4.0));
	const Eigen::Vector3d calTransM(0.02, -0.01, 0.04);

	auto samples = generateRigidSamples(calRot, calTransM, 10);
	auto hmdToTarget = calibration::computeRefToTargetOffset(samples, calRot, calTransM);

	const Eigen::Matrix3d wrongRot = calibration::eulerDegreesToRotationMatrix(Eigen::Vector3d(40.0, -20.0, 15.0));
	const Eigen::Vector3d wrongTransM(0.15, 0.08, -0.12);

	double rms = calibration::retargetingErrorRMS(samples, hmdToTarget, wrongRot, wrongTransM);
	CHECK(rms > 0.05);
}

TEST_CASE("Instantaneous drift is identity for matching poses", "[drift]")
{
	auto q = Eigen::Quaterniond(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitY()));
	Eigen::Vector3d pos(0.2, 0.1, 1.5);

	auto sample = calibration::computeInstantaneousDrift(q, pos, q, pos);
	CHECK(sample.rotation.coeffs().isApprox(Eigen::Quaterniond::Identity().coeffs(), 1e-6));
	CHECK(sample.translation.isApprox(Eigen::Vector3d::Zero(), 1e-6));
}

TEST_CASE("Instantaneous drift captures full rotation delta", "[drift]")
{
	Eigen::Quaterniond raw(Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()));
	Eigen::Quaterniond corrected(Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitX()));
	Eigen::Vector3d pos = Eigen::Vector3d::Zero();

	auto sample = calibration::computeInstantaneousDrift(corrected, pos, raw, pos);
	Eigen::Quaterniond expected = (corrected * raw.conjugate()).normalized();
	CHECK(sample.rotation.coeffs().isApprox(expected.coeffs(), 1e-6));
}

TEST_CASE("Drift outlier rejection blocks large spikes", "[drift]")
{
	calibration::DriftInstantaneousSample sample;
	sample.translation = Eigen::Vector3d(0.2, 0.0, 0.0);
	CHECK_FALSE(calibration::isDriftSampleWithinLimits(sample, {}));

	sample.translation = Eigen::Vector3d::Zero();
	sample.rotation = Eigen::Quaterniond(Eigen::AngleAxisd(20.0 * EIGEN_PI / 180.0, Eigen::Vector3d::UnitZ()));
	CHECK_FALSE(calibration::isDriftSampleWithinLimits(sample, {}));
}

TEST_CASE("Rotation step clamp limits per-frame delta", "[drift]")
{
	Eigen::Quaterniond large(Eigen::AngleAxisd(30.0 * EIGEN_PI / 180.0, Eigen::Vector3d::UnitY()));
	auto clamped = calibration::clampRotationStep(large, 5.0);

	double angle = 2.0 * std::acos(std::clamp(std::abs(clamped.w()), 0.0, 1.0)) * 180.0 / EIGEN_PI;
	CHECK(angle <= 5.01);
}

TEST_CASE("Mount offset RMS is low for consistent rigid mount", "[calibration]")
{
	const Eigen::Matrix3d calRot = calibration::eulerDegreesToRotationMatrix(Eigen::Vector3d(10.0, -5.0, 3.0));
	const Eigen::Vector3d calTransM(0.01, 0.02, -0.03);
	const Eigen::Matrix3d mountRot = mountRotation(0.2, -0.15, 0.1);
	const Eigen::Vector3d mountTrans(0.04, -0.02, 0.06);

	std::vector<calibration::Sample> samples;
	for (size_t i = 0; i < 10; ++i)
	{
		double t = 0.4 * (double)i + 0.6;
		auto ref = makeRefPose(t, t * 0.5, t * 0.2, Eigen::Vector3d(0.2, 0.1, 1.6));
		Eigen::Matrix3d trackerRot = ref.rot * mountRot.transpose();
		Eigen::Vector3d trackerTrans = ref.trans - trackerRot * mountTrans;
		calibration::Pose target(calRot.transpose() * trackerRot, calRot.transpose() * (trackerTrans - calTransM));
		samples.emplace_back(ref, target);
	}

	auto mount = calibration::computeRelativeOffset(samples, calRot, calTransM);
	double rms = calibration::mountOffsetErrorRMS(samples, calRot, calTransM, mount);
	CHECK(rms < 0.03);
}

TEST_CASE("slerpQuaternion returns endpoints and midpoint", "[pose_blend]")
{
	Eigen::Quaterniond from(Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()));
	Eigen::Quaterniond to(Eigen::AngleAxisd(1.2, Eigen::Vector3d::UnitX()));

	auto atStart = calibration::slerpQuaternion(from, to, 0.0);
	auto atEnd = calibration::slerpQuaternion(from, to, 1.0);
	auto mid = calibration::slerpQuaternion(from, to, 0.5);

	CHECK(atStart.coeffs().isApprox(from.normalized().coeffs(), 1e-6));
	CHECK(atEnd.coeffs().isApprox(to.normalized().coeffs(), 1e-6));

	double midAngle = 2.0 * std::acos(std::clamp(std::abs(mid.w()), 0.0, 1.0));
	CHECK(midAngle > 0.4);
	CHECK(midAngle < 0.8);
}

TEST_CASE("lerpVector3 returns endpoints and clamps extrapolation", "[pose_blend]")
{
	Eigen::Vector3d from(1.0, 2.0, 3.0);
	Eigen::Vector3d to(4.0, 6.0, 9.0);

	CHECK(calibration::lerpVector3(from, to, 0.0).isApprox(from, 1e-9));
	CHECK(calibration::lerpVector3(from, to, 1.0).isApprox(to, 1e-9));
	CHECK(calibration::lerpVector3(from, to, 0.5).isApprox(Eigen::Vector3d(2.5, 4.0, 6.0), 1e-9));
	CHECK(calibration::lerpVector3(from, to, 2.0).isApprox(to, 1e-9));
}

TEST_CASE("Prediction lag estimate finds delayed SLAM signal", "[prediction]")
{
	calibration::AngVelSeries series;
	const int lag = 2;
	const int count = 80;

	for (int i = 0; i < count; ++i)
	{
		double v = std::sin(i * 0.25);
		series.tracker.push_back(v);
		int slamIndex = i - lag;
		series.slam.push_back(slamIndex >= 0 ? std::sin(slamIndex * 0.25) : 0.0);
	}

	auto estimate = calibration::estimatePredictionLagFrames(series, 45, 3);
	REQUIRE(estimate.valid);
	CHECK(estimate.lagFrames == lag);
}

TEST_CASE("Prediction lag estimate uses linear velocity when angular signal is flat", "[prediction]")
{
	calibration::AngVelSeries series;
	const int lag = 1;
	const int count = 80;

	for (int i = 0; i < count; ++i)
	{
		series.tracker.push_back(0.1);
		int slamIndex = i - lag;
		series.slam.push_back(slamIndex >= 0 ? 0.1 : 0.0);

		const double lin = std::sin(i * 0.3);
		series.trackerLin.push_back(lin);
		series.slamLin.push_back(slamIndex >= 0 ? std::sin(slamIndex * 0.3) : 0.0);
	}

	auto estimate = calibration::estimatePredictionLagFrames(series, 45, 3);
	REQUIRE(estimate.valid);
	CHECK(estimate.lagFrames == lag);
}

TEST_CASE("Prediction lag estimate rejects flat signals", "[prediction]")
{
	calibration::AngVelSeries series;
	for (int i = 0; i < 60; ++i)
	{
		series.tracker.push_back(0.1);
		series.slam.push_back(0.1);
	}

	auto estimate = calibration::estimatePredictionLagFrames(series, 45, 3);
	CHECK_FALSE(estimate.valid);
}

TEST_CASE("TrackerTranslationJitterRMS is zero for static tracker", "[calibration]")
{
	std::vector<calibration::Sample> samples;
	const Eigen::Vector3d pos(1.0, 0.5, 1.5);
	for (size_t i = 0; i < 10; ++i)
	{
		auto ref = makeRefPose(0.1 * (double)i, 0.0, 0.0, Eigen::Vector3d::Zero());
		samples.emplace_back(ref, calibration::Pose(ref.rot, pos));
	}

	CHECK(calibration::trackerTranslationJitterRMS(samples) < 1e-9);
}

TEST_CASE("TrackerTranslationJitterRMS increases with stepwise motion", "[calibration]")
{
	std::vector<calibration::Sample> samples;
	for (size_t i = 0; i < 10; ++i)
	{
		auto ref = makeRefPose(0.1 * (double)i, 0.0, 0.0, Eigen::Vector3d::Zero());
		samples.emplace_back(ref, calibration::Pose(ref.rot, Eigen::Vector3d(0.01 * (double)i, 0.0, 0.0)));
	}

	CHECK(calibration::trackerTranslationJitterRMS(samples) > 0.005);
}

TEST_CASE("SecondAxisVariance rejects coplanar head motion", "[calibration]")
{
	std::vector<calibration::Sample> varied;
	for (size_t i = 0; i < 12; ++i)
	{
		double t = 0.35 * (double)i + 0.5;
		auto ref = makeRefPose(t, t * 0.6, t * 0.25, Eigen::Vector3d::Zero());
		varied.emplace_back(ref, ref);
	}

	std::vector<calibration::Sample> coplanar;
	for (size_t i = 0; i < 12; ++i)
	{
		double t = 0.35 * (double)i + 0.5;
		auto ref = makeRefPose(t, 0.0, 0.0, Eigen::Vector3d::Zero());
		coplanar.emplace_back(ref, ref);
	}

	double variedVariance = calibration::secondAxisVariance(varied);
	double coplanarVariance = calibration::secondAxisVariance(coplanar);

	CHECK(variedVariance > calibration::AxisVarianceThreshold);
	CHECK(coplanarVariance < calibration::AxisVarianceThreshold);
}

TEST_CASE("Weighted mount offset RMS matches unweighted on uniform rigid data", "[tracking_quality]")
{
	const Eigen::Vector3d expectedEuler(8.0, -4.0, 2.0);
	const Eigen::Matrix3d calRot = calibration::eulerDegreesToRotationMatrix(expectedEuler);
	const Eigen::Vector3d calTransM(0.01, -0.02, 0.03);
	const Eigen::Matrix3d mountRot = mountRotation(0.1, 0.2, -0.05);
	const Eigen::Vector3d mountTrans(0.02, 0.01, -0.03);

	auto samples = generateRigidSamples(calRot, calTransM, 16);
	for (auto& s : samples)
	{
		const Eigen::Matrix3d trackerRefRot = calRot * s.target.rot;
		const Eigen::Vector3d trackerRefTrans = calRot * s.target.trans + calTransM;
		s.ref.rot = trackerRefRot * mountRot;
		s.ref.trans = trackerRefRot * mountTrans + trackerRefTrans;
	}

	calibration::RelativeOffsetResult mount;
	mount.rotation = Eigen::Quaterniond(mountRot);
	mount.translation = mountTrans;

	const auto weights = calibration::computeSampleWeights(samples);
	const double unweighted = calibration::mountOffsetErrorRMS(samples, calRot, calTransM, mount);
	const double weighted = calibration::mountOffsetErrorRMSWeighted(samples, calRot, calTransM, mount, weights);

	CHECK(unweighted < 0.005);
	CHECK(weighted < 0.005);
	CHECK_THAT(weighted, WithinAbs(unweighted, 0.001));
}

TEST_CASE("Runtime mount residual is near zero for consistent poses", "[tracking_quality]")
{
	const Eigen::Matrix3d calRot = calibration::eulerDegreesToRotationMatrix(Eigen::Vector3d(5.0, -3.0, 2.0));
	const Eigen::Vector3d calTransM(0.01, 0.0, 0.02);
	const Eigen::Matrix3d mountRot = mountRotation(0.15, -0.1, 0.05);
	const Eigen::Vector3d mountTrans(0.03, -0.01, 0.02);

	calibration::RelativeOffsetResult mount;
	mount.rotation = Eigen::Quaterniond(mountRot);
	mount.translation = mountTrans;

	const calibration::Pose tracker(Eigen::Matrix3d::Identity(), Eigen::Vector3d(1.0, 0.5, 1.5));
	const Eigen::Matrix3d trackerRefRot = calRot * tracker.rot;
	const Eigen::Vector3d trackerRefTrans = calRot * tracker.trans + calTransM;
	const calibration::Pose slamHmd(trackerRefRot * mountRot, trackerRefRot * mountTrans + trackerRefTrans);

	const double residual = calibration::runtimeMountResidualMm(slamHmd, tracker, calRot, calTransM, mount);
	CHECK(residual < 0.1);
}

TEST_CASE("Guardian shift detector flags SLAM jump with stable tracker", "[tracking_quality]")
{
	const calibration::Pose prevSlam(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
	const calibration::Pose curSlam(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.05, 0.0, 0.0));
	const calibration::Pose prevTracker(Eigen::Matrix3d::Identity(), Eigen::Vector3d(1.0, 0.0, 1.5));
	const calibration::Pose curTracker(Eigen::Matrix3d::Identity(), Eigen::Vector3d(1.001, 0.0, 1.5));

	const auto result = calibration::detectGuardianShift(prevSlam, curSlam, prevTracker, curTracker);
	CHECK(result.suspect);
	CHECK(result.slamJumpMm >= 45.0);
	CHECK(result.trackerJumpMm < 5.0);
}

TEST_CASE("Guardian shift detector ignores correlated motion", "[tracking_quality]")
{
	const calibration::Pose prevSlam(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
	const calibration::Pose curSlam(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.05, 0.0, 0.0));
	const calibration::Pose prevTracker(Eigen::Matrix3d::Identity(), Eigen::Vector3d(1.0, 0.0, 1.5));
	const calibration::Pose curTracker(Eigen::Matrix3d::Identity(), Eigen::Vector3d(1.05, 0.0, 1.5));

	const auto result = calibration::detectGuardianShift(prevSlam, curSlam, prevTracker, curTracker);
	CHECK_FALSE(result.suspect);
}