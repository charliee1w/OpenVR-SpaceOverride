// SPDX-License-Identifier: AGPL-3.0-only

#include "calibration/Math.h"

#include <cmath>
#include <algorithm>

namespace calibration
{
	Eigen::Vector3d rotationVector(const Eigen::Matrix3d& rot)
	{
		Eigen::AngleAxisd aa(rot);
		return aa.angle() * aa.axis();
	}

	double angleFromRotationMatrix3(const Eigen::Matrix3d& rot)
	{
		double c = (rot(0, 0) + rot(1, 1) + rot(2, 2) - 1.0) / 2.0;
		return std::acos(std::max(-1.0, std::min(1.0, c)));
	}

	DeltaRotationSample deltaRotationSamples(const Sample& s1, const Sample& s2)
	{
		auto dref = s1.ref.rot * s2.ref.rot.transpose();
		auto dtarget = s1.target.rot * s2.target.rot.transpose();

		DeltaRotationSample ds;
		ds.ref = rotationVector(dref);
		ds.target = rotationVector(dtarget);

		auto refA = angleFromRotationMatrix3(dref);
		auto targetA = angleFromRotationMatrix3(dtarget);
		ds.valid = refA > 0.4 && targetA > 0.4 && ds.ref.norm() > 0.01 && ds.target.norm() > 0.01;

		return ds;
	}

	RotationCalibrationResult calibrateRotation(const std::vector<Sample>& samples)
	{
		std::vector<DeltaRotationSample> deltas;

		for (size_t i = 0; i < samples.size(); i++)
		{
			for (size_t j = 0; j < i; j++)
			{
				auto delta = deltaRotationSamples(samples[i], samples[j]);
				if (delta.valid)
					deltas.push_back(delta);
			}
		}

		RotationCalibrationResult result;
		result.sampleCount = samples.size();
		result.deltaCount = deltas.size();

		if (deltas.empty())
			return result;

		Eigen::MatrixXd refPoints(deltas.size(), 3), targetPoints(deltas.size(), 3);

		for (size_t i = 0; i < deltas.size(); i++)
		{
			refPoints.row(i) = deltas[i].ref;
			targetPoints.row(i) = deltas[i].target;
		}

		auto crossCV = refPoints.transpose() * targetPoints;

		Eigen::BDCSVD<Eigen::MatrixXd> bdcsvd;
		auto svd = bdcsvd.compute(crossCV, Eigen::ComputeThinU | Eigen::ComputeThinV);

		Eigen::Matrix3d i = Eigen::Matrix3d::Identity();
		if ((svd.matrixU() * svd.matrixV().transpose()).determinant() < 0)
			i(2, 2) = -1;

		Eigen::Matrix3d rot = svd.matrixV() * i * svd.matrixU().transpose();
		rot.transposeInPlace();

		result.eulerDegrees = rot.eulerAngles(2, 1, 0) * 180.0 / EIGEN_PI;
		return result;
	}

	Eigen::Vector3d calibrateTranslation(const std::vector<Sample>& samples, const Eigen::Matrix3d& rotation)
	{
		std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> deltas;

		for (size_t i = 0; i < samples.size(); i++)
		{
			Sample s_i = samples[i];
			s_i.target.rot = rotation * s_i.target.rot;
			s_i.target.trans = rotation * s_i.target.trans;

			for (size_t j = 0; j < i; j++)
			{
				Sample s_j = samples[j];
				s_j.target.rot = rotation * s_j.target.rot;
				s_j.target.trans = rotation * s_j.target.trans;

				auto QAi = s_i.ref.rot.transpose();
				auto QAj = s_j.ref.rot.transpose();
				auto dQA = QAj - QAi;
				auto CA = QAj * (s_j.ref.trans - s_j.target.trans) - QAi * (s_i.ref.trans - s_i.target.trans);
				deltas.push_back(std::make_pair(CA, dQA));

				auto QBi = s_i.target.rot.transpose();
				auto QBj = s_j.target.rot.transpose();
				auto dQB = QBj - QBi;
				auto CB = QBj * (s_j.ref.trans - s_j.target.trans) - QBi * (s_i.ref.trans - s_i.target.trans);
				deltas.push_back(std::make_pair(CB, dQB));
			}
		}

		if (deltas.empty())
			return Eigen::Vector3d::Zero();

		Eigen::VectorXd constants(deltas.size() * 3);
		Eigen::MatrixXd coefficients(deltas.size() * 3, 3);

		for (size_t i = 0; i < deltas.size(); i++)
		{
			for (int axis = 0; axis < 3; axis++)
			{
				constants(i * 3 + axis) = deltas[i].first(axis);
				coefficients.row(i * 3 + axis) = deltas[i].second.row(axis);
			}
		}

		Eigen::Vector3d trans = coefficients.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(constants);
		return trans * 100.0;
	}

	double secondAxisVariance(const std::vector<Sample>& samples)
	{
		std::vector<Eigen::Vector4d> points;
		points.reserve(samples.size());
		Eigen::Vector4d mean = Eigen::Vector4d::Zero();

		for (auto& sample : samples)
		{
			Eigen::Quaterniond q(sample.target.rot);
			if (q.w() < 0)
				q.coeffs() = -q.coeffs();

			Eigen::Vector4d point(q.w(), q.x(), q.y(), q.z());
			mean += point;
			points.push_back(point);
		}

		if (points.empty())
			return 0.0;

		mean /= (double)points.size();

		Eigen::Matrix4d cov = Eigen::Matrix4d::Zero();
		for (auto& point : points)
		{
			Eigen::Vector4d d = point - mean;
			cov += d * d.transpose();
		}
		cov /= (double)points.size();

		Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver(cov);
		return solver.eigenvalues()(1);
	}

	Eigen::Vector3d computeRefToTargetOffset(const std::vector<Sample>& samples, const Eigen::Matrix3d& calRot, const Eigen::Vector3d& calTrans)
	{
		Eigen::Vector3d accum = Eigen::Vector3d::Zero();

		for (auto& sample : samples)
			accum += sample.ref.rot.transpose() * (calRot * sample.target.trans + calTrans - sample.ref.trans);

		return accum / (double)samples.size();
	}

	double retargetingErrorRMS(const std::vector<Sample>& samples, const Eigen::Vector3d& hmdToTargetPos, const Eigen::Matrix3d& calRot, const Eigen::Vector3d& calTrans)
	{
		double accum = 0;

		for (auto& sample : samples)
			accum += (calRot * sample.target.trans + calTrans - (sample.ref.rot * hmdToTargetPos + sample.ref.trans)).squaredNorm();

		return std::sqrt(accum / (double)samples.size());
	}

	std::vector<double> computeSampleWeights(const std::vector<Sample>& samples)
	{
		std::vector<double> weights(samples.size(), 1.0);
		if (samples.size() < 2)
			return weights;

		for (size_t i = 1; i < samples.size(); ++i)
		{
			const double step = (samples[i].target.trans - samples[i - 1].target.trans).norm();
			weights[i] = 1.0 / (1.0 + step * step * 10000.0);
		}

		double sum = 0.0;
		for (double w : weights)
			sum += w;
		if (sum > 0.0)
		{
			for (double& w : weights)
				w /= sum;
		}

		return weights;
	}

	static RelativeOffsetResult computeRelativeOffsetImpl(const std::vector<Sample>& samples, const Eigen::Matrix3d& calRot, const Eigen::Vector3d& calTrans, const std::vector<double>* weights)
	{
		RelativeOffsetResult result;

		if (samples.empty())
			return result;

		Eigen::Matrix4d quatAccum = Eigen::Matrix4d::Zero();
		Eigen::Vector3d transAccum = Eigen::Vector3d::Zero();
		double transWeightSum = 0.0;

		for (size_t i = 0; i < samples.size(); ++i)
		{
			const double w = weights ? (*weights)[i] : 1.0;
			auto& sample = samples[i];

			Eigen::Matrix3d trackerRot = calRot * sample.target.rot;
			Eigen::Vector3d trackerTrans = calRot * sample.target.trans + calTrans;

			Eigen::Matrix3d offsetRot = trackerRot.transpose() * sample.ref.rot;
			Eigen::Vector3d offsetTrans = trackerRot.transpose() * (sample.ref.trans - trackerTrans);

			Eigen::Quaterniond q(offsetRot);
			Eigen::Vector4d v(q.w(), q.x(), q.y(), q.z());
			quatAccum += w * (v * v.transpose());
			transAccum += w * offsetTrans;
			transWeightSum += w;
		}

		Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver(quatAccum);
		Eigen::Vector4d avg = solver.eigenvectors().col(3).normalized();

		Eigen::Quaterniond q(avg(0), avg(1), avg(2), avg(3));
		q.normalize();
		if (q.w() < 0)
			q.coeffs() = -q.coeffs();

		result.rotation = q;
		if (transWeightSum > 0.0)
			result.translation = transAccum / transWeightSum;
		else
			result.translation = Eigen::Vector3d::Zero();
		return result;
	}

	RelativeOffsetResult computeRelativeOffset(const std::vector<Sample>& samples, const Eigen::Matrix3d& calRot, const Eigen::Vector3d& calTrans)
	{
		return computeRelativeOffsetImpl(samples, calRot, calTrans, nullptr);
	}

	RelativeOffsetResult computeRelativeOffsetWeighted(const std::vector<Sample>& samples, const Eigen::Matrix3d& calRot, const Eigen::Vector3d& calTrans)
	{
		const auto weights = computeSampleWeights(samples);
		return computeRelativeOffsetImpl(samples, calRot, calTrans, &weights);
	}

	double retargetingErrorRMSWeighted(const std::vector<Sample>& samples, const Eigen::Vector3d& hmdToTargetPos, const Eigen::Matrix3d& calRot, const Eigen::Vector3d& calTrans, const std::vector<double>& weights)
	{
		if (samples.empty() || weights.size() != samples.size())
			return 0.0;

		double accum = 0.0;
		double weightSum = 0.0;

		for (size_t i = 0; i < samples.size(); ++i)
		{
			const double w = weights[i];
			const auto& sample = samples[i];
			const double err = (calRot * sample.target.trans + calTrans - (sample.ref.rot * hmdToTargetPos + sample.ref.trans)).squaredNorm();
			accum += w * err;
			weightSum += w;
		}

		return weightSum > 0.0 ? std::sqrt(accum / weightSum) : 0.0;
	}

	double mountOffsetErrorRMSWeighted(const std::vector<Sample>& samples, const Eigen::Matrix3d& calRot, const Eigen::Vector3d& calTrans, const RelativeOffsetResult& mount, const std::vector<double>& weights)
	{
		if (samples.empty() || weights.size() != samples.size())
			return 0.0;

		Eigen::Matrix3d mountRot = mount.rotation.toRotationMatrix();
		double accum = 0.0;
		double weightSum = 0.0;

		for (size_t i = 0; i < samples.size(); ++i)
		{
			const double w = weights[i];
			const auto& sample = samples[i];

			Eigen::Matrix3d trackerRot = calRot * sample.target.rot;
			Eigen::Vector3d trackerTrans = calRot * sample.target.trans + calTrans;
			Eigen::Vector3d predictedTrans = trackerRot * mount.translation + trackerTrans;
			Eigen::Matrix3d predictedRot = trackerRot * mountRot;

			double transError = (sample.ref.trans - predictedTrans).squaredNorm();
			Eigen::Matrix3d rotDelta = sample.ref.rot * predictedRot.transpose();
			double rotError = angleFromRotationMatrix3(rotDelta);
			accum += w * (transError + rotError * rotError);
			weightSum += w;
		}

		return weightSum > 0.0 ? std::sqrt(accum / weightSum) : 0.0;
	}

	double runtimeMountResidualMm(const Pose& slamHmd, const Pose& tracker, const Eigen::Matrix3d& calRot, const Eigen::Vector3d& calTransM, const RelativeOffsetResult& mount)
	{
		const Eigen::Matrix3d trackerRefRot = calRot * tracker.rot;
		const Eigen::Vector3d trackerRefTrans = calRot * tracker.trans + calTransM;
		const Eigen::Matrix3d mountRot = mount.rotation.toRotationMatrix();
		const Eigen::Vector3d predictedTrans = trackerRefRot * mount.translation + trackerRefTrans;
		return (slamHmd.trans - predictedTrans).norm() * 1000.0;
	}

	GuardianShiftResult detectGuardianShift(const Pose& prevSlam, const Pose& curSlam, const Pose& prevTracker, const Pose& curTracker,
		double slamJumpThresholdMm, double trackerStableThresholdMm)
	{
		GuardianShiftResult result;
		result.slamJumpMm = (curSlam.trans - prevSlam.trans).norm() * 1000.0;
		result.trackerJumpMm = (curTracker.trans - prevTracker.trans).norm() * 1000.0;
		result.suspect = result.slamJumpMm >= slamJumpThresholdMm && result.trackerJumpMm <= trackerStableThresholdMm;
		return result;
	}

	double trackerTranslationJitterRMS(const std::vector<Sample>& samples)
	{
		if (samples.size() < 2)
			return 0.0;

		double accum = 0.0;
		for (size_t i = 1; i < samples.size(); ++i)
		{
			const double step = (samples[i].target.trans - samples[i - 1].target.trans).norm();
			accum += step * step;
		}

		return std::sqrt(accum / (double)(samples.size() - 1));
	}

	double mountOffsetErrorRMS(const std::vector<Sample>& samples, const Eigen::Matrix3d& calRot, const Eigen::Vector3d& calTrans, const RelativeOffsetResult& mount)
	{
		if (samples.empty())
			return 0.0;

		Eigen::Matrix3d mountRot = mount.rotation.toRotationMatrix();
		double accum = 0.0;

		for (auto& sample : samples)
		{
			Eigen::Matrix3d trackerRot = calRot * sample.target.rot;
			Eigen::Vector3d trackerTrans = calRot * sample.target.trans + calTrans;
			Eigen::Vector3d predictedTrans = trackerRot * mount.translation + trackerTrans;
			Eigen::Matrix3d predictedRot = trackerRot * mountRot;

			double transError = (sample.ref.trans - predictedTrans).squaredNorm();
			Eigen::Matrix3d rotDelta = sample.ref.rot * predictedRot.transpose();
			double rotError = angleFromRotationMatrix3(rotDelta);
			accum += transError + rotError * rotError;
		}

		return std::sqrt(accum / (double)samples.size());
	}

	Eigen::Matrix3d eulerDegreesToRotationMatrix(const Eigen::Vector3d& eulerDegrees)
	{
		Eigen::Vector3d eulerRad = eulerDegrees * EIGEN_PI / 180.0;
		return (Eigen::AngleAxisd(eulerRad(0), Eigen::Vector3d::UnitZ()) *
			Eigen::AngleAxisd(eulerRad(1), Eigen::Vector3d::UnitY()) *
			Eigen::AngleAxisd(eulerRad(2), Eigen::Vector3d::UnitX())).toRotationMatrix();
	}
}