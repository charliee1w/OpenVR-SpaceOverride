// SPDX-License-Identifier: AGPL-3.0-only

#define WIN32_LEAN_AND_MEAN

#include "Calibration.h"
#include "Configuration.h"
#include "IPCClient.h"

#include <string>
#include <vector>
#include <set>
#include <iostream>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <fstream>
#include <iomanip>

#include <windows.h>

#include <Dense>


static IPCClient Driver;
CalibrationContext CalCtx;

// Calibration results previously existed only in the overlay's message list, so a
// past calibration's quality could not be reviewed afterwards - which is exactly
// what you need when repeat calibrations disagree. Append a one-line record per
// completed calibration next to the driver's session logs.
static void LogCalibrationResult(const CalibrationContext &ctx, double rmsErrorMm, double spreadM,
	int accepted, int rejAng, int rejLin, int coverageCells)
{
	char localAppData[MAX_PATH] = {};
	DWORD n = GetEnvironmentVariableA("LOCALAPPDATA", localAppData, MAX_PATH);
	if (n == 0 || n >= MAX_PATH)
		return;

	std::string dir = std::string(localAppData) + "\\OpenVR-SpaceOverride\\logs";
	CreateDirectoryA((std::string(localAppData) + "\\OpenVR-SpaceOverride").c_str(), nullptr);
	CreateDirectoryA(dir.c_str(), nullptr);

	SYSTEMTIME st;
	GetLocalTime(&st);

	std::ofstream out(dir + "\\calibration.log", std::ios::app);
	if (!out)
		return;

	out << std::fixed << std::setprecision(5)
		<< st.wYear << "-" << std::setw(2) << std::setfill('0') << st.wMonth << "-"
		<< std::setw(2) << std::setfill('0') << st.wDay << " "
		<< std::setw(2) << std::setfill('0') << st.wHour << ":"
		<< std::setw(2) << std::setfill('0') << st.wMinute << ":"
		<< std::setw(2) << std::setfill('0') << st.wSecond << std::setfill(' ')
		<< "  rms_mm=" << std::setprecision(1) << rmsErrorMm
		<< "  spread_m=" << std::setprecision(3) << spreadM
		<< "  hmdScale=" << std::setprecision(5) << ctx.hmdScale
		<< "  targetModelScale=" << ctx.targetModelScale
		<< "  speed=" << (int)ctx.calibrationSpeed
		// Distinguishes a well-covered calibration from a clumped one: accepted samples
		// plus how many were rejected for moving too fast (angular vs linear), and how
		// many distinct 10cm cells the accepted samples occupy. Many samples in few
		// cells = clumped, which the RMS residual alone cannot reveal.
		<< "  accepted=" << accepted
		<< "  rej_ang=" << rejAng
		<< "  rej_lin=" << rejLin
		<< "  cells=" << coverageCells
		<< "  tracker=" << ctx.trackerSerial
		<< "\n";
}

// Last commands successfully sent to the driver. Scan runs every ~1s; skip no-ops.
struct AppliedDeviceTransform
{
	bool known = false;
	bool enabled = false;
	vr::HmdVector3d_t translation = { 0, 0, 0 };
	vr::HmdQuaternion_t rotation = { 1, 0, 0, 0 };
	double scale = 1.0;
};

static AppliedDeviceTransform g_appliedTf[vr::k_unMaxTrackedDeviceCount];
static bool g_appliedSlamKnown[vr::k_unMaxTrackedDeviceCount];
static bool g_appliedSlam[vr::k_unMaxTrackedDeviceCount];
static protocol::SetHmdTracker g_appliedHmd{};
static bool g_appliedHmdKnown = false;
static protocol::SetOneEuro g_appliedOneEuro{};
static bool g_appliedOneEuroKnown = false;

static void InvalidateAppliedDriverState()
{
	for (uint32_t i = 0; i < vr::k_unMaxTrackedDeviceCount; ++i)
	{
		g_appliedTf[i] = AppliedDeviceTransform{};
		g_appliedSlamKnown[i] = false;
		g_appliedSlam[i] = false;
	}
	g_appliedHmdKnown = false;
	g_appliedOneEuroKnown = false;
}

static bool ApproxEq(double a, double b, double eps = 1e-9)
{
	return std::fabs(a - b) <= eps;
}

static bool VecEq(const vr::HmdVector3d_t &a, const vr::HmdVector3d_t &b)
{
	return ApproxEq(a.v[0], b.v[0]) && ApproxEq(a.v[1], b.v[1]) && ApproxEq(a.v[2], b.v[2]);
}

static bool QuatEq(const vr::HmdQuaternion_t &a, const vr::HmdQuaternion_t &b)
{
	// Same rotation if equal or negated (q and -q).
	bool same = ApproxEq(a.w, b.w) && ApproxEq(a.x, b.x) && ApproxEq(a.y, b.y) && ApproxEq(a.z, b.z);
	bool neg = ApproxEq(a.w, -b.w) && ApproxEq(a.x, -b.x) && ApproxEq(a.y, -b.y) && ApproxEq(a.z, -b.z);
	return same || neg;
}

static bool HmdTrackerEq(const protocol::SetHmdTracker &a, const protocol::SetHmdTracker &b)
{
	return a.hmdID == b.hmdID
		&& a.trackerID == b.trackerID
		&& a.enabled == b.enabled
		&& a.native == b.native
		&& a.slamFallback == b.slamFallback
		&& a.enableAngularVelocity == b.enableAngularVelocity
		&& ApproxEq(a.predictionTime, b.predictionTime, 1e-4)
		&& QuatEq(a.offsetRotation, b.offsetRotation)
		&& VecEq(a.offsetTranslation, b.offsetTranslation)
		&& QuatEq(a.calibrationRotation, b.calibrationRotation)
		&& VecEq(a.calibrationTranslation, b.calibrationTranslation)
		&& ApproxEq(a.calibrationScale, b.calibrationScale)
		&& ApproxEq(a.hmdScale, b.hmdScale);
}

static bool OneEuroEq(const protocol::OneEuroParams &a, const protocol::OneEuroParams &b)
{
	return ApproxEq(a.minCutoff, b.minCutoff) && ApproxEq(a.beta, b.beta) && ApproxEq(a.dCutoff, b.dCutoff);
}

void InitCalibrator()
{
	Driver.Connect();
	// Driver is fresh after connect; force a full push on the next scan.
	InvalidateAppliedDriverState();
}

struct Pose
{
	Eigen::Matrix3d rot;
	Eigen::Vector3d trans;

	Pose() { }
	Pose(vr::HmdMatrix34_t hmdMatrix)
	{
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				rot(i,j) = hmdMatrix.m[i][j];
			}
		}
		trans = Eigen::Vector3d(hmdMatrix.m[0][3], hmdMatrix.m[1][3], hmdMatrix.m[2][3]);
	}
	Pose(double x, double y, double z) : trans(Eigen::Vector3d(x,y,z)) { }
};

struct Sample
{
	Pose ref, target;
	bool valid;
	Sample() : valid(false) { }
	Sample(Pose ref, Pose target) : valid(true), ref(ref), target(target) { }
};

struct DSample
{
	bool valid;
	Eigen::Vector3d ref, target;
};

bool StartsWith(const std::string &str, const std::string &prefix)
{
	if (str.length() < prefix.length())
		return false;

	return str.compare(0, prefix.length(), prefix) == 0;
}

bool EndsWith(const std::string &str, const std::string &suffix)
{
	if (str.length() < suffix.length())
		return false;

	return str.compare(str.length() - suffix.length(), suffix.length(), suffix) == 0;
}

Eigen::Vector3d RotationVector(const Eigen::Matrix3d& rot)
{
	Eigen::AngleAxisd aa(rot);
	return aa.angle() * aa.axis();
}

double AngleFromRotationMatrix3(const Eigen::Matrix3d& rot)
{
	double c = (rot(0, 0) + rot(1, 1) + rot(2, 2) - 1.0) / 2.0;
	return acos(max(-1.0, min(1.0, c)));
}

struct DetectionState
{
	std::vector<uint32_t> candidates;
	std::vector<std::vector<double>> candidateSpeeds;
	std::vector<double> hmdSpeeds;
	std::vector<Eigen::Matrix3d> prevRot; // [0] = HMD, [i+1] = candidates[i]
	bool havePrev = false;
	double prevTime = 0;

	void Clear()
	{
		candidates.clear();
		candidateSpeeds.clear();
		hmdSpeeds.clear();
		prevRot.clear();
		havePrev = false;
		prevTime = 0;
	}
};

static DetectionState Detection;

static std::string GetDeviceSerial(uint32_t id)
{
	char serial[vr::k_unMaxPropertyStringSize] = {};
	vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_SerialNumber_String, serial, vr::k_unMaxPropertyStringSize);
	return std::string(serial);
}

static std::string GetDeviceTrackingSystem(uint32_t id)
{
	char system[vr::k_unMaxPropertyStringSize] = {};
	vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_TrackingSystemName_String, system, vr::k_unMaxPropertyStringSize);
	return std::string(system);
}

static std::string GetDeviceModelNumber(uint32_t id)
{
	char model[vr::k_unMaxPropertyStringSize] = {};
	vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_ModelNumber_String, model, vr::k_unMaxPropertyStringSize);
	return std::string(model);
}

struct ModelScaleEntry
{
	const char *pattern;
	double scale;
};

static const ModelScaleEntry ModelScales[] = {
	{ "tundra tracker",           0.9969 	},  	// Tundra Tracker
	{ "vive tracker 3.0 mv",      1.0034 	},  	// HTC Vive Tracker 3.0
	{ "vive tracker mv",          1.00585 	}, 		// HTC Vive Tracker 1.0 / 2018
};

static double GetLighthouseModelScale(uint32_t id)
{
	if (id == vr::k_unTrackedDeviceIndexInvalid) 
		return 1.0;

	std::string model = GetDeviceModelNumber(id);
	std::transform(model.begin(), model.end(), model.begin(),
		[](unsigned char c) { return (char)std::tolower(c); });

	for (auto &entry : ModelScales)
	{
		if (model.find(entry.pattern) != std::string::npos)
			return entry.scale;
	}

	return 1.0;
}

static double AngularSpeedBetween(const Eigen::Matrix3d &cur, const Eigen::Matrix3d &prev, double dt)
{
	Eigen::Matrix3d delta = cur * prev.transpose();
	double c = (delta(0,0) + delta(1,1) + delta(2,2) - 1.0) / 2.0;
	if (c > 1.0) c = 1.0;
	if (c < -1.0) c = -1.0;
	return acos(c) / dt;
}

static double PearsonCorrelation(const std::vector<double> &a, const std::vector<double> &b)
{
	if (a.size() != b.size() || a.empty())
		return 0.0;

	double meanA = 0, meanB = 0;
	for (size_t i = 0; i < a.size(); i++) { meanA += a[i]; meanB += b[i]; }
	meanA /= a.size();
	meanB /= b.size();

	double cov = 0, varA = 0, varB = 0;
	for (size_t i = 0; i < a.size(); i++)
	{
		double da = a[i] - meanA, db = b[i] - meanB;
		cov += da * db;
		varA += da * da;
		varB += db * db;
	}

	if (varA < 1e-9 || varB < 1e-9)
		return 0.0;

	return cov / std::sqrt(varA * varB);
}

DSample DeltaRotationSamples(Sample s1, Sample s2)
{
	// Difference in rotation between samples.
	auto dref = s1.ref.rot * s2.ref.rot.transpose();
	auto dtarget = s1.target.rot * s2.target.rot.transpose();

	// When stuck together, the two tracked objects rotate as a pair,
	// therefore their axes of rotation must be equal between any given pair of samples.
	DSample ds;
	ds.ref = RotationVector(dref);
	ds.target = RotationVector(dtarget);

	// Reject samples that were too close to each other.
	auto refA = AngleFromRotationMatrix3(dref);
	auto targetA = AngleFromRotationMatrix3(dtarget);
	ds.valid = refA > 0.4 && targetA > 0.4 && ds.ref.norm() > 0.01 && ds.target.norm() > 0.01;

	return ds;
}

Eigen::Vector3d CalibrateRotation(const std::vector<Sample>& samples)
{
	std::vector<DSample> deltas;

	for (size_t i = 0; i < samples.size(); i++)
	{
		for (size_t j = 0; j < i; j++)
		{
			auto delta = DeltaRotationSamples(samples[i], samples[j]);
			if (delta.valid)
				deltas.push_back(delta);
		}
	}
	char buf[256];
	snprintf(buf, sizeof buf, "Got %zd samples with %zd delta samples\n", samples.size(), deltas.size());
	CalCtx.Log(buf);

	// With no usable deltas the cross-covariance is the zero matrix and the SVD below
	// yields an arbitrary (identity-like) rotation that carries no information. Signal
	// it rather than returning a confident-looking result. The NaN propagates through
	// the solve to the residual, which the `!(rmsError <= 0.1)` gate rejects.
	if (deltas.empty())
	{
		CalCtx.Log("No usable rotation deltas - head movement was too small to solve rotation.\n");
		return Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(),
			std::numeric_limits<double>::quiet_NaN(),
			std::numeric_limits<double>::quiet_NaN());
	}

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
	{
		i(2, 2) = -1;
	}

	Eigen::Matrix3d rot = svd.matrixV() * i * svd.matrixU().transpose();
	rot.transposeInPlace();

	Eigen::Vector3d euler = rot.eulerAngles(2, 1, 0) * 180.0 / EIGEN_PI;

	snprintf(buf, sizeof buf, "Calibrated rotation: yaw=%.2f pitch=%.2f roll=%.2f\n", euler[1], euler[2], euler[0]);
	CalCtx.Log(buf);
	return euler;
}

// Minimum positional spread (RMS, metres) required to fit headset scale at all.
// Empirically this is the reliable discriminator: logged calibrations at spread
// 0.235-0.286 m all landed within 0.5% of each other, while one at 0.160 m fitted
// a scale ~3% off. Matches the live coach's threshold, so "cover more space" and
// "scale accepted" agree. Below this, scale falls back to exactly 1.
static const double ScaleSpreadThreshold = 0.20;
static const double MinCalibratedScale = 0.9;
static const double MaxCalibratedScale = 1.1;
// Secondary guard for the case where spread is adequate but residuals are noisy.
// Note: the formal standard error only captures random scatter, not the
// ill-conditioning bias that dominates at low spread -- it reported 0.2% on the
// 0.160 m fit that was actually ~3% wrong -- which is why the spread gate above,
// not this, is the primary check.
static const double MaxScaleStdErr = 0.005;

Eigen::Vector3d CalibrateTranslation(const std::vector<Sample>& samples, const Eigen::Matrix3d& rotation, double scale)
{
	std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> deltas;

	for (size_t i = 0; i < samples.size(); i++)
	{
		Sample s_i = samples[i];
		s_i.target.rot = rotation * s_i.target.rot;
		s_i.target.trans = scale * (rotation * s_i.target.trans);

		for (size_t j = 0; j < i; j++)
		{
			Sample s_j = samples[j];
			s_j.target.rot = rotation * s_j.target.rot;
			s_j.target.trans = scale * (rotation * s_j.target.trans);

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
	auto transcm = trans * 100.0;

	char buf[256];
	snprintf(buf, sizeof buf, "Calibrated translation x=%.2f y=%.2f z=%.2f\n", transcm[0], transcm[1], transcm[2]);
	CalCtx.Log(buf);
	return transcm;
}

// Positional spread of the last scale estimate, recorded for the calibration log:
// it is the quantity that determines whether scale was observable at all.
static double g_lastScaleSpread = 0.0;

static double EstimateHmdSpaceScale(const std::vector<Sample> &samples, const Eigen::Matrix3d &rotation, double targetModelScale)
{
	Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
	for (auto &sample : samples)
		centroid += rotation * sample.target.trans;
	centroid /= (double)samples.size();

	double spread = 0;
	for (auto &sample : samples)
		spread += (rotation * sample.target.trans - centroid).squaredNorm();
	spread = std::sqrt(spread / (double)samples.size());
	g_lastScaleSpread = spread;

	char buf[256];
	if (spread < ScaleSpreadThreshold)
	{
		snprintf(buf, sizeof buf,
			"Not enough positional movement to measure headset scale (spread %.2f m, need >= %.2f m), assuming 1.\n"
			"Walk and crouch to cover more of your play space while calibrating; head rotation alone cannot determine scale.\n",
			spread, ScaleSpreadThreshold);
		CalCtx.Log(buf);
		return 1.0;
	}

	Eigen::MatrixXd coefficients(samples.size() * 3, 7);
	Eigen::VectorXd constants(samples.size() * 3);

	for (size_t i = 0; i < samples.size(); i++)
	{
		Eigen::Vector3d rotatedPos = rotation * samples[i].target.trans;
		Eigen::Matrix3d rotatedRot = rotation * samples[i].target.rot;

		coefficients.block<3, 1>(i * 3, 0) = rotatedPos;
		coefficients.block<3, 3>(i * 3, 1) = Eigen::Matrix3d::Identity();
		coefficients.block<3, 3>(i * 3, 4) = rotatedRot;
		constants.segment<3>(i * 3) = samples[i].ref.trans;
	}

	Eigen::BDCSVD<Eigen::MatrixXd> svd = coefficients.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
	Eigen::VectorXd result = svd.solve(constants);
	double fittedScale = result(0);

	// Observability check. The scale multiplies the tracker position while params 1-3
	// are a free translation, so a cluster of samples in a small volume makes the two
	// nearly degenerate - only the *spread* carries scale information. Head rotation
	// alone moves a head-mounted tracker in a ~10-20cm arc, which clears the spread
	// guard above while leaving the fit badly conditioned: with ~4mm sample noise the
	// scale is then uncertain by a few percent, and repeat calibrations disagree by
	// more than the quantity being measured. Estimate the standard error from the SVD
	// and refuse the fit when it is not actually determined by the data.
	const Eigen::VectorXd sv = svd.singularValues();
	const int nObs = (int)(samples.size() * 3);
	const int nParams = 7;
	double scaleStdErr = std::numeric_limits<double>::infinity();
	if (nObs > nParams && sv.size() > 0 && sv(0) > 0.0)
	{
		const double resid2 = (coefficients * result - constants).squaredNorm();
		const double sigma2 = resid2 / (double)(nObs - nParams);
		// (A^T A)^-1 [0,0] via the thin SVD: sum_j V(0,j)^2 / s_j^2
		double cov00 = 0.0;
		const double tol = sv(0) * 1e-9;
		for (int j = 0; j < sv.size(); j++)
			if (sv(j) > tol)
				cov00 += (svd.matrixV()(0, j) * svd.matrixV()(0, j)) / (sv(j) * sv(j));
		if (cov00 > 0.0 && sigma2 >= 0.0)
			scaleStdErr = std::sqrt(sigma2 * cov00);
	}

	if (!(scaleStdErr <= MaxScaleStdErr))
	{
		snprintf(buf, sizeof buf,
			"Headset scale is not determined by this calibration (fit %.5f +/- %.2f%%, positional spread %.2f m) - assuming 1.\n"
			"To measure scale, walk around and cover more of your play space while calibrating; head rotation alone is not enough.\n",
			fittedScale, scaleStdErr * 100.0, spread);
		CalCtx.Log(buf);
		return 1.0;
	}

	if (fittedScale < MinCalibratedScale || fittedScale > MaxCalibratedScale)
	{
		snprintf(buf, sizeof buf, "Fitted space scale %.5f is not plausible, assuming headset scale 1\n", fittedScale);
		CalCtx.Log(buf);
		return 1.0;
	}

	snprintf(buf, sizeof buf, "Fitted headset space scale relative to lighthouse: %.5f (%+.2f%%, +/- %.2f%%), implied absolute headset scale: %.5f\n",
		fittedScale, (fittedScale - 1.0) * 100.0, scaleStdErr * 100.0, fittedScale / targetModelScale);
	CalCtx.Log(buf);
	return fittedScale;
}

static const double AxisVarianceThreshold = 0.0005;

static double SecondAxisVariance(const std::vector<Sample> &samples)
{
	std::vector<Eigen::Vector4d> points;
	points.reserve(samples.size());
	Eigen::Vector4d mean = Eigen::Vector4d::Zero();

	for (auto &sample : samples)
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
	for (auto &point : points)
	{
		Eigen::Vector4d d = point - mean;
		cov += d * d.transpose();
	}
	cov /= (double)points.size();

	Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver(cov);
	return solver.eigenvalues()(1);
}

static Eigen::Vector3d ComputeRefToTargetOffset(const std::vector<Sample> &samples, const Eigen::Matrix3d &calRot, const Eigen::Vector3d &calTrans, double calScale)
{
	Eigen::Vector3d accum = Eigen::Vector3d::Zero();

	for (auto &sample : samples)
		accum += sample.ref.rot.transpose() * (calScale * (calRot * sample.target.trans) + calTrans - sample.ref.trans);

	return accum / (double)samples.size();
}

static double RetargetingErrorRMS(const std::vector<Sample> &samples, const Eigen::Vector3d &hmdToTargetPos, const Eigen::Matrix3d &calRot, const Eigen::Vector3d &calTrans, double calScale)
{
	double accum = 0;

	for (auto &sample : samples)
		accum += (calScale * (calRot * sample.target.trans) + calTrans - (sample.ref.rot * hmdToTargetPos + sample.ref.trans)).squaredNorm();

	return std::sqrt(accum / (double)samples.size());
}

Sample CollectSample(const CalibrationContext &ctx)
{
	vr::TrackedDevicePose_t reference, target;
	reference.bPoseIsValid = false;
	target.bPoseIsValid = false;

	reference = ctx.devicePoses[0];
	target = ctx.devicePoses[ctx.targetID];

	bool ok = true;
	if (!reference.bPoseIsValid)
	{
		CalCtx.Log("Reference device is not tracking\n"); ok = false;
	}
	if (!target.bPoseIsValid)
	{
		CalCtx.Log("Target device is not tracking\n"); ok = false;
	}
	if (!ok)
	{
		CalCtx.Log("Aborting calibration!\n");
		CalCtx.state = CalibrationState::None;
		return Sample();
	}

	return Sample(
		Pose(reference.mDeviceToAbsoluteTracking),
		Pose(target.mDeviceToAbsoluteTracking)
	);
}

vr::HmdQuaternion_t VRRotationQuat(Eigen::Vector3d eulerdeg)
{
	auto euler = eulerdeg * EIGEN_PI / 180.0;

	Eigen::Quaterniond rotQuat =
		Eigen::AngleAxisd(euler(0), Eigen::Vector3d::UnitZ()) *
		Eigen::AngleAxisd(euler(1), Eigen::Vector3d::UnitY()) *
		Eigen::AngleAxisd(euler(2), Eigen::Vector3d::UnitX());

	vr::HmdQuaternion_t vrRotQuat;
	vrRotQuat.x = rotQuat.coeffs()[0];
	vrRotQuat.y = rotQuat.coeffs()[1];
	vrRotQuat.z = rotQuat.coeffs()[2];
	vrRotQuat.w = rotQuat.coeffs()[3];
	return vrRotQuat;
}

vr::HmdVector3d_t VRTranslationVec(Eigen::Vector3d transcm)
{
	auto trans = transcm * 0.01;
	vr::HmdVector3d_t vrTrans;
	vrTrans.v[0] = trans[0];
	vrTrans.v[1] = trans[1];
	vrTrans.v[2] = trans[2];
	return vrTrans;
}

static void ApplyDeviceTransform(uint32_t id, bool enabled, const vr::HmdVector3d_t &translation,
	const vr::HmdQuaternion_t &rotation, double scale)
{
	if (id >= vr::k_unMaxTrackedDeviceCount)
		return;

	auto &prev = g_appliedTf[id];
	if (prev.known
		&& prev.enabled == enabled
		&& (!enabled || (VecEq(prev.translation, translation) && QuatEq(prev.rotation, rotation) && ApproxEq(prev.scale, scale))))
	{
		return;
	}

	protocol::Request req(protocol::RequestSetDeviceTransform);
	if (enabled)
		req.setDeviceTransform = { id, true, translation, rotation, scale };
	else
	{
		vr::HmdVector3d_t zeroV{ 0, 0, 0 };
		vr::HmdQuaternion_t zeroQ{ 1, 0, 0, 0 };
		req.setDeviceTransform = { id, false, zeroV, zeroQ, 1.0 };
	}
	Driver.SendBlocking(req);

	prev.known = true;
	prev.enabled = enabled;
	prev.translation = translation;
	prev.rotation = rotation;
	prev.scale = scale;
}

void ResetAndDisableOffsets(uint32_t id)
{
	vr::HmdVector3d_t zeroV{ 0, 0, 0 };
	vr::HmdQuaternion_t zeroQ{ 1, 0, 0, 0 };
	ApplyDeviceTransform(id, false, zeroV, zeroQ, 1.0);
}

void SendOneEuroParams()
{
	protocol::Request req(protocol::RequestSetOneEuro);
	req.setOneEuro.headEnabled = CalCtx.headFilterEnabled;
	req.setOneEuro.head = CalCtx.headFilterParams;
	req.setOneEuro.drift = CalCtx.driftFilterParams;

	if (g_appliedOneEuroKnown
		&& g_appliedOneEuro.headEnabled == req.setOneEuro.headEnabled
		&& OneEuroEq(g_appliedOneEuro.head, req.setOneEuro.head)
		&& OneEuroEq(g_appliedOneEuro.drift, req.setOneEuro.drift))
	{
		return;
	}

	try
	{
		Driver.SendBlocking(req);
		g_appliedOneEuro = req.setOneEuro;
		g_appliedOneEuroKnown = true;
	}
	catch (const std::runtime_error &e)
	{
		std::cerr << "Failed to send One Euro params: " << e.what() << std::endl;
	}
}

void SendHmdTrackerCommand(uint32_t hmdID, uint32_t trackerID, bool enabled)
{
	protocol::Request req(protocol::RequestSetHmdTracker);
	req.setHmdTracker.hmdID = hmdID;
	req.setHmdTracker.trackerID = trackerID;
	req.setHmdTracker.enabled = enabled;
	req.setHmdTracker.native = CalCtx.enableNative;
	req.setHmdTracker.slamFallback = CalCtx.fallbackToSlam;
	req.setHmdTracker.predictionTime = CalCtx.predictionTime;
	req.setHmdTracker.enableAngularVelocity = CalCtx.enableAngularVelocity;
	req.setHmdTracker.offsetRotation = CalCtx.relativeRotation;
	req.setHmdTracker.offsetTranslation = CalCtx.relativeTranslation;
	req.setHmdTracker.calibrationRotation = VRRotationQuat(CalCtx.calibratedRotation);
	req.setHmdTracker.calibrationTranslation = VRTranslationVec(CalCtx.calibratedTranslation);
	req.setHmdTracker.calibrationScale = CalCtx.calibratedScale;
	req.setHmdTracker.hmdScale = CalCtx.hmdScale;

	if (g_appliedHmdKnown && HmdTrackerEq(g_appliedHmd, req.setHmdTracker))
		return;

	Driver.SendBlocking(req);
	g_appliedHmd = req.setHmdTracker;
	g_appliedHmdKnown = true;
}

// https://stackoverflow.com/questions/12374087/average-of-multiple-quaternions/27410865
void ComputeRelativeOffset(CalibrationContext &ctx, const std::vector<Sample> &samples, const Eigen::Matrix3d &calRot, const Eigen::Vector3d &calTrans, double calScale)
{
	if (samples.empty())
		return;

	Eigen::Matrix4d quatAccum = Eigen::Matrix4d::Zero();
	Eigen::Vector3d transAccum = Eigen::Vector3d::Zero();

	for (auto &sample : samples)
	{
		Eigen::Matrix3d trackerRot = calRot * sample.target.rot;
		Eigen::Vector3d trackerTrans = calScale * (calRot * sample.target.trans) + calTrans;

		Eigen::Matrix3d offsetRot = trackerRot.transpose() * sample.ref.rot;
		Eigen::Vector3d offsetTrans = trackerRot.transpose() * (sample.ref.trans - trackerTrans);

		Eigen::Quaterniond q(offsetRot);
		Eigen::Vector4d v(q.w(), q.x(), q.y(), q.z());
		quatAccum += v * v.transpose();
		transAccum += offsetTrans;
	}

	Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver(quatAccum);
	Eigen::Vector4d avg = solver.eigenvectors().col(3).normalized();

	Eigen::Quaterniond q(avg(0), avg(1), avg(2), avg(3));
	q.normalize();
	if (q.w() < 0)
		q.coeffs() = -q.coeffs();

	transAccum /= (double)samples.size();

	ctx.relativeRotation.w = q.w();
	ctx.relativeRotation.x = q.x();
	ctx.relativeRotation.y = q.y();
	ctx.relativeRotation.z = q.z();
	ctx.relativeTranslation.v[0] = transAccum.x();
	ctx.relativeTranslation.v[1] = transAccum.y();
	ctx.relativeTranslation.v[2] = transAccum.z();
	ctx.validRelativeOffset = true;
}

static_assert(vr::k_unTrackedDeviceIndex_Hmd == 0, "HMD index expected to be 0");

void ScanAndApplyProfile(CalibrationContext &ctx)
{
	char buffer[vr::k_unMaxPropertyStringSize];
	ctx.enabled = ctx.validProfile;

	if (ctx.enabled)
	{
		ctx.targetID = vr::k_unTrackedDeviceIndexInvalid;
		if (!ctx.trackerSerial.empty())
		{
			for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
			{
				if (vr::VRSystem()->GetTrackedDeviceClass(id) == vr::TrackedDeviceClass_Invalid)
					continue;
				if (GetDeviceSerial(id) == ctx.trackerSerial)
				{
					ctx.targetID = id;
					break;
				}
			}
		}
	}

	// Write the desired transform once per device. Do not disable-all then re-enable:
	// that races the pose thread and briefly publishes uncalibrated poses (far jumps).
	for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
	{
		auto deviceClass = vr::VRSystem()->GetTrackedDeviceClass(id);
		if (deviceClass == vr::TrackedDeviceClass_Invalid)
			continue;

		// HMD pose is replaced from the head tracker; never space-warp it.
		if (id == vr::k_unTrackedDeviceIndex_Hmd)
		{
			ResetAndDisableOffsets(id);
			continue;
		}

		if (!ctx.enabled)
		{
			ResetAndDisableOffsets(id);
			continue;
		}

		vr::ETrackedPropertyError err = vr::TrackedProp_Success;
		vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_TrackingSystemName_String, buffer, vr::k_unMaxPropertyStringSize, &err);

		if (err != vr::TrackedProp_Success)
		{
			ResetAndDisableOffsets(id);
			continue;
		}

		std::string trackingSystem(buffer);

		// Head-mounted lighthouse tracker is the HMD pose source; keep its raw pose.
		if (deviceClass == vr::TrackedDeviceClass_GenericTracker && trackingSystem == ctx.targetTrackingSystem && id == ctx.targetID)
		{
			ResetAndDisableOffsets(id);
			continue;
		}

		if (trackingSystem == ctx.targetTrackingSystem) {
			double deviceScale = ctx.calibratedScale * GetLighthouseModelScale(id) / ctx.targetModelScale;
			ApplyDeviceTransform(
				id,
				true,
				VRTranslationVec(ctx.calibratedTranslation),
				VRRotationQuat(ctx.calibratedRotation),
				deviceScale);
		}
		else
		{
			ResetAndDisableOffsets(id);
		}
	}

	bool overrideActive = ctx.enabled && ctx.validRelativeOffset && ctx.targetID != vr::k_unTrackedDeviceIndexInvalid;

	for (uint32_t id = 0; overrideActive && id < vr::k_unMaxTrackedDeviceCount; ++id)
	{
		auto deviceClass = vr::VRSystem()->GetTrackedDeviceClass(id);
		if (deviceClass == vr::TrackedDeviceClass_Invalid)
			continue;

		bool sync = ctx.continuousSync
			&& id != vr::k_unTrackedDeviceIndex_Hmd
			&& deviceClass != vr::TrackedDeviceClass_TrackingReference;

		if (sync)
		{
			vr::ETrackedPropertyError err = vr::TrackedProp_Success;
			vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_TrackingSystemName_String, buffer, vr::k_unMaxPropertyStringSize, &err);
			sync = err == vr::TrackedProp_Success && std::string(buffer) != ctx.targetTrackingSystem;
		}

		if (g_appliedSlamKnown[id] && g_appliedSlam[id] == sync)
			continue;

		protocol::Request req(protocol::RequestSetSlamSync);
		req.setSlamSync = { id, sync };
		Driver.SendBlocking(req);
		g_appliedSlamKnown[id] = true;
		g_appliedSlam[id] = sync;
	}

	if (overrideActive)
	{
		SendHmdTrackerCommand(vr::k_unTrackedDeviceIndex_Hmd, ctx.targetID, true);
	}
	else
	{
		SendHmdTrackerCommand(vr::k_unTrackedDeviceIndex_Hmd, vr::k_unTrackedDeviceIndexInvalid, false);
	}

	SendOneEuroParams();

	if (ctx.enabled && ctx.chaperone.valid && ctx.chaperone.autoApply)
	{
		uint32_t quadCount = 0;
		vr::VRChaperoneSetup()->GetLiveCollisionBoundsInfo(nullptr, &quadCount);

		// Heuristic: when SteamVR resets to a blank-ish chaperone, it uses empty geometry,
		// but manual adjustments (e.g. via a play space mover) will not touch geometry.
		if (quadCount != ctx.chaperone.geometry.size())
		{
			ApplyChaperoneBounds();
		}
	}
}

// V3-e: sample quality gate state (HMD angular and linear speed between sampling ticks).
static Eigen::Matrix3d g_prevSampleHmdRot;
static Eigen::Vector3d g_prevSampleHmdPos = Eigen::Vector3d::Zero();
static double g_prevSampleTime = 0;
static bool g_havePrevSample = false;

// Live calibration guidance state.
static double g_lastHintTime = 0;
static double g_lastFastDropTime = 0;

// Sampling diagnostics: how many candidate samples the speed gates rejected, split
// by cause. Written to calibration.log so "the gates are too strict / the data is
// sparse" is a measurable claim rather than a guess.
static int g_rejAngCount = 0;
static int g_rejLinCount = 0;

// Distinct 10cm cells occupied by the accepted sample positions. Many samples in
// few cells means the coverage is clumped even when the sample count is high -
// exactly the failure mode a spread number alone hides.
static int CoverageCells(const std::vector<Sample> &samples)
{
	std::set<long long> cells;
	for (auto &s : samples)
	{
		long long cx = (long long)std::floor(s.target.trans.x() * 10.0);
		long long cy = (long long)std::floor(s.target.trans.y() * 10.0);
		long long cz = (long long)std::floor(s.target.trans.z() * 10.0);
		cells.insert((cx * 73856093LL) ^ (cy * 19349663LL) ^ (cz * 83492791LL));
	}
	return (int)cells.size();
}

// RMS spread of the target (head tracker) positions collected so far. Rotation is
// distance-preserving, so this equals the spread used later for the scale fit, and
// it is what tells us whether the user has actually translated through space (the
// only motion that makes headset scale observable) vs merely rotated in place.
static double TargetSpread(const std::vector<Sample> &samples)
{
	if (samples.size() < 2)
		return 0.0;
	Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
	for (auto &s : samples)
		centroid += s.target.trans;
	centroid /= (double)samples.size();
	double var = 0.0;
	for (auto &s : samples)
		var += (s.target.trans - centroid).squaredNorm();
	return std::sqrt(var / (double)samples.size());
}

static void BeginSamplingPhase(CalibrationContext &ctx, uint32_t targetID)
{
	ctx.targetID = targetID;
	ctx.targetTrackingSystem = GetDeviceTrackingSystem(targetID);
	ctx.hmdSerial = GetDeviceSerial(vr::k_unTrackedDeviceIndex_Hmd);
	ctx.trackerSerial = GetDeviceSerial(targetID);

	char buf[256];
	snprintf(buf, sizeof buf, "Using headset tracker: %s (id %d)\n", ctx.trackerSerial.c_str(), targetID);
	ctx.Log(buf);

	ResetAndDisableOffsets(targetID);
	SendHmdTrackerCommand(vr::k_unTrackedDeviceIndex_Hmd, vr::k_unTrackedDeviceIndexInvalid, false);

	ctx.state = CalibrationState::Sampling;
	ctx.wantedUpdateInterval = 0.0;
	g_havePrevSample = false;
	g_lastHintTime = 0;
	g_lastFastDropTime = 0;
	g_rejAngCount = 0;
	g_rejLinCount = 0;
	ctx.sampleHint = "Move your head slowly through different angles";
	ctx.sampleHintLevel = 1;
	ctx.Log("Starting calibration...\n");
}

static std::vector<Sample> collectedSamples;
static int coplanarRetries = 0;

void StartCalibration()
{
	CalCtx.state = CalibrationState::Begin;
	CalCtx.wantedUpdateInterval = 0.0;
	CalCtx.messages.clear();
	Detection.Clear();
	collectedSamples.clear();
	coplanarRetries = 0;
	g_havePrevSample = false;
}

static void AbortAndRestoreProfile(CalibrationContext &ctx)
{
	if (ctx.targetID != vr::k_unTrackedDeviceIndexInvalid)
		ResetAndDisableOffsets(ctx.targetID);

	LoadProfile(ctx);
	ctx.state = CalibrationState::None;
	collectedSamples.clear();
	coplanarRetries = 0;
}

void CalibrationTick(double time)
{
	if (!vr::VRSystem())
		return;

	auto &ctx = CalCtx;
	if ((time - ctx.timeLastTick) < 0.05)
		return;

	ctx.timeLastTick = time;
	vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseRawAndUncalibrated, 0.0f, ctx.devicePoses, vr::k_unMaxTrackedDeviceCount);

	if (ctx.state == CalibrationState::None)
	{
		ctx.wantedUpdateInterval = 1.0;

		if ((time - ctx.timeLastScan) >= 1.0)
		{
			ScanAndApplyProfile(ctx);
			ctx.timeLastScan = time;
		}
		return;
	}

	if (ctx.state == CalibrationState::Editing)
	{
		ctx.wantedUpdateInterval = 0.1;

		if ((time - ctx.timeLastScan) >= 0.1)
		{
			ScanAndApplyProfile(ctx);
			ctx.timeLastScan = time;
		}
		return;
	}

	if (ctx.state == CalibrationState::Begin)
	{
		SendHmdTrackerCommand(vr::k_unTrackedDeviceIndex_Hmd, vr::k_unTrackedDeviceIndexInvalid, false);

		if (vr::VRSystem()->GetTrackedDeviceClass(vr::k_unTrackedDeviceIndex_Hmd) != vr::TrackedDeviceClass_HMD ||
			!ctx.devicePoses[vr::k_unTrackedDeviceIndex_Hmd].bPoseIsValid)
		{
			ctx.state = CalibrationState::None;
			CalCtx.Log("No tracking HMD found, aborting calibration!\n");
			return;
		}

		std::string hmdSystem = GetDeviceTrackingSystem(vr::k_unTrackedDeviceIndex_Hmd);

		Detection.Clear();
		for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
		{
			if (vr::VRSystem()->GetTrackedDeviceClass(id) != vr::TrackedDeviceClass_GenericTracker)
				continue;
			if (!ctx.devicePoses[id].bPoseIsValid)
				continue;

			Detection.candidates.push_back(id);
		}

		if (Detection.candidates.empty())
		{
			ctx.state = CalibrationState::None;
			CalCtx.Log("No trackers from a different tracking system detected, aborting!\n");
			return;
		}

		if (Detection.candidates.size() == 1)
		{
			ctx.targetID = Detection.candidates[0];
			BeginSamplingPhase(ctx, Detection.candidates[0]);
			return;
		}

		Detection.candidateSpeeds.resize(Detection.candidates.size());
		CalCtx.Log("Move your head around to identify the headset tracker...\n");
		ctx.state = CalibrationState::Detect;
		ctx.wantedUpdateInterval = 0.0;
		return;
	}

	if (ctx.state == CalibrationState::Detect)
	{
		if (!ctx.devicePoses[vr::k_unTrackedDeviceIndex_Hmd].bPoseIsValid)
			return;

		Eigen::Matrix3d hmdRot = Pose(ctx.devicePoses[vr::k_unTrackedDeviceIndex_Hmd].mDeviceToAbsoluteTracking).rot;

		std::vector<Eigen::Matrix3d> curRot(Detection.candidates.size());
		for (size_t i = 0; i < Detection.candidates.size(); i++)
			curRot[i] = Pose(ctx.devicePoses[Detection.candidates[i]].mDeviceToAbsoluteTracking).rot;

		double dt = time - Detection.prevTime;
		if (Detection.havePrev && dt > 1e-4)
		{
			Detection.hmdSpeeds.push_back(AngularSpeedBetween(hmdRot, Detection.prevRot[0], dt));
			for (size_t i = 0; i < Detection.candidates.size(); i++)
				Detection.candidateSpeeds[i].push_back(AngularSpeedBetween(curRot[i], Detection.prevRot[i + 1], dt));

			CalCtx.Progress((int) Detection.hmdSpeeds.size(), 40);
		}

		Detection.prevRot.assign(1, hmdRot);
		Detection.prevRot.insert(Detection.prevRot.end(), curRot.begin(), curRot.end());
		Detection.prevTime = time;
		Detection.havePrev = true;

		if ((int) Detection.hmdSpeeds.size() < 40)
			return;

		double hmdPeak = 0;
		for (double s : Detection.hmdSpeeds)
			hmdPeak = max(hmdPeak, s);

		if (hmdPeak < 0.5)
		{
			Detection.Clear();
			ctx.state = CalibrationState::None;
			CalCtx.Log("Didn't detect enough head movement, aborting! Try again and move your head more.\n");
			return;
		}

		double bestCorr = -2, secondCorr = -2;
		int bestIdx = -1;
		for (size_t i = 0; i < Detection.candidates.size(); i++)
		{
			double corr = PearsonCorrelation(Detection.hmdSpeeds, Detection.candidateSpeeds[i]);
			if (corr > bestCorr)
			{
				secondCorr = bestCorr;
				bestCorr = corr;
				bestIdx = (int) i;
			}
			else if (corr > secondCorr)
			{
				secondCorr = corr;
			}
		}

		if (bestIdx == -1 || bestCorr < 0.7 || (bestCorr - secondCorr) < 0.1)
		{
			Detection.Clear();
			ctx.state = CalibrationState::None;
			CalCtx.Log("Couldn't clearly identify the headset tracker, aborting! Make sure only the headset tracker moves with your head, then try again.\n");
			return;
		}

		uint32_t targetID = Detection.candidates[bestIdx];
		Detection.Clear();
		BeginSamplingPhase(ctx, targetID);
		return;
	}

	// V3-e: drop samples taken while the head is moving fast. The HMD pose arrives over
	// a wireless link and the tracker pose over lighthouse, with different latency; a
	// sample pair captured mid-motion pairs two instants that are not the same instant,
	// and that skew is baked into the solved offset. Gate BOTH rotation and translation:
	// covering play space for scale observability requires walking, which is exactly the
	// linear motion that injects skew, so position must be sampled at the pauses.
	if (ctx.devicePoses[vr::k_unTrackedDeviceIndex_Hmd].bPoseIsValid)
	{
		const double kMaxSampleAngSpeed = 1.5;  // rad/s
		const double kMaxSampleLinSpeed = 0.25; // m/s; at ~4ms skew this is <1mm of error
		Pose hmdPose(ctx.devicePoses[vr::k_unTrackedDeviceIndex_Hmd].mDeviceToAbsoluteTracking);
		double dt = time - g_prevSampleTime;

		bool tooFastAng = false, tooFastLin = false;
		if (g_havePrevSample && dt > 1e-4)
		{
			tooFastAng = AngularSpeedBetween(hmdPose.rot, g_prevSampleHmdRot, dt) > kMaxSampleAngSpeed;
			tooFastLin = ((hmdPose.trans - g_prevSampleHmdPos).norm() / dt) > kMaxSampleLinSpeed;
		}

		g_prevSampleHmdRot = hmdPose.rot;
		g_prevSampleHmdPos = hmdPose.trans;
		g_prevSampleTime = time;
		g_havePrevSample = true;

		if (tooFastAng || tooFastLin)
		{
			// Count rejections by cause so calibration.log can show whether the gates
			// are actually starving the solve of samples.
			if (tooFastLin) ++g_rejLinCount; else ++g_rejAngCount;
			// This is why the bar can stall with no explanation: samples are being
			// dropped. Tell the user, and hold the message briefly so it is readable.
			ctx.sampleHint = tooFastLin
				? "Hold still where you are - samples are only taken when you pause"
				: "Turn your head more slowly - samples are being skipped";
			ctx.sampleHintLevel = 2;
			g_lastFastDropTime = time;
			return;
		}
	}

	auto sample = CollectSample(ctx);
	if (!sample.valid)
	{
		return;
	}

	auto &samples = collectedSamples;
	samples.push_back(sample);

	CalCtx.Progress(samples.size(), CalCtx.SampleCount());

	// Live guidance: after each accepted sample, tell the user what the solver still
	// needs, so deficient motion is corrected during sampling instead of only being
	// reported after a full batch (which then discards a quarter and restarts).
	// Throttled, and it does not override a fresh "slow down" message.
	if ((time - g_lastHintTime) > 0.3 && (time - g_lastFastDropTime) > 0.8)
	{
		g_lastHintTime = time;
		const size_t target = CalCtx.SampleCount();
		if (samples.size() < target / 5)
		{
			ctx.sampleHint = "Move your head slowly through different angles";
			ctx.sampleHintLevel = 1;
		}
		else if (SecondAxisVariance(samples) < AxisVarianceThreshold * 2.0)
		{
			ctx.sampleHint = "Add rotation variety - tilt ear-to-shoulder and look up/down, not just left and right";
			ctx.sampleHintLevel = 1;
		}
		else if (TargetSpread(samples) < ScaleSpreadThreshold)
		{
			// Samples are only taken while you are still, so the motion that works is
			// move-then-pause rather than continuous walking.
			ctx.sampleHint = "Move to a different spot or height, pause, then look around - repeat to lock in scale";
			ctx.sampleHintLevel = 1;
		}
		else
		{
			ctx.sampleHint = "Looking good - keep moving smoothly until the bar fills";
			ctx.sampleHintLevel = 0;
		}
	}

	if (samples.size() >= CalCtx.SampleCount())
	{
		CalCtx.Log("\n");

		double axisVariance = SecondAxisVariance(samples);
		if (axisVariance < AxisVarianceThreshold)
		{
			if (++coplanarRetries >= 10)
			{
				CalCtx.Log("Not enough rotation variety after several attempts, aborting calibration! Previous calibration restored.\n");
				AbortAndRestoreProfile(ctx);
				return;
			}

			char buf[256];
			snprintf(buf, sizeof buf, "Head movement is too uniform (axis variance %.5f), tilt and turn your head in different directions! Collecting more samples...\n", axisVariance);
			CalCtx.Log(buf);
			samples.erase(samples.begin(), samples.begin() + samples.size() / 4);
			return;
		}
		coplanarRetries = 0;

		ctx.calibratedRotation = CalibrateRotation(samples);

		Eigen::Vector3d eulerRad = ctx.calibratedRotation * EIGEN_PI / 180.0;
		Eigen::Matrix3d calRot =
			(Eigen::AngleAxisd(eulerRad(0), Eigen::Vector3d::UnitZ()) *
			 Eigen::AngleAxisd(eulerRad(1), Eigen::Vector3d::UnitY()) *
			 Eigen::AngleAxisd(eulerRad(2), Eigen::Vector3d::UnitX())).toRotationMatrix();

		double calScale = 1.0;
		ctx.calibratedScale = calScale;
		ctx.targetModelScale = GetLighthouseModelScale(ctx.targetID);

		ctx.hmdScale = EstimateHmdSpaceScale(samples, calRot, ctx.targetModelScale);

		for (auto &sample : samples)
			sample.ref.trans /= ctx.hmdScale;

		ctx.calibratedTranslation = CalibrateTranslation(samples, calRot, calScale);
		Eigen::Vector3d calTransM = ctx.calibratedTranslation * 0.01;

		Eigen::Vector3d hmdToTarget = ComputeRefToTargetOffset(samples, calRot, calTransM, calScale);
		double rmsError = RetargetingErrorRMS(samples, hmdToTarget, calRot, calTransM, calScale);

		char buf2[256];
		snprintf(buf2, sizeof buf2, "Calibration residual error (RMS): %.1f mm\n", rmsError * 1000.0);
		CalCtx.Log(buf2);

		// Written as !(x <= limit) so a NaN residual fails the gate. `rmsError > 0.1`
		// is false for NaN, which would have let a degenerate solve through to
		// SaveProfile and then into the pose pipeline.
		if (!(rmsError <= 0.1))
		{
			CalCtx.Log("Calibration quality is too low, aborting! Previous calibration restored. Try again with a slower calibration speed, moving smoothly.\n");
			AbortAndRestoreProfile(ctx);
			return;
		}

		ComputeRelativeOffset(ctx, samples, calRot, calTransM, calScale);

		ctx.validProfile = true;
		SaveProfile(ctx);
		LogCalibrationResult(ctx, rmsError * 1000.0, g_lastScaleSpread,
			(int)samples.size(), g_rejAngCount, g_rejLinCount, CoverageCells(samples));
		CalCtx.Log("Finished calibration, profile saved\n");

		if (CalCtx.notificationId != 0) {
			vr::VRNotifications()->RemoveNotification(CalCtx.notificationId);
			CalCtx.notificationId = 0;
		}


		ctx.state = CalibrationState::None;
		samples.clear();
	}
}

void LoadChaperoneBounds()
{
	vr::VRChaperoneSetup()->RevertWorkingCopy();

	uint32_t quadCount = 0;
	vr::VRChaperoneSetup()->GetLiveCollisionBoundsInfo(nullptr, &quadCount);

	CalCtx.chaperone.geometry.resize(quadCount);
	vr::VRChaperoneSetup()->GetLiveCollisionBoundsInfo(&CalCtx.chaperone.geometry[0], &quadCount);
	vr::VRChaperoneSetup()->GetWorkingStandingZeroPoseToRawTrackingPose(&CalCtx.chaperone.standingCenter);
	vr::VRChaperoneSetup()->GetWorkingPlayAreaSize(&CalCtx.chaperone.playSpaceSize.v[0], &CalCtx.chaperone.playSpaceSize.v[1]);
	CalCtx.chaperone.valid = true;
}

void ApplyChaperoneBounds()
{
	vr::VRChaperoneSetup()->RevertWorkingCopy();
	vr::VRChaperoneSetup()->SetWorkingCollisionBoundsInfo(&CalCtx.chaperone.geometry[0], CalCtx.chaperone.geometry.size());
	vr::VRChaperoneSetup()->SetWorkingStandingZeroPoseToRawTrackingPose(&CalCtx.chaperone.standingCenter);
	vr::VRChaperoneSetup()->SetWorkingPlayAreaSize(CalCtx.chaperone.playSpaceSize.v[0], CalCtx.chaperone.playSpaceSize.v[1]);
	vr::VRChaperoneSetup()->CommitWorkingCopy(vr::EChaperoneConfigFile_Live);
}
