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
// Solver internals recorded for the calibration log. These are statics rather than
// parameters because they are produced deep inside the solve and are only ever read
// back out here; threading them through every call site would add noise without
// adding information.
//
// g_lastScaleSource is the single most useful field: it distinguishes a scale that
// was actually measured from one that was carried over or defaulted, which the value
// alone cannot show (a kept 0.996 and a measured 0.996 look identical).
static double g_lastScaleStdErr = -1.0;
// N3-c: the cluster-corrected companion to g_lastScaleStdErr. The naive OLS error treats
// every sample as independent; samples arrive in bursts at a station, so it is optimistic.
// This is the honest number. -1 when not computed.
static double g_lastScaleStdErrEff = -1.0;
static const char *g_lastScaleSource = "unknown";
// Raw scale fitted by THIS run, before pooling (-1 when the run did not measure scale), and
// what the pooling did with it. Kept so run-to-run scatter stays visible in the log exactly the
// way lever_raw_mm keeps the lever arm's scatter visible -- an averaged number on its own hides
// the very quantity that justifies averaging it. Declared here with the other log fields
// because LogCalibrationResult reads them; PoolHmdScale below is what writes them.
static double g_lastScaleRaw = -1.0;
static const char *g_lastScalePoolAction = "none";
// How many observations the solvers actually saw, after station collapsing, and the independent
// pairwise-baseline scale estimate with the number of long baselines it was drawn from.
// scale_pair is logged only -- it is a candidate replacement for the SVD fit, accumulating
// evidence for a build before anything depends on it.
static int g_lastSolveN = 0;
static double g_lastScalePairwise = -1.0;
static int g_lastScalePairCount = 0;
// Declared here rather than beside StationTracker below: the scale solve needs the station
// partition to compute the effective sample size, and it runs earlier in this file.
static std::vector<int> g_sampleStation;   // parallel to the sample vector
static int g_stationCount = 0;
static double g_lastAxisVariance = 0.0;
static size_t g_lastSampleTarget = 0;
static bool g_lastEarlyFinish = false;
static double g_lastSplitHalfDeg = -1.0;
static int g_lastStationCount = 0;
// Tilt as solved, captured BEFORE gravity levelling zeroes it. Recomputing it from
// ctx.calibratedRotation at log time reports 0.00 forever, because by then the levelling
// has already discarded exactly the quantity being logged -- which silently destroyed
// this measurement across four calibrations. It is the only remaining term in the error
// budget with a wide range (0-29 mm), so it has to survive to disk.
static double g_lastTiltX = 0.0;
static double g_lastTiltZ = 0.0;
// Running average of the head-tracker lever arm across calibrations, plus this run's
// raw measurement so the log shows both and the scatter stays visible. The sample
// count itself lives in CalibrationContext::leverSamples so it persists and resets
// together with relativeTranslation instead of drifting out of sync with it.
static Eigen::Vector3d g_lastLever = Eigen::Vector3d::Zero();
static Eigen::Vector3d g_lastLeverRaw = Eigen::Vector3d::Zero();

// How far the calibration tips the vertical axis, in degrees. Both spaces are
// gravity-levelled, so the true relative rotation has yaw as its only free degree of
// freedom and this should be ~0; what it actually is, is tilt -- either solver noise
// or a genuinely un-level room setup, which repeated runs tell apart.
//
// Computed from the rotation matrix, not from the Euler angles: pitch/roll near zero
// and near +/-180 can describe the same physical rotation with a compensating yaw, so
// measuring "distance from the expected flip" per-angle reports ~179 degrees for a
// perfectly ordinary solve. Taking the absolute cosine makes it flip-agnostic.
static Eigen::Matrix3d EulerDegToMatrix(const Eigen::Vector3d &eulerDeg)
{
	const Eigen::Vector3d r = eulerDeg * EIGEN_PI / 180.0;
	return (Eigen::AngleAxisd(r(0), Eigen::Vector3d::UnitZ()) *
		Eigen::AngleAxisd(r(1), Eigen::Vector3d::UnitY()) *
		Eigen::AngleAxisd(r(2), Eigen::Vector3d::UnitX())).toRotationMatrix();
}

// Signed, two-component. An earlier version returned acos(|cos|) -- a magnitude, which
// is strictly non-negative and therefore positively biased: fed pure zero-mean noise it
// reports a healthy-looking average tilt forever and can never average to zero. Using
// it to decide "is there a real tilt here" would have answered yes no matter what.
// Signed components average to zero under noise and to the true tilt under a real one,
// which is the only way repeated runs can settle the question.
//
// Folds the +/-180 degree convention flip, since pitch/roll near zero and near 180 can
// describe the same physical rotation with a compensating yaw.
static void SignedTiltDeg(const Eigen::Vector3d &eulerDeg, double &tiltX, double &tiltZ)
{
	Eigen::Vector3d v = EulerDegToMatrix(eulerDeg) * Eigen::Vector3d::UnitY();
	if (v.y() < 0.0)
		v = -v;

	double z = v.z(), x = -v.x();
	if (z > 1.0) z = 1.0; else if (z < -1.0) z = -1.0;
	if (x > 1.0) x = 1.0; else if (x < -1.0) x = -1.0;

	tiltX = std::asin(z) * 180.0 / EIGEN_PI;
	tiltZ = std::asin(x) * 180.0 / EIGEN_PI;
}

// Shared by the success and failure records: resolve the log path, open for append, and write
// the timestamp prefix. Returns false when the log is unavailable, in which case the caller
// writes nothing rather than half a line.
static bool OpenCalibrationLog(std::ofstream &out)
{
	char localAppData[MAX_PATH] = {};
	DWORD n = GetEnvironmentVariableA("LOCALAPPDATA", localAppData, MAX_PATH);
	if (n == 0 || n >= MAX_PATH)
		return false;

	std::string dir = std::string(localAppData) + "\\OpenVR-SpaceOverride\\logs";
	CreateDirectoryA((std::string(localAppData) + "\\OpenVR-SpaceOverride").c_str(), nullptr);
	CreateDirectoryA(dir.c_str(), nullptr);

	out.open(dir + "\\calibration.log", std::ios::app);
	if (!out)
		return false;

	SYSTEMTIME st;
	GetLocalTime(&st);

	out << std::fixed
		<< st.wYear << "-" << std::setw(2) << std::setfill('0') << st.wMonth << "-"
		<< std::setw(2) << std::setfill('0') << st.wDay << " "
		<< std::setw(2) << std::setfill('0') << st.wHour << ":"
		<< std::setw(2) << std::setfill('0') << st.wMinute << ":"
		<< std::setw(2) << std::setfill('0') << st.wSecond << std::setfill(' ');
	return true;
}

// Records a calibration that did NOT produce a profile.
//
// calibration.log held successful solves only, so an abandoned or aborted run left no trace
// whatsoever. A user reporting "calibration keeps failing" therefore produced a log containing
// nothing but clean successes, and the failures had to be reconstructed from memory -- which is
// exactly backwards, since the runs that die are the ones worth a record. Coverage is passed in
// rather than read from file statics so this can be called from abort paths that run before,
// during or after sampling.
// Rigid-pair gate counters. Declared up here because the calibration.log writers below read
// them; the gate itself lives in CalibrationTick. See the block comment there for the
// measurement that set the thresholds.
static int g_rejRigidCount = 0;
static int g_rejStaleCount = 0;
static int g_frameBreakCount = 0;

// Raw-sample spread at finish time. The finish gate tests THIS quantity against
// ScaleSpreadThreshold, but the scale fit recomputes spread over the station-collapsed
// solve set and gates on that — so a run can pass the finish gate and still land in the
// kept_low_spread branch with no visible reason. Logged as spread_raw= beside spread_m=
// (which carries the solve-set value after a solve) so the divergence between the two
// gates is measurable before deciding whether to unify them (audit 2026-08-16,
// refuter-confirmed; conservative failure — scale kept, not corrupted — measurement first).
static double g_lastFinishSpread = 0.0;

static double RotAngleBetween(const Eigen::Matrix3d &a, const Eigen::Matrix3d &b);

// windows.h defines min/max as macros in this TU, so std::min does not compile here.
static double Vec3Norm(const vr::HmdVector3d_t &v)
{
	return std::sqrt(v.v[0] * v.v[0] + v.v[1] * v.v[1] + v.v[2] * v.v[2]);
}

static void LogCalibrationOutcome(const CalibrationContext &ctx, const char *result,
	const char *reason, int accepted, int stations, double spreadM, double axisVar,
	int rejAng, int rejLin, int droppedFrames)
{
	std::ofstream out;
	if (!OpenCalibrationLog(out))
		return;

	out << std::setprecision(3)
		<< "  result=" << result
		<< "  reason=" << reason
		<< "  accepted=" << accepted
		<< "  stations=" << stations
		<< "  target=" << g_lastSampleTarget
		<< "  spread_m=" << spreadM
		<< "  spread_raw=" << g_lastFinishSpread
		<< "  axis_var=" << std::setprecision(6) << axisVar
		<< "  rej_ang=" << rejAng
		<< "  rej_lin=" << rejLin
		// Rigid-pair gate. rej_stale counts bit-identical HMD poses (a stalled pose source),
		// frame_breaks counts SLAM relocalisations that forced a restart. A run with a
		// nonzero frame_breaks is one the old code would have solved as if nothing happened.
		<< "  rej_rigid=" << g_rejRigidCount
		<< "  rej_stale=" << g_rejStaleCount
		<< "  frame_breaks=" << g_frameBreakCount
		<< "  dropped=" << droppedFrames
		<< "  speed=" << (int)ctx.calibrationSpeed
		// The scale and lever the run was ABOUT to modify, so an abort can be told apart from
		// a run that never got far enough to threaten them.
		<< std::setprecision(5)
		<< "  hmdScale=" << ctx.hmdScale
		// The manual body-scale trim. Logged so the record shows it — its silent solve-time
		// reset (A12) was invisible precisely because no line carried it.
		<< "  trim=" << ctx.calibratedScale
		<< "  hmdscale_n=" << ctx.hmdScaleSamples
		<< "  hmd=" << ctx.hmdSerial
		<< "  tracker=" << ctx.trackerSerial
		<< "\n";
}

static void LogCalibrationResult(const CalibrationContext &ctx, double rmsErrorMm, double spreadM,
	int accepted, int rejAng, int rejLin, int coverageCells)
{
	// As solved, not as published: ctx.calibratedRotation has been levelled by now.
	const double tiltX = g_lastTiltX;
	const double tiltZ = g_lastTiltZ;

	std::ofstream out;
	if (!OpenCalibrationLog(out))
		return;

	out << std::setprecision(5)
		<< "  rms_mm=" << std::setprecision(1) << rmsErrorMm
		<< "  spread_m=" << std::setprecision(3) << spreadM
		<< "  spread_raw=" << g_lastFinishSpread
		<< "  hmdScale=" << std::setprecision(5) << ctx.hmdScale
		<< "  trim=" << ctx.calibratedScale
		<< "  targetModelScale=" << ctx.targetModelScale
		<< "  speed=" << (int)ctx.calibrationSpeed
		// Distinguishes a well-covered calibration from a clumped one: accepted samples
		// plus how many were rejected for moving too fast (angular vs linear), and how
		// many distinct 10cm cells the accepted samples occupy. Many samples in few
		// cells = clumped, which the RMS residual alone cannot reveal.
		<< "  accepted=" << accepted
		<< "  target=" << g_lastSampleTarget
		<< "  early=" << (g_lastEarlyFinish ? 1 : 0)
		<< "  rej_ang=" << rejAng
		<< "  rej_lin=" << rejLin
		// Rigid-pair gate. rej_stale counts bit-identical HMD poses (a stalled pose source),
		// frame_breaks counts SLAM relocalisations that forced a restart. A run with a
		// nonzero frame_breaks is one the old code would have solved as if nothing happened.
		<< "  rej_rigid=" << g_rejRigidCount
		<< "  rej_stale=" << g_rejStaleCount
		<< "  frame_breaks=" << g_frameBreakCount
		<< "  cells=" << coverageCells
		// Where hmdScale came from, and how well determined it was. "measured" is the
		// only value that reflects this run; "kept_*" means the run could not observe
		// scale and the previous measurement was carried over.
		<< "  scale_src=" << g_lastScaleSource
		<< "  scale_stderr_pct=" << std::setprecision(3)
		<< (g_lastScaleStdErr >= 0.0 ? g_lastScaleStdErr * 100.0 : -1.0)
		// scale_stderr_pct is the naive OLS error and is optimistic -- it is what the
		// MaxScaleStdErr gate still uses. scale_stderr_eff_pct applies the station design
		// effect sqrt(n/K) and is the number to believe (N3-c). Log both so the gate can
		// eventually be re-tuned against real data rather than a guess.
		<< "  scale_stderr_eff_pct=" << std::setprecision(3)
		<< (g_lastScaleStdErrEff >= 0.0 ? g_lastScaleStdErrEff * 100.0 : -1.0)
		// hmdScale above is the value that SHIPS, now a running average. These three make the
		// averaging auditable: the raw fit this run produced (-1 when the run did not measure
		// scale), how many runs are in the average, and what the pooling actually did. Without
		// the raw column the log would show a reassuringly stable hmdScale and hide the
		// per-solve scatter that is the entire reason for averaging it.
		<< "  hmdscale_raw=" << std::setprecision(5)
		<< (g_lastScaleRaw > 0.0 ? g_lastScaleRaw : -1.0)
		<< "  hmdscale_n=" << ctx.hmdScaleSamples
		<< "  scale_pool=" << g_lastScalePoolAction
		// Observations the solve actually ran on (station-collapsed), and the independent
		// pairwise-baseline scale with its long-baseline count. scale_pair is NOT applied; it is
		// here to be compared against hmdscale_raw over real runs.
		<< "  solve_n=" << g_lastSolveN
		<< "  scale_pair=" << (g_lastScalePairwise > 0.0 ? g_lastScalePairwise : -1.0)
		<< "  scale_pair_n=" << g_lastScalePairCount
		// Solved rotation, plus the pitch/roll tilt away from the expected flip. If
		// tilt_* scatters run to run it is solver noise (and worth constraining); if it
		// repeats, it is a real tilt between the two spaces and must be kept.
		<< "  yaw=" << std::setprecision(2) << ctx.calibratedRotation(1)
		<< "  pitch=" << ctx.calibratedRotation(2)
		<< "  roll=" << ctx.calibratedRotation(0)
		<< "  tilt_x=" << tiltX
		<< "  tilt_z=" << tiltZ
		// Raw lever arm this run, and the running average actually applied. The raw
		// column is what reveals run-to-run scatter; the averaged one is what ships.
		<< "  lever_raw_mm=" << std::setprecision(1)
		<< g_lastLeverRaw.x() * 1000.0 << "/" << g_lastLeverRaw.y() * 1000.0 << "/" << g_lastLeverRaw.z() * 1000.0
		<< "  lever_mm=" << g_lastLever.x() * 1000.0 << "/" << g_lastLever.y() * 1000.0 << "/" << g_lastLever.z() * 1000.0
		<< "  lever_n=" << ctx.leverSamples
		<< std::setprecision(2)
		<< "  split_deg=" << g_lastSplitHalfDeg
		<< "  stations=" << g_lastStationCount
		<< "  axis_var=" << std::setprecision(6) << g_lastAxisVariance
		<< "  hmd=" << ctx.hmdSerial
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

// Mirror of the driver's SetHmdTracker acceptance rules (ServerTrackedDeviceProvider.cpp, "IPC
// input boundary"). The protocol has no NACK — HandleRequest answers ResponseSuccess whether or
// not the driver accepted the message — so a message the driver would reject must be caught HERE,
// before SendBlocking and before the g_appliedHmd cache records it as applied. Without this
// pre-flight, one rejection became permanent silent divergence: the UI showed the new calibration
// (and enable/disable state, which rides the same message) while the driver kept the old one, and
// the dedupe suppressed every retry for the life of the overlay process.
static bool SendableQuat(const vr::HmdQuaternion_t &q)
{
	if (!std::isfinite(q.w) || !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z))
		return false;
	const double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
	return std::fabs(1.0 - n) <= 1e-3;
}

static bool SendableVec(const vr::HmdVector3d_t &v, double maxAbs)
{
	if (!std::isfinite(v.v[0]) || !std::isfinite(v.v[1]) || !std::isfinite(v.v[2]))
		return false;
	return std::fabs(v.v[0]) <= maxAbs && std::fabs(v.v[1]) <= maxAbs && std::fabs(v.v[2]) <= maxAbs;
}

static bool SendableScale(double s)
{
	return std::isfinite(s) && s >= 0.5 && s <= 2.0;
}

static const char *HmdTrackerRequestProblem(const protocol::SetHmdTracker &cmd)
{
	if (!SendableQuat(cmd.offsetRotation) || !SendableQuat(cmd.calibrationRotation))
		return "offset/calibration rotation is not a finite unit quaternion";
	if (!SendableVec(cmd.offsetTranslation, 10.0) || !SendableVec(cmd.calibrationTranslation, 10.0))
		return "offset/calibration translation not finite or >10 m";
	if (!SendableScale(cmd.calibrationScale) || !SendableScale(cmd.hmdScale))
		return "calibrationScale/hmdScale outside [0.5, 2.0]";
	if (!std::isfinite(cmd.predictionTime))
		return "predictionTime not finite";
	return nullptr;
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

// `quiet` suppresses the user-facing log line so the split-half reproducibility check
// can re-solve on subsets without narrating each one.
Eigen::Vector3d CalibrateRotation(const std::vector<Sample>& samples, bool quiet = false)
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

	if (!quiet)
	{
		snprintf(buf, sizeof buf, "Calibrated rotation: yaw=%.2f pitch=%.2f roll=%.2f\n", euler[1], euler[2], euler[0]);
		CalCtx.Log(buf);
	}
	return euler;
}

// Minimum positional spread (RMS, metres) required to fit headset scale at all.
// Empirically this is the reliable discriminator: logged calibrations at spread
// 0.235-0.286 m all landed within 0.5% of each other, while one at 0.160 m fitted
// a scale ~3% off. Matches the live coach's threshold, so "cover more space" and
// "scale accepted" agree. Below this, scale falls back to exactly 1.
// Lowered from 0.20. That figure was set from calibrations taken before samples were
// gated on linear speed, when a 0.160 m run fitted a scale ~3% off; with the speed
// gates in place -- and now permitting samples while actually walking -- low-spread
// fits are far better behaved, and 0.20 was rejecting scale on runs that a real user
// can realistically produce. The residual risk of a badly conditioned fit at low
// spread is handled by LowSpreadMaxDeviation below rather than by refusing to look.
static const double ScaleSpreadThreshold = 0.15;
// Above this spread the fit is trusted on its own merits; between the two the fit is
// only accepted if it stays close to unity (see LowSpreadMaxDeviation).
static const double ScaleConfidentSpread = 0.20;
// Real headset-to-lighthouse scale sits within a fraction of a percent of 1. A
// low-spread fit claiming more than this is ill-conditioning, not a discovery -- it is
// exactly the shape of the historical 0.966 outlier, which the formal standard error
// failed to catch. Plausibility, not precision, is what filters that case.
// Tightened from 0.02, which was set from first principles rather than from the data
// and promptly let through a 0.98814 fit at 0.153 m spread -- 1.19% off unity, when
// every well-covered measurement on real hardware has landed in 0.992-0.998, i.e.
// inside 0.8%. A band that admits a value further from unity than any trustworthy
// measurement is not a filter. 1% admits all of them and rejects that fit.
static const double LowSpreadMaxDeviation = 0.01;
static const double MinCalibratedScale = 0.9;
static const double MaxCalibratedScale = 1.1;
// Secondary guard for the case where spread is adequate but residuals are noisy.
// Note: the formal standard error only captures random scatter, not the
// ill-conditioning bias that dominates at low spread -- it reported 0.2% on the
// 0.160 m fit that was actually ~3% wrong -- which is why the spread gate above,
// not this, is the primary check.
// Relaxed from 0.005: at the lower spread now permitted the formal standard error
// naturally rises, and holding it at 0.5% would reject those fits before the
// plausibility check below ever saw them. It was never the effective filter anyway --
// it reported 0.2% on the 0.160 m fit that was ~3% wrong -- so it serves as a coarse
// sanity bound while LowSpreadMaxDeviation does the real work.
static const double MaxScaleStdErr = 0.012;

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

static double EstimateHmdSpaceScale(const std::vector<Sample> &samples, const Eigen::Matrix3d &rotation, double targetModelScale, double priorScale)
{
	// Unreachable from the live path (readyToSolve cannot fire on an empty set), but
	// CalibrateRotation beside this one was hardened for exactly this shape of degenerate input
	// (finding 4) and these were not. Scale is the one quantity the runtime can never
	// re-estimate, so the safe answer on no data is the prior, never a NaN.
	if (samples.empty())
	{
		CalCtx.Log("No samples to fit headset scale - keeping the previous value.\n");
		g_lastScaleSource = (priorScale != 1.0) ? "kept_no_samples" : "default_no_samples";
		g_lastScaleStdErr = -1.0;
		g_lastScaleStdErrEff = -1.0;
		return priorScale;
	}

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
		if (priorScale != 1.0)
			snprintf(buf, sizeof buf,
				// %.3f, not %.2f: a straddling run (raw spread passes the finish gate, collapsed
				// spread fails this one — A11) rendered both numbers identically at two decimals,
				// telling the user "spread 0.15 m, need >= 0.15 m".
				"Not enough positional movement to measure headset scale (spread %.3f m, need >= %.3f m).\n"
				"Keeping your previously measured headset scale %.5f (%+.2f%%). To re-measure it, walk and\n"
				"crouch to cover more of your play space; head rotation alone cannot determine scale.\n",
				spread, ScaleSpreadThreshold, priorScale, (priorScale - 1.0) * 100.0);
		else
			snprintf(buf, sizeof buf,
				"Not enough positional movement to measure headset scale (spread %.3f m, need >= %.3f m), assuming 1.\n"
				"Walk and crouch to cover more of your play space while calibrating; head rotation alone cannot determine scale.\n",
				spread, ScaleSpreadThreshold);
		CalCtx.Log(buf);
		g_lastScaleSource = (priorScale != 1.0) ? "kept_low_spread" : "default_low_spread";
		g_lastScaleStdErr = -1.0;
		g_lastScaleStdErrEff = -1.0;
		return priorScale;
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

	g_lastScaleStdErr = scaleStdErr;

	// N3-c fix: the standard error above assumes all nObs rows are independent. They are not.
	// Samples are collected in bursts while the user holds still at a station, so within a
	// station they are near-duplicates of each other -- the intra-cluster correlation is close
	// to 1. Under rho ~ 1 the effective sample size is the number of *stations* K, not the
	// number of samples n, and the variance inflation (design effect) is n/K. So the honest
	// standard error is sqrt(n/K) times the naive one. That is the ~7x optimism N3-c records:
	// the formula reported 0.2% on a 0.160 m fit that was ~3% wrong.
	//
	// This corrects the number that is *reported*. It deliberately does NOT move the
	// MaxScaleStdErr gate onto it: that threshold (0.012) was tuned against the naive value,
	// and switching the gate would tighten acceptance by the same sqrt(n/K) -- silently
	// rejecting most calibrations on a live rig with no replay harness to check the new
	// operating point against. Re-tuning the gate needs a few sessions of logged
	// scale_stderr_eff_pct to pick a threshold from data. The spread gate above remains the
	// primary check, as the comment there already says.
	{
		int stations = 0;
		if (g_sampleStation.size() == samples.size() && !g_sampleStation.empty())
		{
			std::set<int> distinct(g_sampleStation.begin(), g_sampleStation.end());
			stations = (int)distinct.size();
		}
		else
		{
			stations = g_stationCount;   // fallback: partition unavailable or stale
		}

		if (stations >= 1 && std::isfinite(scaleStdErr) && samples.size() > (size_t)stations)
			g_lastScaleStdErrEff = scaleStdErr
				* std::sqrt((double)samples.size() / (double)stations);
		else
			g_lastScaleStdErrEff = std::isfinite(scaleStdErr) ? scaleStdErr : -1.0;
	}

	if (!(scaleStdErr <= MaxScaleStdErr))
	{
		g_lastScaleSource = (priorScale != 1.0) ? "kept_weak_fit" : "default_weak_fit";
		if (priorScale != 1.0)
			snprintf(buf, sizeof buf,
				"Headset scale is not determined by this calibration (fit %.5f +/- %.2f%%, positional spread %.2f m).\n"
				"Keeping your previously measured headset scale %.5f. To re-measure it, walk around and cover\n"
				"more of your play space while calibrating; head rotation alone is not enough.\n",
				fittedScale, scaleStdErr * 100.0, spread, priorScale);
		else
			snprintf(buf, sizeof buf,
				"Headset scale is not determined by this calibration (fit %.5f +/- %.2f%%, positional spread %.2f m) - assuming 1.\n"
				"To measure scale, walk around and cover more of your play space while calibrating; head rotation alone is not enough.\n",
				fittedScale, scaleStdErr * 100.0, spread);
		CalCtx.Log(buf);
		return priorScale;
	}

	if (fittedScale < MinCalibratedScale || fittedScale > MaxCalibratedScale)
	{
		g_lastScaleSource = (priorScale != 1.0) ? "kept_implausible" : "default_implausible";
		if (priorScale != 1.0)
			snprintf(buf, sizeof buf, "Fitted space scale %.5f is not plausible; keeping previous headset scale %.5f\n", fittedScale, priorScale);
		else
			snprintf(buf, sizeof buf, "Fitted space scale %.5f is not plausible, assuming headset scale 1\n", fittedScale);
		CalCtx.Log(buf);
		return priorScale;
	}

	snprintf(buf, sizeof buf, "Fitted headset space scale relative to lighthouse: %.5f (%+.2f%%, +/- %.2f%%), implied absolute headset scale: %.5f\n",
		fittedScale, (fittedScale - 1.0) * 100.0, scaleStdErr * 100.0, fittedScale / targetModelScale);
	// Low-spread band: the fit is usable but weakly conditioned, so accept it only if it
	// is close to unity. A real headset scale never strays far from 1; a large excursion
	// here is the estimator failing, and in fusion mode a wrong scale cannot be undone
	// at runtime, so the prior is the safer answer.
	if (spread < ScaleConfidentSpread && std::fabs(fittedScale - 1.0) > LowSpreadMaxDeviation)
	{
		snprintf(buf, sizeof buf,
			"Fitted scale %.5f (%+.2f%%) is too far from 1 to trust at spread %.2f m - keeping %.5f.\n"
			"Cover more of your play space to measure a scale this different.\n",
			fittedScale, (fittedScale - 1.0) * 100.0, spread, priorScale);
		CalCtx.Log(buf);
		g_lastScaleSource = (priorScale != 1.0) ? "kept_lowspread_outlier" : "default_lowspread_outlier";
		return priorScale;
	}

	CalCtx.Log(buf);
	g_lastScaleSource = (spread < ScaleConfidentSpread) ? "measured_lowspread" : "measured";
	return fittedScale;
}

// Largest single-run jump still treated as noise rather than a changed setup. Measured
// run-to-run sigma on this rig is 0.757%, so 3% is ~4 sigma: past that the likelier
// explanation is a different headset, streaming stack or SLAM build than a bad draw, and
// blending across a real change would fight it for eight runs. Mirrors kRemountStepM's role
// for the lever arm.
static const double kScaleResetStep = 0.03;
// Same cap, and the same reason, as kMaxLeverAverage: an uncapped window makes every
// recalibration weaker than the last, so a genuine change can never be tracked.
static const int kMaxScaleAverage = 8;

// Average hmdScale across calibrations.
//
// Scale is the one calibrated quantity the runtime can never re-estimate: fusion applies
// hmdScale directly and the EKF carries yaw + translation with no scale state, so whatever a
// run happens to draw is frozen until the next calibration. Orientation and translation
// self-heal; this does not. Measured over 14 consecutive solves on this rig the draw scatters
// sigma 0.757% (peak-to-peak 2.63%) while each solve's own error bar averages 0.202% -- so the
// estimator is not merely noisy, it does not know that it is noisy, and no per-solve gate
// tuned against that error bar can catch it. Averaging is the one remedy that does not require
// trusting the error bar, and it is the identical scheme ComputeRelativeOffset already applies
// to the lever arm.
//
// Only pool a run that actually MEASURED scale. Every kept_*/default_* path in
// EstimateHmdSpaceScale returns priorScale unchanged, so pooling those would blend the running
// average into itself: the sample count would climb while no information was added, making the
// average look better determined than it is and freezing it against later real measurements.
static void PoolHmdScale(CalibrationContext &ctx, double fitted)
{
	// rfind(...,0)==0 is "starts with", which is exactly the distinction the source string
	// encodes: "measured" and "measured_lowspread" are this run's own fit, everything else
	// ("kept_*", "default_*") is the prior handed back.
	const bool measured = std::string(g_lastScaleSource).rfind("measured", 0) == 0;

	if (!measured)
	{
		// Identical to the pre-pooling behaviour: the estimator already returned the prior.
		g_lastScaleRaw = -1.0;
		g_lastScalePoolAction = "skipped_not_measured";
		ctx.hmdScale = fitted;
		return;
	}

	g_lastScaleRaw = fitted;
	g_lastScalePoolAction = "blended";

	// Which headset the average describes. A different HMD -- or the same headset behind a
	// different streaming stack, which this rig has logged as both "1PASH5D1P17365" and
	// "VRLINKHMDQUESTPRO" -- has its own SLAM scale, so the old average is not evidence about
	// the new one. Keyed on hmdScaleSerial rather than hmdSerial for the same reason the lever
	// arm keys on leverSerial: BeginSamplingPhase overwrites the live serial when sampling
	// starts, so an abandoned run must not be able to retarget the average.
	if (ctx.hmdScaleSamples > 0 && !ctx.hmdScaleSerial.empty() && !ctx.hmdSerial.empty()
		&& ctx.hmdScaleSerial != ctx.hmdSerial)
	{
		ctx.Log("Headset changed - starting a fresh headset-scale average.\n");
		ctx.hmdScaleSamples = 0;
		g_lastScalePoolAction = "reset_hmd_changed";
	}

	const double prior = ctx.hmdScale;
	const double stepPct = (prior > 0.0) ? std::fabs(fitted - prior) / prior : 0.0;

	if (ctx.hmdScaleSamples > 0 && stepPct > kScaleResetStep)
	{
		char buf[256];
		snprintf(buf, sizeof buf,
			"Headset scale moved %.2f%% (%.5f -> %.5f) - too far to be run-to-run noise, so this\n"
			"is treated as a changed setup and starts a fresh average instead of being blended.\n",
			stepPct * 100.0, prior, fitted);
		ctx.Log(buf);
		ctx.hmdScaleSamples = 0;
		g_lastScalePoolAction = "reset_large_step";
	}

	double merged = fitted;
	if (ctx.hmdScaleSamples > 0)
	{
		const int n = (std::min)(ctx.hmdScaleSamples, kMaxScaleAverage);
		merged = prior + (fitted - prior) / (double)(n + 1);
	}
	else if (g_lastScalePoolAction == std::string("blended"))
	{
		g_lastScalePoolAction = "seeded";
	}

	if (ctx.hmdScaleSamples < kMaxScaleAverage)
		ctx.hmdScaleSamples++;

	// This solve produced a measurement on the current headset: it owns the average from here.
	// Only when the serial actually read, though. GetDeviceSerial discards its error and
	// returns "" after a wake/re-enumerate (the V2 failure mode, fixed once already for the
	// lever arm): storing "" as the owner permanently disarms the headset-changed reset in
	// the guard above, because that guard requires BOTH serials non-empty to fire. An empty
	// read keeps the previous owner; the average still updates.
	if (!ctx.hmdSerial.empty())
		ctx.hmdScaleSerial = ctx.hmdSerial;
	ctx.hmdScale = merged;

	char scaleBuf[256];
	snprintf(scaleBuf, sizeof scaleBuf,
		"Headset scale: this run %.5f (%+.2f%%); averaged over %d run(s) -> %.5f (%+.2f%%).\n",
		fitted, (fitted - 1.0) * 100.0, ctx.hmdScaleSamples, merged, (merged - 1.0) * 100.0);
	ctx.Log(scaleBuf);
}

// Scale from pairwise baselines: the median of |dref| / |dtarget| over well-separated pairs.
//
// The SVD in EstimateHmdSpaceScale solves scale jointly with a free 3-vector translation and a
// 3x3 orientation block, so scale competes with six nuisance parameters and inherits their
// error -- which is why the observability guard there has to reason about conditioning at all.
// Distances do not have that problem: for any two poses, |ref_i - ref_j| / |tgt_i - tgt_j| is
// the scale ratio directly, because a rigid rotation and a translation each preserve distance
// and cancel exactly. RESEARCH.md D1/B1 records that scale is algebraically recoverable from
// motion pairs; this is that, with the nuisance parameters removed rather than fitted.
//
// Only long baselines count. Per-pair ratio noise goes as sigma_tracker / baseline, and short
// pairs outnumber long ones quadratically, so including them would let noise dominate by sheer
// count. The median over survivors is then insensitive to the occasional latency-skewed sample
// that least squares would chase.
//
// COMPUTED AND LOGGED, NOT APPLIED. It rides alongside the SVD value for one build so the two
// can be compared on real calibrations before either is trusted over the other -- changing the
// estimator and the observation set in the same build would make a bad result uninterpretable.
static double PairwiseBaselineScale(const std::vector<Sample> &samples,
	const Eigen::Vector3d &leverVec, bool haveLever, int &outPairs)
{
	outPairs = 0;
	if (samples.size() < 3)
		return -1.0;

	// At ~0.7 mm of tracker noise at the timescales the solve cares about, a 20 cm baseline
	// carries ~0.5% per-pair ratio uncertainty; shorter pairs degrade rapidly from there.
	const double kMinBaseline = 0.20;

	// The two devices are ~100 mm apart on a rigid mount, so raw |dref| vs |dtgt| are chords
	// between DIFFERENT points on the same body: head rotation sweeps the tracker through an
	// arc the HMD origin does not travel, and the raw ratio measures how much the head turned,
	// not scale -- the logged scale_pair sat at 0.78-0.86 across 2026-08-16 while the SVD fit
	// read 0.994. A first attempt to *filter* rotated pairs out required <1.1 deg of rotation
	// across a >=20 cm baseline, which never happens while calibrating, and logged
	// scale_pair_n=0 all night (2026-08-16/17) -- muting the diagnostic instead of fixing it.
	//
	// So remove the bias instead of filtering it: project each tracker sample to the HMD point
	// it implies, tp + R_tq * lever, and ratio same-point chords. The calibration rotation is
	// a fixed rotation and preserves chord lengths, so raw target-space quantities suffice.
	// Validated offline against all 13 calsamples_20260816/17_* runs before shipping: the
	// corrected median reads 0.986-1.001 on every clean run (naive: 0.71-0.88), agrees with
	// the SVD fit to 0.08-0.44% on the three runs where scale was measured (inside the SVD's
	// own 0.757% run-to-run sigma), and still honestly reports garbage (0.703) on the
	// frame-broken 165347 run -- which the rigid-pair gate now discards anyway.
	//
	// Needs the lever arm, so the first-ever calibration reports unavailable rather than a
	// number known to carry the full rotation bias.
	if (!haveLever)
		return -1.0;

	std::vector<Eigen::Vector3d> implied;
	implied.reserve(samples.size());
	for (auto &s : samples)
		implied.push_back(s.target.trans + s.target.rot * leverVec);

	std::vector<double> ratios;
	for (size_t i = 0; i < samples.size(); i++)
	{
		for (size_t j = 0; j < i; j++)
		{
			const double dRef = (samples[i].ref.trans - samples[j].ref.trans).norm();
			const double dTgt = (implied[i] - implied[j]).norm();
			if (dRef < kMinBaseline || dTgt < kMinBaseline)
				continue;
			ratios.push_back(dRef / dTgt);
		}
	}

	// Too few long baselines to median meaningfully -- that is itself the "not enough spread"
	// condition, reported as unavailable rather than as a confident number.
	if (ratios.size() < 8)
		return -1.0;

	std::sort(ratios.begin(), ratios.end());
	outPairs = (int)ratios.size();
	const size_t mid = ratios.size() / 2;
	return (ratios.size() % 2) ? ratios[mid] : 0.5 * (ratios[mid - 1] + ratios[mid]);
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
	if (samples.empty())
		return Eigen::Vector3d::Zero();

	Eigen::Vector3d accum = Eigen::Vector3d::Zero();

	for (auto &sample : samples)
		accum += sample.ref.rot.transpose() * (calScale * (calRot * sample.target.trans) + calTrans - sample.ref.trans);

	return accum / (double)samples.size();
}

static double RetargetingErrorRMS(const std::vector<Sample> &samples, const Eigen::Vector3d &hmdToTargetPos, const Eigen::Matrix3d &calRot, const Eigen::Vector3d &calTrans, double calScale)
{
	// NaN, not 0: this feeds the `!(rmsError <= 0.1)` accept gate (finding 3), and a zero
	// residual would read as a perfect fit and be saved.
	if (samples.empty())
		return std::numeric_limits<double>::quiet_NaN();

	double accum = 0;

	for (auto &sample : samples)
		accum += (calScale * (calRot * sample.target.trans) + calTrans - (sample.ref.rot * hmdToTargetPos + sample.ref.trans)).squaredNorm();

	return std::sqrt(accum / (double)samples.size());
}

// Which device was untracked on the most recent CollectSample, or nullptr when the sample was
// good. Named rather than boolean so the eventual abort message can say which device went away.
static const char *g_lastCollectFailure = nullptr;

Sample CollectSample(const CalibrationContext &ctx)
{
	vr::TrackedDevicePose_t reference, target;
	reference.bPoseIsValid = false;
	target.bPoseIsValid = false;

	reference = ctx.devicePoses[0];
	target = ctx.devicePoses[ctx.targetID];

	// A single untracked tick is not a failed calibration.
	//
	// This used to abort the whole run -- state = None, every sample discarded -- the first
	// time either pose came back invalid. The head tracker is line-of-sight to a base station,
	// so brief losses are routine: turning away, an arm across the puck, a step through a blind
	// spot. Throwing away a minute of good coverage for one bad frame is never the right
	// response, and because the abort logged only to the in-VR message list it also left no
	// record of why the run vanished.
	//
	// Report which device is missing and let the caller decide: it coasts over short gaps and
	// aborts only once the loss has lasted long enough to be real. No logging here -- this runs
	// at 20 Hz, so a per-tick message would bury the log in the time it takes to walk past a
	// blind spot.
	g_lastCollectFailure = nullptr;
	if (!reference.bPoseIsValid)
		g_lastCollectFailure = "Reference device (HMD)";
	else if (!target.bPoseIsValid)
		g_lastCollectFailure = "Target device (head tracker)";

	if (g_lastCollectFailure)
		return Sample();

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
	// Send() throws on a broken pipe, and this runs on the overlay main loop with no handler
	// above it — an uncaught throw here is std::terminate, killing the overlay before its
	// exit-time SaveProfile. A pipe break happens precisely when the driver goes away
	// (SteamVR shutdown, driver reload), i.e. exactly when unsaved state exists. On failure
	// the applied-state cache is left NOT-known so the next scan re-sends — caching a send
	// that never arrived is the silent-desync class fa7dc28 fixed.
	try
	{
		Driver.SendBlocking(req);
		prev.known = true;
		prev.enabled = enabled;
		prev.translation = translation;
		prev.rotation = rotation;
		prev.scale = scale;
	}
	catch (const std::runtime_error &e)
	{
		prev.known = false;
		std::cerr << "Failed to send device transform for id " << id << ": " << e.what() << std::endl;
	}
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

	// The driver would reject this whole message (and still answer ResponseSuccess). Don't send
	// it and above all don't cache it as applied — the driver keeps its current state, and the
	// next scan retries if the values change. Logged once per distinct offender, not at scan rate.
	if (const char *problem = HmdTrackerRequestProblem(req.setHmdTracker))
	{
		static protocol::SetHmdTracker lastRejected{};
		static bool rejectedKnown = false;
		if (!rejectedKnown || !HmdTrackerEq(lastRejected, req.setHmdTracker))
		{
			std::cerr << "NOT sending SetHmdTracker (driver would reject it): " << problem
				<< " — driver keeps its previous calibration/state" << std::endl;
			lastRejected = req.setHmdTracker;
			rejectedKnown = true;
		}
		return;
	}

	// Same broken-pipe hazard as SetDeviceTransform above; same cache rule on failure.
	try
	{
		Driver.SendBlocking(req);
		g_appliedHmd = req.setHmdTracker;
		g_appliedHmdKnown = true;
	}
	catch (const std::runtime_error &e)
	{
		g_appliedHmdKnown = false;
		std::cerr << "Failed to send SetHmdTracker: " << e.what() << std::endl;
		return;
	}

	// A disable makes the driver wipe its slamSync set: the SetHmdTracker handler runs
	// ResetEstimators(clearLastGood=true, clearSlamSync=true), which memsets the whole array
	// (ServerTrackedDeviceProvider.cpp:479, :576). Our applied-state cache does not see that
	// happen, and the re-enrolment loop in ScanAndApplyProfile is guarded on `overrideActive`,
	// so it does not run while the override is off either. The cache therefore still claims
	// every device is enrolled, and when the override comes back every id hits `continue` and
	// no SetSlamSync is ever re-sent.
	//
	// Measured in session_20260815_211608: the set was populated once at 21:16:15 (ids 1-8),
	// wiped by the disable at 21:16:32, and never re-sent across NINE further disable/enable
	// cycles -- so the body pucks ran the remaining ~76 minutes with no drift correction while
	// the UI showed Continuous Sync on. That is precisely the "body drifts relative to view"
	// symptom the ops register recorded as resolved by turning Continuous Sync ON; the setting
	// was on, but the enrolment silently was not.
	//
	// Upstream re-sent unconditionally on every ~1 s scan, so this state could not desynchronise.
	// The dedupe that replaced it saves a pipe write per scan and must therefore be invalidated
	// wherever the driver drops the state it is caching.
	if (!enabled)
	{
		for (uint32_t i = 0; i < vr::k_unMaxTrackedDeviceCount; ++i)
		{
			g_appliedSlamKnown[i] = false;
			g_appliedSlam[i] = false;
		}
	}
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
	// The lever arm is a physical constant: the tracker is rigidly mounted, so this
	// vector cannot actually change between calibrations. Measured, it moves ~12 mm run
	// to run (observed 103.3 mm vs 91.7 mm magnitude, 13 mm of it in Z), because it is
	// recovered from the arc the tracker sweeps under head rotation and recent runs
	// sample that arc thinly. That noise lands directly as a shift between the view and
	// the body trackers -- recalibrate, get a different offset -- which is exactly the
	// "slight offset every time" symptom.
	//
	// Averaging repeated measurements of a fixed quantity is unambiguously correct and
	// has no tunable: each calibration contributes 1/n and the estimate tightens with
	// use instead of jumping to the latest noisy draw. A remount genuinely does change
	// the vector, so a large step restarts the average rather than being blended away;
	// the bound is set from physical plausibility (a re-seat moves the tracker much
	// further than the estimator's ~12 mm scatter), not fitted to any run.
	const Eigen::Vector3d measured = transAccum;
	static const double kRemountStepM = 0.030;
	// Typical run-to-run scatter of the estimator itself (see comment above: ~12 mm
	// observed). A shift below this is ordinary noise; above it but still under the
	// remount cutoff is neither clearly noise nor clearly a remount, so it gets blended
	// in (as before) but is called out in the log instead of passing silently.
	static const double kNoiseFloorM = 0.012;

	// Deferred tracker-swap reset (see BeginSamplingPhase). Landing it here means the old
	// tracker's average survives in the profile until this run produces a replacement, so an
	// abandoned calibration can no longer destroy a good lever arm. Must run before `prior` is
	// read, or the old tracker's offset would be blended into the new one.
	//
	// Keyed on the persisted lever OWNER (ctx.leverSerial), not on a process-local flag: the
	// flag scheme desynchronised from ctx.trackerSerial, which BeginSamplingPhase overwrites
	// immediately and any SaveProfile persists. An abandoned swap run then stored serial B next
	// to tracker A's average — on relaunch the flag was gone, serials matched, and a later B
	// calibration blended A's mount point in as prior. Conversely, swapping back to A before
	// solving left the flag armed and wiped A's own average. The owner serial travels with the
	// data it describes, so both directions resolve correctly regardless of restarts.
	if (ctx.validRelativeOffset && !ctx.leverSerial.empty() && !ctx.trackerSerial.empty()
		&& ctx.leverSerial != ctx.trackerSerial)
	{
		ctx.Log("Target tracker changed - starting a fresh head-tracker offset average.\n");
		ctx.relativeTranslation = { 0, 0, 0 };
		ctx.validRelativeOffset = false;
		ctx.leverSamples = 0;
	}
	// This solve produced a measurement on the current tracker: it owns the average from here.
	ctx.leverSerial = ctx.trackerSerial;

	const Eigen::Vector3d prior(
		ctx.relativeTranslation.v[0], ctx.relativeTranslation.v[1], ctx.relativeTranslation.v[2]);

	// A profile loaded from disk carries a lever arm already, so it counts as one prior
	// observation; otherwise a new session would discard everything learned before it.
	if (ctx.validRelativeOffset && ctx.leverSamples == 0)
		ctx.leverSamples = 1;

	const double shiftM = (measured - prior).norm();
	if (ctx.leverSamples > 0 && shiftM > kRemountStepM)
	{
		char buf[192];
		snprintf(buf, sizeof buf,
			"Head tracker offset moved %.0f mm - treating as a remount and starting a fresh average.\n",
			shiftM * 1000.0);
		CalCtx.Log(buf);
		ctx.leverSamples = 0;
	}
	else if (ctx.leverSamples > 0 && shiftM > kNoiseFloorM)
	{
		char buf[192];
		snprintf(buf, sizeof buf,
			"Head tracker offset moved %.0f mm - more than typical run-to-run noise but under the remount\n"
			"threshold, so it is being blended into the average rather than reset.\n",
			shiftM * 1000.0);
		CalCtx.Log(buf);
	}

	// Bounded averaging window. leverSamples persists in the profile (lever_n), so an
	// uncapped count makes every recalibration weaker than the last: at n=20 a genuine
	// 25 mm re-seat — under kRemountStepM, so blended rather than reset — moves the
	// applied lever by 1.2 mm and leaves the user ~24 mm wrong, and the next run helps
	// even less (1/22, 1/23, ...). That is the same "slight offset every time" symptom
	// this averaging was added to remove, turned into a sticky one. Cap the window so
	// it keeps averaging down noise but stays able to track a real remount.
	static const int kMaxLeverAverage = 8;
	Eigen::Vector3d merged = measured;
	if (ctx.leverSamples > 0)
	{
		const int n = (std::min)(ctx.leverSamples, kMaxLeverAverage);
		merged = prior + (measured - prior) / (double)(n + 1);
	}
	if (ctx.leverSamples < kMaxLeverAverage)
		ctx.leverSamples++;

	char leverBuf[224];
	snprintf(leverBuf, sizeof leverBuf,
		"Head tracker offset: this run (%.1f, %.1f, %.1f) mm; averaged over %d run(s) -> (%.1f, %.1f, %.1f) mm.\n",
		measured.x() * 1000.0, measured.y() * 1000.0, measured.z() * 1000.0, ctx.leverSamples,
		merged.x() * 1000.0, merged.y() * 1000.0, merged.z() * 1000.0);
	CalCtx.Log(leverBuf);

	g_lastLever = merged;
	g_lastLeverRaw = measured;

	ctx.relativeTranslation.v[0] = merged.x();
	ctx.relativeTranslation.v[1] = merged.y();
	ctx.relativeTranslation.v[2] = merged.z();
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

		// Refresh targetModelScale from the live device. It is a pure function of the head
		// tracker's model, but ParseProfile defaults it to 1.0 when the key is absent — and a
		// profile written before the key existed (upstream, early fork, or a restored backup)
		// then silently selects the wrong geometry at the deviceScale line below: model/1.0
		// is the exact "absolute" regime the A-X1 revert disproved, shipping the Tundras 0.34%
		// off against the head anchor with no log line. Recomputing here whenever the tracker
		// is present makes the stored value self-healing regardless of profile vintage; the
		// next SaveProfile persists the healed value. (2026-08-17 audit completeness critic P1
		// — a cross-unit defect: the loader was sane in isolation, the divisor was sane given
		// the value, and no per-unit audit owned the join.)
		if (ctx.targetID != vr::k_unTrackedDeviceIndexInvalid)
			ctx.targetModelScale = GetLighthouseModelScale(ctx.targetID);
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
		// Broken-pipe guard (see SetDeviceTransform). One failure means the pipe is dead for
		// all of them — stop the loop rather than throwing up to 60 more times this scan.
		try
		{
			Driver.SendBlocking(req);
			g_appliedSlamKnown[id] = true;
			g_appliedSlam[id] = sync;
		}
		catch (const std::runtime_error &e)
		{
			g_appliedSlamKnown[id] = false;
			std::cerr << "Failed to send SetSlamSync for id " << id << ": " << e.what() << std::endl;
			break;
		}
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

// Rigid-pair consistency state. The HMD and the head tracker are bolted to the same head,
// so between any two samples their displacements can differ by no more than the lever arm
// sweeping through the rotation: | |dref| - |dtgt| | <= 2*|L|*sin(dtheta/2). Anything past
// that is not motion, it is one of the two reference frames moving underneath us -- which
// on a streamed headset means a SLAM relocalisation mid-run.
static bool g_havePrevAccepted = false;
static Eigen::Vector3d g_prevAcceptedRef = Eigen::Vector3d::Zero();
static Eigen::Vector3d g_prevAcceptedTgt = Eigen::Vector3d::Zero();
static Eigen::Matrix3d g_prevAcceptedTgtRot = Eigen::Matrix3d::Identity();

// Tracking-loss coasting. g_lossStart is when the current gap began (0 while tracking),
// g_lossTotalTicks counts every dropped tick across the run so the log can show that a
// finished calibration had holes in it -- coverage numbers alone cannot reveal that.
static double g_lossStart = 0.0;
static int g_lossRunTicks = 0;
static int g_lossTotalTicks = 0;
// How long either device may stay untracked before the run is abandoned. Long enough to walk
// through a blind spot and come back, short enough that a genuinely dead tracker does not
// leave the user staring at a frozen bar.
static const double kMaxTrackingLossSec = 2.0;

// Plateau finish. Runs used to solve the instant all three gates cleared, which is why 12 of
// the last 14 calibrations on this rig finished with spread between 0.150 and 0.199 m against
// a 0.150 m gate: the run cut the user off at the minimum every time, and scale precision goes
// as 1/spread. Once the gates pass, keep sampling while coverage is still materially improving.
// This never lowers a bar -- the live gates stay authoritative -- it only delays the solve.
static double g_gatesPassedTime = 0.0;   // 0 until all three gates first pass
static double g_plateauRefSpread = 0.0;  // spread at the last plateau checkpoint
static double g_plateauRefTime = 0.0;
// Improvement below this over one window counts as plateaued.
static const double kPlateauMinGainFrac = 0.02;
static const double kPlateauWindowSec = 4.0;
// Hard ceiling on the extra sampling, so "keep going while it improves" can never become an
// unbounded run for a user who keeps drifting slowly across the room.
static const double kPlateauMaxExtraSec = 20.0;

// --- Stations -------------------------------------------------------------
//
// Sampling is capped at 20 Hz (CalibrationTick early-returns below 0.05 s), and
// samples are only taken while nearly still. Standing in one place therefore emits
// ~20 near-identical samples a second: each one advances the sample counter, none of
// them adds information, and the O(N^2) pairwise solves then weight that cluster as
// though it were hundreds of independent observations.
//
// A "station" is one occupied pose: a run of samples within a small position and
// orientation radius. Samples past a cap within one station are discarded rather than
// collected, so the counter can only advance by actually moving or looking somewhere
// new -- which is exactly the motion the solve needs. Progress is then measured in
// stations, not samples.
// Wider than the original 5 cm / 10° so a slow yaw in place cannot mint a new
// "station" every few degrees and fill the progress bar without ever walking.
// Finish also requires TargetSpread + SecondAxisVariance; stations alone never
// complete a run (see readyToSolve below).
static const double kStationPosRadius = 0.08;          // m
static const double kStationAngRadius = 18.0 * EIGEN_PI / 180.0;
static const int kMaxSamplesPerStation = 10;           // beyond this a pose is redundant

struct StationTracker
{
	bool have = false;
	Eigen::Vector3d pos = Eigen::Vector3d::Zero();
	Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
	int count = 0;
	int index = -1;
};
static StationTracker g_station;
// g_sampleStation / g_stationCount are declared near the top of this file — the scale solve
// needs them and runs earlier.
// High-water mark of the readiness bar, so it never runs backwards (display only).
static double g_readyFracHigh = 0.0;
// The split-half check re-solves the rotation twice and the solve is O(N^2) in samples,
// so it is throttled rather than run on every accepted sample.
static double g_lastSplitCheckTime = 0.0;
static double g_splitFailStart = 0.0;

// Angle between two rotations, in radians. Representation-independent, unlike
// comparing Euler triples.
static double RotAngleBetween(const Eigen::Matrix3d &a, const Eigen::Matrix3d &b)
{
	double c = ((a.transpose() * b).trace() - 1.0) * 0.5;
	if (c > 1.0) c = 1.0;
	if (c < -1.0) c = -1.0;
	return std::acos(c);
}

static void ResetStations()
{
	g_station = StationTracker();
	g_sampleStation.clear();
	g_stationCount = 0;
	g_readyFracHigh = 0.0;
	g_lastSplitCheckTime = 0.0;
	g_splitFailStart = 0.0;
}

// Split-half reproducibility: solve the rotation on alternating stations and compare.
// This is the assumption-light test the residual cannot provide -- the residual is
// computed after three-plus nuisance parameters are refitted, so it absorbs most of a
// gross rotation error (a run ~17 degrees wrong still scored 9.7 mm). Two independent
// halves of the data disagreeing is direct evidence the solve is not determined,
// whatever the residual says.
//
// Returns false when the halves are too small to compare yet; `outDeg` receives the
// disagreement in degrees.
static bool SplitHalfRotationAgreement(const std::vector<Sample> &samples, double &outDeg)
{
	if (samples.size() != g_sampleStation.size())
		return false;

	std::vector<Sample> a, b;
	for (size_t i = 0; i < samples.size(); i++)
		((g_sampleStation[i] % 2) == 0 ? a : b).push_back(samples[i]);

	// Each half needs enough distinct poses to solve at all.
	if (a.size() < 24 || b.size() < 24)
		return false;

	const Eigen::Matrix3d ra = EulerDegToMatrix(CalibrateRotation(a, true));
	const Eigen::Matrix3d rb = EulerDegToMatrix(CalibrateRotation(b, true));
	outDeg = RotAngleBetween(ra, rb) * 180.0 / EIGEN_PI;
	return true;
}

// Largest disagreement between independent halves that still counts as converged.
// Tilt is worth ~28 mm of foot-vs-head displacement per degree, so this is set to keep
// the rotation contribution to a few mm rather than to any statistical convention.
static const double kMaxSplitHalfDeg = 1.5;

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

// Which room axis the accepted positions cover worst, as an actionable sentence, or nullptr
// when coverage is even enough that no single direction is worth singling out.
//
// Positional coverage is what makes scale observable, and "cover more of your play space" is
// not actionable advice to someone who has already walked a line across the room -- they have
// covered space, just not in the direction that is still degenerate. Room axes are used rather
// than principal axes deliberately: "step sideways" is something a person can do, an
// eigenvector is not.
static const char *WeakestSpreadAxis(const std::vector<Sample> &samples)
{
	if (samples.size() < 4)
		return nullptr;

	Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
	for (auto &s : samples)
		centroid += s.target.trans;
	centroid /= (double)samples.size();

	Eigen::Vector3d var = Eigen::Vector3d::Zero();
	for (auto &s : samples)
	{
		const Eigen::Vector3d d = s.target.trans - centroid;
		var += Eigen::Vector3d(d.x() * d.x(), d.y() * d.y(), d.z() * d.z());
	}
	var /= (double)samples.size();

	int worst = 0;
	if (var[1] < var[worst]) worst = 1;
	if (var[2] < var[worst]) worst = 2;

	// Only name an axis when coverage is genuinely lopsided. Vertical spread is always the
	// smallest for someone standing up, so a bare argmin would tell every user to crouch
	// forever; requiring a 4x deficit against the best axis keeps the advice meaningful.
	const double best = var.maxCoeff();
	if (best <= 0.0 || var[worst] > 0.25 * best)
		return nullptr;

	// OpenVR world axes: +x right, +y up, -z forward.
	switch (worst)
	{
	case 0:  return "Thin left-to-right - step sideways, pause, then look around";
	case 1:  return "Thin vertically - crouch low and stand tall, pausing at each height";
	default: return "Thin front-to-back - step forward and back, pause, then look around";
	}
}

// One averaged observation per station.
//
// The solvers treat every sample as an independent observation, and they are not: samples arrive
// in bursts of up to kMaxSamplesPerStation while the user holds still, so within a station they
// are near-duplicates with intra-cluster correlation ~1. EstimateHmdSpaceScale already knows
// this and patches over it, inflating the reported standard error by sqrt(n/K) after the fact
// (N3-c). Averaging each station down to one observation makes the independence assumption TRUE
// rather than corrected-for: within-station noise averages down by sqrt(members), the reported
// error bar becomes meaningful by construction, and the O(N^2) pair loops shrink by the square
// of the reduction (~200 samples -> ~30 stations is ~20,000 pairs -> ~435).
//
// It also deletes the pairs that carry no information: CalibrateRotation forms a delta for every
// pair, and a pair drawn from inside one station is two readings of the same pose, contributing
// noise with no signal.
//
// Rotations are averaged as sign-aligned quaternions. That linear approximation to the true
// Karcher mean is valid precisely because a station spans at most kStationAngRadius -- every
// member is within 18 degrees of the others, so there is no antipodal ambiguity and the error of
// the approximation sits far below the sample noise.
static std::vector<Sample> CollapseToStations(const std::vector<Sample> &samples,
	const std::vector<int> &station)
{
	// Partition unavailable or out of step: fall back to the raw set rather than inventing one.
	if (samples.empty() || samples.size() != station.size())
		return samples;

	int maxIdx = -1;
	for (int s : station)
		if (s > maxIdx)
			maxIdx = s;
	if (maxIdx < 0)
		return samples;

	struct Acc
	{
		Eigen::Vector3d refT = Eigen::Vector3d::Zero();
		Eigen::Vector3d tgtT = Eigen::Vector3d::Zero();
		Eigen::Quaterniond refQ = Eigen::Quaterniond(0, 0, 0, 0);
		Eigen::Quaterniond tgtQ = Eigen::Quaterniond(0, 0, 0, 0);
		int n = 0;
	};
	std::vector<Acc> acc((size_t)maxIdx + 1);

	for (size_t i = 0; i < samples.size(); i++)
	{
		const int s = station[i];
		if (s < 0 || (size_t)s >= acc.size())
			continue;
		Acc &a = acc[(size_t)s];

		a.refT += samples[i].ref.trans;
		a.tgtT += samples[i].target.trans;

		Eigen::Quaterniond qr(samples[i].ref.rot);
		Eigen::Quaterniond qt(samples[i].target.rot);
		// q and -q are the same rotation, so summing raw coefficients can cancel two identical
		// orientations to zero. Align each addend with the running sum first.
		if (a.n > 0)
		{
			if (a.refQ.coeffs().dot(qr.coeffs()) < 0.0)
				qr.coeffs() *= -1.0;
			if (a.tgtQ.coeffs().dot(qt.coeffs()) < 0.0)
				qt.coeffs() *= -1.0;
		}
		a.refQ.coeffs() += qr.coeffs();
		a.tgtQ.coeffs() += qt.coeffs();
		a.n++;
	}

	std::vector<Sample> out;
	out.reserve(acc.size());
	for (const Acc &a : acc)
	{
		if (a.n == 0)
			continue;
		// Degenerate sum (members cancelled despite alignment): drop the station rather than
		// normalising a near-zero quaternion into an arbitrary rotation.
		if (a.refQ.coeffs().norm() < 1e-9 || a.tgtQ.coeffs().norm() < 1e-9)
			continue;

		Pose r, t;
		r.trans = a.refT / (double)a.n;
		t.trans = a.tgtT / (double)a.n;
		r.rot = a.refQ.normalized().toRotationMatrix();
		t.rot = a.tgtQ.normalized().toRotationMatrix();
		out.push_back(Sample(r, t));
	}

	// Never hand back something the solvers cannot use.
	if (out.size() < 3)
		return samples;
	return out;
}

// Write the raw sample set to disk alongside the solve, one JSON object per line.
//
// Agents.md calls the missing replay harness "the binding constraint on estimator work" for the
// driver. The calibration solver has the identical problem and no equivalent: calibration.log
// records what a solve concluded, never what it was given, so a change to the solver can only
// be inspected, never measured. On a rig where scale is frozen for the whole session, an
// unmeasurable solver change is indistinguishable from the run-to-run scatter it is meant to
// remove. These dumps make a re-solve reproducible offline -- same input, N different solvers,
// compared against each other.
//
// Must be called BEFORE the sample set is rescaled by hmdScale: the point is the raw pairs.
// ~200 samples at ~160 bytes is ~32 KB per calibration, and kMaxTotalSamples bounds it, so
// this is not gated behind a toggle the way the multi-megabyte pose capture is.
static void DumpCalibrationSamples(const CalibrationContext &ctx,
	const std::vector<Sample> &samples)
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
	char stamp[32];
	snprintf(stamp, sizeof stamp, "%04d%02d%02d_%02d%02d%02d",
		st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);

	std::ofstream out(dir + "\\calsamples_" + stamp + ".jsonl");
	if (!out)
		return;

	// Header line. Everything a re-solve needs that is not per-sample: which devices, what the
	// run was configured as, and the prior scale the solve started from.
	out << std::fixed << std::setprecision(9)
		<< "{\"type\":\"meta\",\"schema\":1"
		<< ",\"hmd\":\"" << ctx.hmdSerial << "\""
		<< ",\"tracker\":\"" << ctx.trackerSerial << "\""
		<< ",\"speed\":" << (int)ctx.calibrationSpeed
		<< ",\"stations\":" << g_stationCount
		<< ",\"samples\":" << samples.size()
		<< ",\"targetModelScale\":" << ctx.targetModelScale
		<< ",\"priorHmdScale\":" << ctx.hmdScale
		<< ",\"priorHmdScaleN\":" << ctx.hmdScaleSamples
		<< "}\n";

	const bool haveStations = (g_sampleStation.size() == samples.size());
	for (size_t i = 0; i < samples.size(); i++)
	{
		// Quaternion rather than the 3x3: four numbers instead of nine, and unambiguous.
		const Eigen::Quaterniond qr(samples[i].ref.rot);
		const Eigen::Quaterniond qt(samples[i].target.rot);
		out << "{\"i\":" << i
			<< ",\"st\":" << (haveStations ? g_sampleStation[i] : -1)
			<< ",\"rp\":[" << samples[i].ref.trans.x() << "," << samples[i].ref.trans.y()
			<< "," << samples[i].ref.trans.z() << "]"
			<< ",\"rq\":[" << qr.w() << "," << qr.x() << "," << qr.y() << "," << qr.z() << "]"
			<< ",\"tp\":[" << samples[i].target.trans.x() << "," << samples[i].target.trans.y()
			<< "," << samples[i].target.trans.z() << "]"
			<< ",\"tq\":[" << qt.w() << "," << qt.x() << "," << qt.y() << "," << qt.z() << "]"
			<< "}\n";
	}
}

static void BeginSamplingPhase(CalibrationContext &ctx, uint32_t targetID)
{
	ctx.targetID = targetID;
	ctx.targetTrackingSystem = GetDeviceTrackingSystem(targetID);
	ctx.hmdSerial = GetDeviceSerial(vr::k_unTrackedDeviceIndex_Hmd);

	std::string newTrackerSerial = GetDeviceSerial(targetID);
	// The lever arm is rigidly tied to one physical tracker. Averaging across a tracker
	// swap would blend two unrelated mount points together (the 30 mm remount-distance
	// check only catches this by coincidence, if the new lever arm happens to differ
	// enough). Keying the reset on serial identity catches every swap, not just distant
	// ones.
	//
	// Two things this must NOT do, both fixed 2026-08-06:
	//
	// (a) It must not clear the stored offset *here*. WriteProfile only emits rel_*/lever_n
	//     under `if (ctx.validRelativeOffset)` (Configuration.cpp:221) while `validProfile`
	//     stays true, and SaveProfile is reachable before this run ever solves — from overlay
	//     shutdown (Main.cpp:365) and three UI toggles (UserInterface.cpp:168/181/229). Wiping
	//     at the start of sampling therefore *permanently* deleted a good lever arm from both
	//     the registry and the JSON mirror if the user swapped trackers and then closed the
	//     overlay mid-calibration. The swap is instead detected at solve time, where the
	//     replacement measurement actually exists: ComputeRelativeOffset compares the current
	//     tracker against ctx.leverSerial — the persisted owner of the stored average — so no
	//     transient state here has to survive an abandoned run or an overlay restart.
	//
	// (b) An empty serial is not a swap. GetDeviceSerial discards the ETrackedPropertyError
	//     and returns "" when the property read fails, which is routine for a device that has
	//     just woken or re-enumerated. Storing "" would also break serial-based target
	//     recovery below and the connected-tracker check in UserInterface.cpp.
	if (!newTrackerSerial.empty())
	{
		if (ctx.validRelativeOffset && !ctx.leverSerial.empty()
			&& newTrackerSerial != ctx.leverSerial)
		{
			ctx.Log("Target tracker changed - the head-tracker offset average will restart"
				" when this calibration solves.\n");
		}
		// Keep the live serial current — target recovery and the UI check both key off it.
		// The stored average stays attributed to ctx.leverSerial until a solve replaces it.
		ctx.trackerSerial = newTrackerSerial;
	}

	char buf[256];
	snprintf(buf, sizeof buf, "Using headset tracker: %s (id %d)\n",
		newTrackerSerial.empty() ? ctx.trackerSerial.c_str() : newTrackerSerial.c_str(), targetID);
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
	g_havePrevAccepted = false;
	g_rejRigidCount = 0;
	g_rejStaleCount = 0;
	g_frameBreakCount = 0;
	g_lossStart = 0.0;
	g_lossRunTicks = 0;
	g_lossTotalTicks = 0;
	g_gatesPassedTime = 0.0;
	g_plateauRefSpread = 0.0;
	g_plateauRefTime = 0.0;
	// Per-run solver telemetry; stale values would otherwise be attributed to this run.
	g_lastEarlyFinish = false;
	g_lastScaleSource = "unknown";
	g_lastScaleStdErr = -1.0;
	g_lastScaleStdErrEff = -1.0;
	g_lastScaleRaw = -1.0;
	g_lastScalePoolAction = "none";
	g_lastSolveN = 0;
	g_lastScalePairwise = -1.0;
	g_lastScalePairCount = 0;
	g_lastAxisVariance = 0.0;
	g_lastSplitHalfDeg = -1.0;
	g_lastStationCount = 0;
	g_lastTiltX = 0.0;
	g_lastTiltZ = 0.0;
	ctx.sampleHint = "Move your head slowly through different angles";
	ctx.sampleHintLevel = 1;
	ctx.Log("Starting calibration...\n");
}

static std::vector<Sample> collectedSamples;

void StartCalibration()
{
	CalCtx.state = CalibrationState::Begin;
	CalCtx.wantedUpdateInterval = 0.0;
	CalCtx.messages.clear();
	Detection.Clear();
	collectedSamples.clear();
	g_havePrevSample = false;
	ResetStations();
}

static void AbortAndRestoreProfile(CalibrationContext &ctx)
{
	if (ctx.targetID != vr::k_unTrackedDeviceIndexInvalid)
		ResetAndDisableOffsets(ctx.targetID);

	// An abort is supposed to restore the CALIBRATION. LoadProfile restores the whole profile,
	// and ParseProfile unconditionally rewrites every Settings/Smoothing field too
	// (Configuration.cpp: native/fallbackSlam/eAngVel, continuousSync, predictionTime,
	// headFilterEnabled + both One-Euro sets, calibration_speed). None of those controls call
	// SaveProfile at the point of change -- persistence happens only at solve time or at
	// overlay exit -- so an abort silently threw away whatever the user had just set, and the
	// message said only "Previous calibration restored".
	//
	// That is not cosmetic: continuousSync reverting to the stored value makes the next 1 Hz
	// ScanAndApplyProfile push sync=false to every body puck, so the pucks lose drift
	// correction while the UI still shows the setting the user chose. Aborts are routine on
	// this rig (tracking_lost on a single-base-station room, rms_gate on a relocalisation), so
	// this fires often. Snapshot the non-calibration fields across the reload and put them back.
	const bool sNative = ctx.enableNative;
	const bool sFallback = ctx.fallbackToSlam;
	const bool sEAngVel = ctx.enableAngularVelocity;
	const bool sSync = ctx.continuousSync;
	const double sPred = ctx.predictionTime;
	const bool sHeadFilt = ctx.headFilterEnabled;
	const protocol::OneEuroParams sHeadP = ctx.headFilterParams;
	const protocol::OneEuroParams sDriftP = ctx.driftFilterParams;
	const CalibrationContext::Speed sSpeed = ctx.calibrationSpeed;

	LoadProfile(ctx);

	ctx.enableNative = sNative;
	ctx.fallbackToSlam = sFallback;
	ctx.enableAngularVelocity = sEAngVel;
	ctx.continuousSync = sSync;
	ctx.predictionTime = sPred;
	ctx.headFilterEnabled = sHeadFilt;
	ctx.headFilterParams = sHeadP;
	ctx.driftFilterParams = sDriftP;
	ctx.calibrationSpeed = sSpeed;

	ctx.state = CalibrationState::None;
	collectedSamples.clear();
	ResetStations();
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
			LogCalibrationOutcome(ctx, "abort", "tracker_id_ambiguous",
				0, 0, 0.0, 0.0, 0, 0, 0);
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
		// Relaxed from 1.5 / 0.25. The old linear limit rejected anything faster than a
		// shuffle, so repositioning around the room collected nothing and the bar
		// appeared to stall every time the user did the one thing scale needs. What the
		// gate is really bounding is v * (tracker-vs-HMD latency skew) baked into the
		// solved offset -- and in fusion mode the EKF re-anchors translation
		// continuously at runtime, so that particular error is absorbed rather than
		// permanent. Trading a few mm of translation skew for samples collected while
		// actually moving is the right side of that bargain: it is what makes adequate
		// positional spread reachable at all.
		const double kMaxSampleAngSpeed = 2.0;  // rad/s
		const double kMaxSampleLinSpeed = 0.35; // m/s; a slow walk, not a shuffle
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
		// Coast. Only a loss that outlasts kMaxTrackingLossSec ends the run; anything shorter
		// costs a few samples and nothing else.
		if (g_lossStart == 0.0)
			g_lossStart = time;
		++g_lossRunTicks;
		++g_lossTotalTicks;

		const double lostSec = time - g_lossStart;
		if (lostSec >= kMaxTrackingLossSec)
		{
			char buf[256];
			snprintf(buf, sizeof buf,
				"%s stopped tracking for %.1f s - aborting calibration. Previous calibration restored.\n",
				g_lastCollectFailure ? g_lastCollectFailure : "A device", lostSec);
			ctx.Log(buf);
			// Log AFTER the restore, matching the rms_gate abort: hmdScale=/trim= on an abort
			// line must mean "what the user is left running" on BOTH abort paths. This one
			// logged pre-restore, so the same field meant two different things depending on
			// the abort reason — which corrupted the log as an analysis input (the scale-pool
			// replay script mis-read exactly these lines; audit critic P4/P5). Coverage
			// fields are captured first, since the restore clears the sample set.
			const int lossAccepted = (int)collectedSamples.size();
			const int lossStations = g_stationCount;
			const double lossSpread = TargetSpread(collectedSamples);
			const double lossAxis = SecondAxisVariance(collectedSamples);
			AbortAndRestoreProfile(ctx);
			LogCalibrationOutcome(ctx, "abort", "tracking_lost",
				lossAccepted, lossStations, lossSpread, lossAxis,
				g_rejAngCount, g_rejLinCount, g_lossTotalTicks);
			return;
		}

		if ((time - g_lastHintTime) > 0.3)
		{
			g_lastHintTime = time;
			ctx.sampleHint = "Tracking dropped - hold still until it comes back";
			ctx.sampleHintLevel = 2;
		}
		return;
	}

	if (g_lossRunTicks > 0)
	{
		// Recovered. Worth one line: a run that survived several dropouts has gaps in its
		// coverage that no coverage metric in the final record would otherwise reveal.
		char buf[160];
		snprintf(buf, sizeof buf,
			"Tracking recovered after %d dropped frame(s).\n", g_lossRunTicks);
		ctx.Log(buf);
		g_lossRunTicks = 0;
	}
	g_lossStart = 0.0;

	// ---- Rigid-pair consistency (N-cal-a) -------------------------------------------------
	//
	// The solve assumes ref (SLAM) and target (lighthouse) are two rigidly-related spaces for
	// the whole run, and until now nothing checked that. The V3-e speed gate above cannot: it
	// updates g_prevSampleHmdPos before returning, so on a mid-run frame shift the samples on
	// BOTH sides are accepted and get solved together. Worse, a frozen pose source reads as
	// zero speed, so that gate preferentially accepts exactly the stale samples.
	//
	// Measured on this rig, replaying the seven calsamples_20260816_* runs:
	//   run      logged rms_mm   rigid rejects   frozen   discontinuity
	//   022323   3.1                 0             0        -
	//   022422   4.6                 1             0        -
	//   023412   2.7                 0             0        -
	//   161436   4.3                 0             0        -
	//   161724   1.7                 0             0        -
	//   023120   15.9  (worst)       2             0        -
	//   165347   abort rms_gate      2             7        i=210, residual 0.660 m
	// Benign runs sit at p99 <= 0.009 m and max <= 0.034 m; the 165347 break is 0.660 m, and
	// its seven frozen samples carry a bit-identical HMD pose while the tracker moved 3-4 cm.
	// So the thresholds below are ~2x above the worst benign p99 and ~3x below the observed
	// break, and cost 0-2 samples out of 220-515 on runs that were fine.
	//
	// What 165347 cost: two internally-clean solves 0.76 m and 3 deg apart were averaged into
	// one, produced 127-237 mm RMS, and the user lost a 428-sample run to "reason=rms_gate"
	// with nothing in the log saying why. The silent case is worse -- a 1 cm shift passes every
	// existing gate and writes hmdScale +1.22% off, which is frozen for the session.
	{
		const double kStaleTgtMotionM  = 0.005;  // tracker moved this far while HMD did not move at all
		const double kRigidRejectM     = 0.020;  // drop the sample
		const double kFrameBreakM      = 0.100;  // the reference frame moved; everything so far is unusable

		if (g_havePrevAccepted)
		{
			const double dRef = (sample.ref.trans - g_prevAcceptedRef).norm();
			const double dTgt = (sample.target.trans - g_prevAcceptedTgt).norm();
			const double dTheta = RotAngleBetween(sample.target.rot, g_prevAcceptedTgtRot);
			const double lever = ctx.validRelativeOffset
				? Vec3Norm(ctx.relativeTranslation)
				: 0.15;   // pre-first-calibration fallback, deliberately generous
			const double bound = 2.0 * lever * std::sin(dTheta * 0.5);
			const double residual = std::fabs(dRef - dTgt) - bound;

			// Bit-identical HMD pose while the tracker demonstrably moved. Unambiguous: the
			// pose source stalled. No threshold needed, and it is the streamed-headset failure
			// that precedes the jump.
			if (dRef == 0.0 && dTgt > kStaleTgtMotionM)
			{
				++g_rejStaleCount;
				ctx.sampleHint = "Headset pose is frozen - samples paused until it updates";
				ctx.sampleHintLevel = 2;
				return;
			}

			if (residual > kFrameBreakM)
			{
				// The pre-shift samples cannot be reconciled with the post-shift ones by any
				// rigid transform, so keeping them is what produces the two-solves-averaged
				// failure. Discard and restart collection rather than abort: the user is
				// already standing there, and the run is recoverable if the frame settles.
				++g_frameBreakCount;
				char fb[256];
				snprintf(fb, sizeof fb,
					"Headset tracking origin shifted %.2f m mid-calibration (relocalisation).\n"
					"Discarding the %d samples taken before it and starting the collection over.\n",
					residual, (int)collectedSamples.size());
				ctx.Log(fb);
				collectedSamples.clear();
				g_sampleStation.clear();
				ResetStations();
				g_havePrevAccepted = false;
				ctx.sampleHint = "Headset re-centred itself - restarting, keep moving";
				ctx.sampleHintLevel = 2;
				return;
			}

			if (residual > kRigidRejectM)
			{
				++g_rejRigidCount;
				return;
			}
		}

		g_prevAcceptedRef = sample.ref.trans;
		g_prevAcceptedTgt = sample.target.trans;
		g_prevAcceptedTgtRot = sample.target.rot;
		g_havePrevAccepted = true;
	}

	auto &samples = collectedSamples;
	// Hard ceiling. Solves are O(N^2). Once full, do not append; still run finish logic.
	static const size_t kMaxTotalSamples = 600;
	const bool atSampleCap = samples.size() >= kMaxTotalSamples;

	// Assign this sample to a station, and drop it if the station is already saturated.
	// This is what stops standing still from filling the progress bar: the counter now
	// advances only when the head actually occupies a new pose.
	const bool newStation = !g_station.have
		|| (sample.ref.trans - g_station.pos).norm() > kStationPosRadius
		|| RotAngleBetween(sample.ref.rot, g_station.rot) > kStationAngRadius;

	if (newStation)
	{
		g_station.have = true;
		g_station.pos = sample.ref.trans;
		g_station.rot = sample.ref.rot;
		g_station.count = 0;
		// At the sample ceiling nothing further is kept, so counting new stations would
		// inflate stationFrac, stationsOk and the logged stations= field past the sample
		// set actually being solved. Track the pose so hinting still works; don't count it.
		if (!atSampleCap)
			g_station.index = g_stationCount++;
	}
	bool redundantSample = false;
	if (!newStation && g_station.count >= kMaxSamplesPerStation && !atSampleCap)
	{
		// Redundant pose: don't append, but DO fall through to the finish logic below. This
		// used to `return`, which meant the plateau finish and its 20 s ceiling were only ever
		// evaluated on an APPENDED sample — a user who completed coverage and then held still
		// (exactly what the sampleHint tells them to do) parked in a saturated station and the
		// run could not finish until they moved again. Audit 2026-08-16, refuter-verified:
		// no other exit existed.
		redundantSample = true;
		if ((time - g_lastHintTime) > 0.3 && (time - g_lastFastDropTime) > 0.8)
		{
			g_lastHintTime = time;
			ctx.sampleHint = "Got this spot - move somewhere else, or look a different way";
			ctx.sampleHintLevel = 1;
		}
	}

	if (!atSampleCap && !redundantSample)
	{
		g_station.count++;
		g_sampleStation.push_back(g_station.index);
		samples.push_back(sample);
	}
	if (samples.empty())
		return;

	// Three independent readiness fractions. The bar tracks the *weakest* gate so it
	// cannot hit 100% while scale is still unobservable (spread) or translation Y is
	// in the null space (yaw-only motion → low axis variance). Station count alone used
	// to complete runs in ~20s with scale_src=kept_low_spread and a frozen residual offset.
	const size_t stationTarget = CalCtx.StationTarget();
	const double spreadNow = TargetSpread(samples);
	const double axisNow = SecondAxisVariance(samples);
	g_lastSampleTarget = stationTarget;
	g_lastAxisVariance = axisNow;
	g_lastScaleSpread = spreadNow;
	g_lastFinishSpread = spreadNow;   // survives the solve's overwrite of g_lastScaleSpread

	// (std::min) avoids Windows.h min/max macros.
	const double stationDenom = stationTarget > 0 ? (double)stationTarget : 1.0;
	const double stationFrac = (std::min)(1.0, (double)g_stationCount / stationDenom);
	const double spreadFrac = (std::min)(1.0, spreadNow / ScaleSpreadThreshold);
	const double axisFrac = (std::min)(1.0, axisNow / AxisVarianceThreshold);
	const double readyFrac = (std::min)(stationFrac, (std::min)(spreadFrac, axisFrac));
	// Display only, and held at its high-water mark. spreadFrac comes from TargetSpread,
	// an RMS about the centroid, so pausing to accumulate samples in one spot after
	// walking the room genuinely lowers it — and a progress bar that visibly runs
	// backwards reads as a fault. The finish decision below uses the live gates
	// (stationsOk/axisOk/spreadOk), never this value, so holding the bar cannot let a
	// transient peak complete a run on coverage that has since decayed.
	g_readyFracHigh = (std::max)(g_readyFracHigh, readyFrac);
	// 0..1000 keeps integer Progress() smooth with three continuous gates.
	CalCtx.Progress((int)(g_readyFracHigh * 1000.0 + 0.5), 1000);

	// Live guidance: after each accepted sample, tell the user what the solver still
	// needs, so deficient motion is corrected during sampling instead of only being
	// reported after a full batch (which then discards a quarter and restarts).
	// Throttled, and it does not override a fresh "slow down" message.
	if ((time - g_lastHintTime) > 0.3 && (time - g_lastFastDropTime) > 0.8)
	{
		g_lastHintTime = time;
		if (stationFrac < 0.25)
		{
			ctx.sampleHint = "Move your head slowly through different angles";
			ctx.sampleHintLevel = 1;
		}
		else if (axisFrac < 1.0)
		{
			ctx.sampleHint = "Add rotation variety - tilt ear-to-shoulder and look up/down, not just left and right";
			ctx.sampleHintLevel = 1;
		}
		else if (spreadFrac < 1.0)
		{
			// Samples are only taken while you are still, so the motion that works is
			// move-then-pause rather than continuous walking. Name the deficient direction
			// when there is one; fall back to the general advice when coverage is even.
			const char *axisHint = WeakestSpreadAxis(samples);
			ctx.sampleHint = axisHint
				? axisHint
				: "Walk to another spot or crouch, pause, look around - need more space coverage for scale";
			ctx.sampleHintLevel = 1;
		}
		else if (stationFrac < 1.0)
		{
			ctx.sampleHint = "Looking good - keep covering new angles/spots until the bar fills";
			ctx.sampleHintLevel = 0;
		}
		else
		{
			// Gates are met and the run is in its plateau phase. Say so honestly: the bar is
			// full but more coverage still buys scale precision, and the old text ("finishing
			// calibration") told the user to stop moving at exactly the wrong moment.
			ctx.sampleHint = "Coverage met - keep moving to sharpen the scale fit";
			ctx.sampleHintLevel = 0;
		}
	}

	// Finish only when all three gates pass:
	//   stations  — enough distinct poses (not micro-yaw duplicates)
	//   axis var  — second principal component of orientation (N3-a; yaw-only is insufficient)
	//   spread    — positional RMS that makes scale/lever observable (else kept_low_spread)
	// Split-half remains diagnostic-only (Euler twin ambiguity made it a false positive).
	bool readyToSolve = false;
	const bool stationsOk = (size_t)g_stationCount >= stationTarget;
	const bool axisOk = axisNow >= AxisVarianceThreshold;
	const bool spreadOk = spreadNow >= ScaleSpreadThreshold;

	if (stationsOk && (time - g_lastSplitCheckTime) >= 1.0)
	{
		g_lastSplitCheckTime = time;
		double splitDeg = 0.0;
		if (SplitHalfRotationAgreement(samples, splitDeg))
			g_lastSplitHalfDeg = splitDeg;
	}

	if (stationsOk && axisOk && spreadOk)
	{
		// Gates met -- but meeting them is the floor, not the target. Scale precision goes as
		// 1/spread, and solving the instant the bar filled is what pinned 12 of the last 14
		// runs on this rig between 0.150 and 0.199 m against a 0.150 m gate. Keep collecting
		// while the user is still covering new ground.
		if (g_gatesPassedTime == 0.0)
		{
			g_gatesPassedTime = time;
			g_plateauRefSpread = spreadNow;
			g_plateauRefTime = time;
			char okBuf[240];
			snprintf(okBuf, sizeof okBuf,
				"Coverage met: %d spots, spread %.2f m, axis_var %.5f, %zu samples."
				" Still collecting while coverage improves.\n",
				g_stationCount, spreadNow, axisNow, samples.size());
			CalCtx.Log(okBuf);
		}

		// "Time since the last meaningful improvement", not "improvement over the last window":
		// spread is an RMS about a moving centroid, so it dips whenever samples land near the
		// middle. Measuring against the best figure reached so far makes a dip cost nothing and
		// only genuine new coverage restart the clock.
		if (spreadNow > g_plateauRefSpread * (1.0 + kPlateauMinGainFrac))
		{
			g_plateauRefSpread = spreadNow;
			g_plateauRefTime = time;
		}

		const double heldSec = time - g_gatesPassedTime;
		const bool plateaued = (time - g_plateauRefTime) >= kPlateauWindowSec;
		const bool outOfTime = heldSec >= kPlateauMaxExtraSec;

		if (plateaued || outOfTime || atSampleCap)
		{
			char doneBuf[256];
			snprintf(doneBuf, sizeof doneBuf,
				"Coverage ready: %d spots, spread %.2f m (best %.2f m), axis_var %.5f, %zu samples"
				" after %.0f s extra (%s).\n",
				g_stationCount, spreadNow, g_plateauRefSpread, axisNow, samples.size(), heldSec,
				plateaued ? "improvement plateaued" : (outOfTime ? "time limit" : "sample ceiling"));
			CalCtx.Log(doneBuf);
			readyToSolve = true;
		}
	}
	else if (atSampleCap && stationsOk)
	{
		// Prefer a best-effort solve over an infinite stuck bar. RMS gate can still abort.
		char capBuf[192];
		snprintf(capBuf, sizeof capBuf,
			"Sample ceiling (%zu) with incomplete coverage (spots=%d spread=%.2f axis=%.5f) - solving anyway.\n",
			kMaxTotalSamples, g_stationCount, spreadNow, axisNow);
		CalCtx.Log(capBuf);
		readyToSolve = true;
	}

	if (readyToSolve)
	{
		CalCtx.Log("\n");
			g_lastStationCount = g_stationCount;

		// Before anything mutates the set (the hmdScale rescale below does).
		DumpCalibrationSamples(ctx, samples);

		// Everything downstream solves on ONE observation per station rather than on the raw
		// burst. `samples` stays untouched as the record of what was collected; `solveSet` is
		// what the estimators see.
		std::vector<Sample> solveSet = CollapseToStations(samples, g_sampleStation);
		g_lastSolveN = (int)solveSet.size();
		{
			char collapseBuf[192];
			snprintf(collapseBuf, sizeof collapseBuf,
				"Solving on %d station-averaged observations (from %zu raw samples).\n",
				g_lastSolveN, samples.size());
			CalCtx.Log(collapseBuf);
		}

		ctx.calibratedRotation = CalibrateRotation(solveSet);

		// Both spaces are gravity-referenced -- lighthouse levels off the base station's
		// accelerometer, SLAM off the headset IMU -- so the true relative rotation is a
		// heading, and any pitch/roll in the solve is error unless it exceeds each
		// device's gravity accuracy.
		//
		// Measured here, that error is large: two calibrations taken 18 seconds apart in
		// one session disagreed by up to 1.68 degrees of tilt, which no gravity reference
		// can physically do. Per-run sigma is ~0.67 deg, which accounts for all of the
		// observed run-to-run scatter, and the pooled mean is consistent with zero. The
		// mechanism is a pose-dependent attitude error, so it does NOT average down with
		// more samples or more varied motion -- collecting more data cannot fix it.
		//
		// Estimating tilt therefore has mean squared error ~(0.67 deg)^2, while forcing it
		// to zero has at most (0.35 deg)^2 given the bound on any true tilt. Zeroing wins
		// by ~2x in RMS, which is ~19 mm -> <=10 mm of foot-vs-head displacement through a
		// 1.6 m torso. Tilt is not corrected at runtime (quaternionProjectYaw discards
		// everything but yaw), so this is the single largest frozen error in the system.
		//
		// The signed tilt is still logged, as a diagnostic. If it ever proves repeatable
		// and larger than ~0.4 deg across many sessions, it belongs in a stored per-mount
		// constant -- never in a per-session solve.
		{
			double droppedX = 0.0, droppedZ = 0.0;
			SignedTiltDeg(ctx.calibratedRotation, droppedX, droppedZ);
			// Persist before zeroing; calibration.log is the only place this survives.
			g_lastTiltX = droppedX;
			g_lastTiltZ = droppedZ;

			const Eigen::Matrix3d full = EulerDegToMatrix(ctx.calibratedRotation);
			const double yawDeg = std::atan2(full(0, 2), full(0, 0)) * 180.0 / EIGEN_PI;
			ctx.calibratedRotation = Eigen::Vector3d(0.0, yawDeg, 0.0);

			char tiltBuf[192];
			snprintf(tiltBuf, sizeof tiltBuf,
				"Levelled calibration to gravity: heading %.2f deg, discarded tilt %.2f / %.2f deg.\n",
				yawDeg, droppedX, droppedZ);
			CalCtx.Log(tiltBuf);
		}

		Eigen::Vector3d eulerRad = ctx.calibratedRotation * EIGEN_PI / 180.0;
		Eigen::Matrix3d calRot =
			(Eigen::AngleAxisd(eulerRad(0), Eigen::Vector3d::UnitZ()) *
			 Eigen::AngleAxisd(eulerRad(1), Eigen::Vector3d::UnitY()) *
			 Eigen::AngleAxisd(eulerRad(2), Eigen::Vector3d::UnitX())).toRotationMatrix();

		// calScale is the SOLVE's internal scale (always 1.0 — the fit does not estimate one).
		// ctx.calibratedScale is a DIFFERENT quantity: the user's manual body-scale trim, set
		// only in the UI. This line used to conflate them — `ctx.calibratedScale = calScale` —
		// silently resetting the trim to 1.0 on every successful solve and persisting the reset
		// via the SaveProfile below, with no log token and no message. The solve has no
		// information about the trim (it fits rotation/translation), so it has no business
		// writing it: the identical reasoning the hmdScale comment below applies to hmdScale
		// ("keep the previously measured value rather than destroying it with a hard 1.0") and
		// the same policy A3 applies to every other user-set field across an abort. The live
		// profile's own timeline was the evidence: trim re-entered by hand after a solve wiped
		// it, repeatedly. 2026-08-17 audit synthesis R1 / register A12.
		double calScale = 1.0;
		ctx.targetModelScale = GetLighthouseModelScale(ctx.targetID);

		// Scale is the one calibrated quantity the runtime cannot re-estimate while playing
		// (fusion applies hmdScale directly; the EKF carries yaw+translation state, no scale).
		// If this run lacks the positional spread to observe scale, keep the previously measured
		// value rather than destroying it with a hard 1.0 -- StartCalibration does not reset
		// ctx.hmdScale, so it still holds the last persisted scale at this point.
		const double priorHmdScale = ctx.hmdScale;
		const double fittedHmdScale =
			EstimateHmdSpaceScale(solveSet, calRot, ctx.targetModelScale, priorHmdScale);
		// Independent cross-check, logged only (see PairwiseBaselineScale). If these two agree
		// across a few real calibrations, the pairwise estimator is the better primary; if they
		// diverge, the log says so before anything has been staked on it.
		g_lastScalePairwise = PairwiseBaselineScale(solveSet,
			Eigen::Vector3d(ctx.relativeTranslation.v[0], ctx.relativeTranslation.v[1],
				ctx.relativeTranslation.v[2]),
			ctx.validRelativeOffset, g_lastScalePairCount);
		// Sets ctx.hmdScale: to the running average when this run measured scale, or straight
		// back to the prior when it did not. The translation solve below then runs against the
		// scale that actually ships, so the two stay consistent.
		PoolHmdScale(ctx, fittedHmdScale);

		for (auto &sample : solveSet)
			sample.ref.trans /= ctx.hmdScale;

		ctx.calibratedTranslation = CalibrateTranslation(solveSet, calRot, calScale);
		Eigen::Vector3d calTransM = ctx.calibratedTranslation * 0.01;

		Eigen::Vector3d hmdToTarget = ComputeRefToTargetOffset(solveSet, calRot, calTransM, calScale);
		double rmsError = RetargetingErrorRMS(solveSet, hmdToTarget, calRot, calTransM, calScale);

		char buf2[256];
		snprintf(buf2, sizeof buf2, "Calibration residual error (RMS): %.1f mm\n", rmsError * 1000.0);
		CalCtx.Log(buf2);

		// Written as !(x <= limit) so a NaN residual fails the gate. `rmsError > 0.1`
		// is false for NaN, which would have let a degenerate solve through to
		// SaveProfile and then into the pose pipeline.
		// NaN backstop, nothing more. Written as !(x <= limit) so a NaN residual fails.
		// This was briefly tightened to 12 mm on the belief that a run with a 9.7 mm
		// residual had been ~17 degrees wrong; that run was in fact 1.79 degrees from its
		// neighbour, so the tightening rested on a misreading and has been reverted. The
		// residual is refitted after six nuisance parameters and cannot measure accuracy
		// in either direction -- it should not be used as a quality claim.
		if (!(rmsError <= 0.1))
		{
			CalCtx.Log("Calibration quality is too low, aborting! Previous calibration restored. Try again with a slower calibration speed, moving smoothly.\n");
			// Capture coverage before the restore: AbortAndRestoreProfile clears the sample set
			// and the station partition, and LoadProfile rolls ctx.hmdScale back to the stored
			// value -- which is what we want the record to show, since the rejected fit is not
			// what the user is left running.
			const int outAccepted = (int)samples.size();
			const int outStations = g_stationCount;
			const double outSpread = g_lastScaleSpread;
			const double outAxis = g_lastAxisVariance;
			AbortAndRestoreProfile(ctx);
			LogCalibrationOutcome(ctx, "abort", "rms_gate", outAccepted, outStations,
				outSpread, outAxis, g_rejAngCount, g_rejLinCount, g_lossTotalTicks);
			return;
		}

		ComputeRelativeOffset(ctx, solveSet, calRot, calTransM, calScale);

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
