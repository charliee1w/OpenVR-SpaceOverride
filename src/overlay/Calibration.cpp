// SPDX-License-Identifier: AGPL-3.0-only

#define WIN32_LEAN_AND_MEAN

#include "Calibration.h"
#include "Configuration.h"
#include "IPCClient.h"

#include <string>
#include <vector>
#include <iostream>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <chrono>
#include <limits>
#include <fstream>
#include <iomanip>

#include <windows.h>

#include <Dense>


static IPCClient Driver;
CalibrationContext CalCtx;

// ---------------------------------------------------------------------------
// B3: calibration.log and raw sample dumps.
//
// ACCEPTANCE (why the happy-path output is bit-identical): everything in this
// block is write-only. The statics below are assigned inside the solver and read
// back only by the two log writers; no solver, gate or published value ever reads
// one. The writers take const references, allocate their own streams, and are
// called at points where the calibration result is already final. Removing every
// line of it would leave the same profile, the same driver messages and the same
// poses -- only the files on disk would differ.
//
// Why it exists: a calibration result lived only in the overlay's in-VR message
// list, so it vanished when the list scrolled or the overlay closed. When two
// calibrations disagree -- which is the whole question this fork exists to
// settle -- there was nothing to compare. calibration.log gives one appended line
// per run, success or failure; the .jsonl dumps give the solver's actual input so
// a run can be re-solved offline against a different estimator.
//
// Deliberately absent versus the fork's version of this file: the split-half
// rotation agreement check (two extra O(N^2) solves per second on the overlay
// main loop) and every station/pooling field, none of which exist here.
// ---------------------------------------------------------------------------

// Where the shipped headset scale came from. The value alone cannot show this: a
// measured 0.996 and a defaulted 1.0 look like ordinary numbers, and upstream has
// three separate branches that silently return 1.0. Written at each exit of
// EstimateHmdSpaceScale, read only by LogCalibrationResult.
static const char *g_lastScaleSource = "unknown";
// The scale this run actually fitted, including a fit that was then rejected as
// implausible (-1 when no fit was attempted). The rejected value is the useful
// one: it says how far off the solve was, which "assuming 1" does not.
static double g_lastScaleRaw = -1.0;
// Positional spread of the target samples, in metres, as computed by the scale
// fit. Logged because it is the quantity the scale_src="low_spread" branch tested
// against ScaleSpreadThreshold -- without it that outcome has no number.
static double g_lastScaleSpread = 0.0;

// How far the calibration tips the vertical axis, in degrees. Both spaces are
// gravity-levelled, so the true relative rotation has yaw as its only free degree
// of freedom and this should be ~0; what it actually is, is tilt -- either solver
// noise or a genuinely un-level room, which repeated runs tell apart.
//
// MEASURED, NOT APPLIED. This build keeps upstream's solved rotation exactly as
// solved, tilt included; the number is recorded so the question "is the tilt in
// this room real, or is it noise?" can be settled from the log instead of from a
// code change. Nothing reads tilt_x/tilt_z back.
//
// Computed from the rotation matrix rather than from the Euler angles: pitch and
// roll near zero and near +/-180 can describe the same physical rotation with a
// compensating yaw, so a per-angle "distance from the expected flip" reports ~179
// degrees for a perfectly ordinary solve.
static Eigen::Matrix3d EulerDegToMatrix(const Eigen::Vector3d &eulerDeg)
{
	const Eigen::Vector3d r = eulerDeg * EIGEN_PI / 180.0;
	return (Eigen::AngleAxisd(r(0), Eigen::Vector3d::UnitZ()) *
		Eigen::AngleAxisd(r(1), Eigen::Vector3d::UnitY()) *
		Eigen::AngleAxisd(r(2), Eigen::Vector3d::UnitX())).toRotationMatrix();
}

// Signed, two-component. A magnitude (acos of an absolute cosine) is strictly
// non-negative and therefore positively biased: fed pure zero-mean noise it
// reports a healthy-looking average tilt forever and can never average to zero,
// so it could never answer "is there a real tilt here" with no. Signed components
// average to zero under noise and to the true tilt under a real one.
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

// Resolve the log directory next to the driver's session logs, creating it if
// needed. Returns false when LOCALAPPDATA is unusable, in which case the caller
// writes nothing at all rather than half a record.
static bool CalibrationLogDir(std::string &dir)
{
	char localAppData[MAX_PATH] = {};
	DWORD n = GetEnvironmentVariableA("LOCALAPPDATA", localAppData, MAX_PATH);
	if (n == 0 || n >= MAX_PATH)
		return false;

	const std::string root = std::string(localAppData) + "\\OpenVR-SpaceOverride";
	CreateDirectoryA(root.c_str(), nullptr);
	dir = root + "\\logs";
	CreateDirectoryA(dir.c_str(), nullptr);
	return true;
}

// Shared by the success and failure records: open for append and write the
// timestamp prefix.
static bool OpenCalibrationLog(std::ofstream &out)
{
	std::string dir;
	if (!CalibrationLogDir(dir))
		return false;

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
// A log of successful solves only is exactly backwards: a user reporting
// "calibration keeps failing" then produces a file containing nothing but clean
// successes, and the runs worth a record are the ones that died. `accepted` is
// passed in rather than read from a file static so this can be called from abort
// paths that run before, during or after sampling.
static void LogCalibrationOutcome(const CalibrationContext &ctx, const char *result,
	const char *reason, size_t accepted, int droppedTicks)
{
	std::ofstream out;
	if (!OpenCalibrationLog(out))
		return;

	out << std::setprecision(3)
		<< "  result=" << result
		<< "  reason=" << reason
		<< "  accepted=" << accepted
		<< "  target=" << ctx.SampleCount()
		<< "  speed=" << (int)ctx.calibrationSpeed
		<< "  dropped=" << droppedTicks
		// The scale and trim the run was ABOUT to modify, so an abort can be told
		// apart from a run that never got far enough to threaten them.
		<< std::setprecision(5)
		<< "  hmdScale=" << ctx.hmdScale
		<< "  trim=" << ctx.calibratedScale
		<< "  hmd=" << ctx.hmdSerial
		<< "  tracker=" << ctx.trackerSerial
		<< "\n";
}

// Records a calibration that produced a profile.
static void LogCalibrationResult(const CalibrationContext &ctx, double rmsErrorMm,
	size_t accepted, int droppedTicks)
{
	double tiltX = 0.0, tiltZ = 0.0;
	SignedTiltDeg(ctx.calibratedRotation, tiltX, tiltZ);

	std::ofstream out;
	if (!OpenCalibrationLog(out))
		return;

	out << std::fixed
		<< "  result=ok"
		<< "  rms_mm=" << std::setprecision(1) << rmsErrorMm
		<< "  accepted=" << accepted
		<< "  target=" << ctx.SampleCount()
		<< "  speed=" << (int)ctx.calibrationSpeed
		<< "  dropped=" << droppedTicks
		// Solved rotation as published, plus the pitch/roll tilt away from level.
		// If tilt_* scatters run to run it is solver noise; if it repeats, it is a
		// real tilt between the two spaces. This build applies it either way.
		<< std::setprecision(2)
		<< "  yaw=" << ctx.calibratedRotation(1)
		<< "  pitch=" << ctx.calibratedRotation(2)
		<< "  roll=" << ctx.calibratedRotation(0)
		<< "  tilt_x=" << tiltX
		<< "  tilt_z=" << tiltZ
		// hmdScale is what ships; scale_raw is what this run fitted (including a
		// fit that was rejected), and scale_src says which of upstream's four
		// exits produced the shipped number.
		<< std::setprecision(5)
		<< "  hmdScale=" << ctx.hmdScale
		<< "  scale_raw=" << g_lastScaleRaw
		<< "  scale_src=" << g_lastScaleSource
		<< "  spread_m=" << std::setprecision(3) << g_lastScaleSpread
		<< std::setprecision(5)
		<< "  trim=" << ctx.calibratedScale
		<< "  targetModelScale=" << ctx.targetModelScale
		// The head-tracker lever arm this run measured. Nothing averages it here,
		// so raw is also what ships -- and its run-to-run scatter is the number
		// that decides whether averaging would be worth having.
		<< "  lever_raw_mm=" << std::setprecision(1)
		<< ctx.relativeTranslation.v[0] * 1000.0 << "/"
		<< ctx.relativeTranslation.v[1] * 1000.0 << "/"
		<< ctx.relativeTranslation.v[2] * 1000.0
		<< "  lever_valid=" << (ctx.validRelativeOffset ? 1 : 0)
		<< "  hmd=" << ctx.hmdSerial
		<< "  tracker=" << ctx.trackerSerial
		<< "\n";
}

// ---------------------------------------------------------------------------
// A13: desired driver state, written once instead of disable-then-enable.
//
// ACCEPTANCE (why the happy-path output is bit-identical): the TERMINAL state
// per device after a scan is provably identical in all six branches of
// ScanAndApplyProfile -- HMD -> disabled; !enabled -> disabled; property error
// -> disabled; the target head tracker -> disabled; same tracking system ->
// enabled with the UNCHANGED deviceScale = calibratedScale * model(id) /
// targetModelScale; different tracking system -> disabled. The messages the
// driver ends up holding are byte-for-byte upstream's. What disappears is the
// sub-millisecond window each second in which upstream had every device
// disabled, because it sent RequestSetDeviceTransform(enabled=false) to all of
// them and only then re-enabled the ones that should be on. That window raced
// the pose thread and briefly published uncalibrated poses.
//
// Upstream re-sent unconditionally every ~1 s, so it could not desynchronise
// from the driver. A cache can, so every site where the driver drops state
// this cache claims to know must invalidate it -- see InitCalibrator (fresh
// pipe), the slamSync wipe in SendHmdTrackerCommand (disable), the not-known
// on send failure in each sender, and the unconditional 60 s re-send below.
// ---------------------------------------------------------------------------
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

// Belt-and-braces, blueprint-mandated. Every known way for the cache to fall out
// of step with the driver is invalidated explicitly, but "every known way" is a
// claim about defects we have found. Upstream's unconditional re-send made this
// class of bug structurally impossible; a full push once a minute bounds any
// unknown instance to <= 60 s instead of "for the life of the overlay process",
// at a cost of ~70 pipe writes per minute on a scan loop that already runs 1 Hz.
static const double kFullResendIntervalSec = 60.0;

static void MaybeForceFullResend()
{
	static std::chrono::steady_clock::time_point lastFullResend{};
	static bool haveMark = false;

	const auto now = std::chrono::steady_clock::now();
	if (haveMark && std::chrono::duration<double>(now - lastFullResend).count() < kFullResendIntervalSec)
		return;

	InvalidateAppliedDriverState();
	lastFullResend = now;
	haveMark = true;
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
	// The driver is fresh after connect and holds none of the state this cache
	// remembers; force a full push on the next scan.
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

// B3, second half: write the raw sample set to disk alongside the solve, one JSON object
// per line.
//
// ACCEPTANCE (why the happy-path output is bit-identical): takes the sample vector by
// const reference, writes a file, returns. It reads no solver state it can influence and
// nothing in the overlay or driver reads a dump back.
//
// Why it exists: calibration.log records what a solve CONCLUDED, never what it was GIVEN,
// so a change to the solver can be inspected but not measured -- and on a rig where the
// felt difference between two solvers is a few millimetres, an unmeasurable change is
// indistinguishable from run-to-run scatter. These dumps make a re-solve reproducible
// offline: same input, N different estimators, compared against each other.
//
// Called BEFORE the sample set is rescaled by hmdScale, because the raw pairs are the
// point. Bounded by SampleCount() -- at most 500 samples of ~160 bytes, so ~80 KB per
// calibration -- which is why it needs no toggle.

// 14-day sweep, matching the driver's session-log rule. One dump per completed solve is
// small, but nothing else ever deletes them.
static void PruneOldCalibrationDumps(const std::string &dir)
{
	const std::string pattern = dir + "\\calsamples_*.jsonl";

	FILETIME ftNow;
	GetSystemTimeAsFileTime(&ftNow);
	ULARGE_INTEGER now;
	now.LowPart = ftNow.dwLowDateTime;
	now.HighPart = ftNow.dwHighDateTime;
	const unsigned long long maxAge100ns = 14ULL * 24 * 60 * 60 * 10000000ULL;

	WIN32_FIND_DATAA fd;
	HANDLE h = FindFirstFileA(pattern.c_str(), &fd);
	if (h == INVALID_HANDLE_VALUE)
		return;
	do
	{
		if (fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)
			continue;
		ULARGE_INTEGER wt;
		wt.LowPart = fd.ftLastWriteTime.dwLowDateTime;
		wt.HighPart = fd.ftLastWriteTime.dwHighDateTime;
		if (now.QuadPart > wt.QuadPart && (now.QuadPart - wt.QuadPart) > maxAge100ns)
			DeleteFileA((dir + "\\" + fd.cFileName).c_str());
	} while (FindNextFileA(h, &fd));
	FindClose(h);
}

static void DumpCalibrationSamples(const CalibrationContext &ctx,
	const std::vector<Sample> &samples)
{
	std::string dir;
	if (!CalibrationLogDir(dir))
		return;

	PruneOldCalibrationDumps(dir);

	SYSTEMTIME st;
	GetLocalTime(&st);
	char stamp[32];
	snprintf(stamp, sizeof stamp, "%04d%02d%02d_%02d%02d%02d",
		st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);

	std::ofstream out(dir + "\\calsamples_" + stamp + ".jsonl");
	if (!out)
		return;

	// Header line: everything a re-solve needs that is not per-sample.
	out << std::fixed << std::setprecision(9)
		<< "{\"type\":\"meta\",\"schema\":1"
		<< ",\"hmd\":\"" << ctx.hmdSerial << "\""
		<< ",\"tracker\":\"" << ctx.trackerSerial << "\""
		<< ",\"speed\":" << (int)ctx.calibrationSpeed
		<< ",\"samples\":" << samples.size()
		<< ",\"targetModelScale\":" << ctx.targetModelScale
		<< ",\"hmdScale\":" << ctx.hmdScale
		<< ",\"scaleSrc\":\"" << g_lastScaleSource << "\""
		<< "}\n";

	for (size_t i = 0; i < samples.size(); i++)
	{
		// Quaternion rather than the 3x3: four numbers instead of nine, and unambiguous.
		const Eigen::Quaterniond qr(samples[i].ref.rot);
		const Eigen::Quaterniond qt(samples[i].target.rot);
		out << "{\"i\":" << i
			<< ",\"rp\":[" << samples[i].ref.trans.x() << "," << samples[i].ref.trans.y()
			<< "," << samples[i].ref.trans.z() << "]"
			<< ",\"rq\":[" << qr.w() << "," << qr.x() << "," << qr.y() << "," << qr.z() << "]"
			<< ",\"tp\":[" << samples[i].target.trans.x() << "," << samples[i].target.trans.y()
			<< "," << samples[i].target.trans.z() << "]"
			<< ",\"tq\":[" << qt.w() << "," << qt.x() << "," << qt.y() << "," << qt.z() << "]"
			<< "}\n";
	}
}

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

	// DEGENERATE-INPUT GUARD (A14). Unreachable once the finish gate has fired with a
	// real sample set, so it cannot change a happy-path solve. With no usable deltas
	// the cross-covariance is the zero matrix and the SVD below yields an arbitrary,
	// identity-like rotation that carries no information but looks confident. Signal
	// it instead: the NaN propagates through the solve to the residual, which the
	// `!(rmsError <= 0.1)` gate below rejects.
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

static const double ScaleSpreadThreshold = 0.1;
static const double MinCalibratedScale = 0.9;
static const double MaxCalibratedScale = 1.1;

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

static double EstimateHmdSpaceScale(const std::vector<Sample> &samples, const Eigen::Matrix3d &rotation, double targetModelScale)
{
	// B3 assignment sites. Each exit below records which one it was, and (where a fit
	// happened) what the fit produced, into the three write-only statics at the top of
	// this file. Nothing in this function or any other reads them back -- they exist so
	// calibration.log can distinguish a measured scale from one of upstream's three
	// silent "assuming 1" branches, which the shipped number alone cannot show.
	g_lastScaleSource = "unknown";
	g_lastScaleRaw = -1.0;
	g_lastScaleSpread = 0.0;

	// DEGENERATE-INPUT GUARD (A14). Unreachable from the live path -- the finish gate
	// cannot fire on an empty set -- so no happy-path solve changes. Without it the
	// centroid divides by zero, spread is NaN, `spread < ScaleSpreadThreshold` is FALSE
	// for NaN, and the function goes on to solve a zero-row system. 1.0 is the same
	// answer both of upstream's own inconclusive branches already give.
	if (samples.empty())
	{
		g_lastScaleSource = "no_samples";
		CalCtx.Log("No samples to fit headset scale, assuming 1\n");
		return 1.0;
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
		g_lastScaleSource = "low_spread";
		snprintf(buf, sizeof buf, "Not enough positional movement to estimate headset scale (spread %.2f m), assuming 1\n", spread);
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

	Eigen::VectorXd result = coefficients.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(constants);
	double fittedScale = result(0);
	g_lastScaleRaw = fittedScale;

	if (fittedScale < MinCalibratedScale || fittedScale > MaxCalibratedScale)
	{
		// The rejected value is kept in g_lastScaleRaw deliberately: "assuming 1" does
		// not say how far off the solve was, and that distance is the diagnostic.
		g_lastScaleSource = "implausible";
		snprintf(buf, sizeof buf, "Fitted space scale %.5f is not plausible, assuming headset scale 1\n", fittedScale);
		CalCtx.Log(buf);
		return 1.0;
	}

	g_lastScaleSource = "measured";
	snprintf(buf, sizeof buf, "Fitted headset space scale relative to lighthouse: %.5f (%+.2f%%), implied absolute headset scale: %.5f\n",
		fittedScale, (fittedScale - 1.0) * 100.0, fittedScale / targetModelScale);
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
	// DEGENERATE-INPUT GUARD (A14): unreachable once the finish gate fires; upstream
	// divided the zero accumulator by zero here.
	if (samples.empty())
		return Eigen::Vector3d::Zero();

	Eigen::Vector3d accum = Eigen::Vector3d::Zero();

	for (auto &sample : samples)
		accum += sample.ref.rot.transpose() * (calScale * (calRot * sample.target.trans) + calTrans - sample.ref.trans);

	return accum / (double)samples.size();
}

static double RetargetingErrorRMS(const std::vector<Sample> &samples, const Eigen::Vector3d &hmdToTargetPos, const Eigen::Matrix3d &calRot, const Eigen::Vector3d &calTrans, double calScale)
{
	// DEGENERATE-INPUT GUARD (A14): unreachable once the finish gate fires. NaN, not
	// zero -- this value feeds the `!(rmsError <= 0.1)` accept gate, and a zero
	// residual would read as a perfect fit and be saved.
	if (samples.empty())
		return std::numeric_limits<double>::quiet_NaN();

	double accum = 0;

	for (auto &sample : samples)
		accum += (calScale * (calRot * sample.target.trans) + calTrans - (sample.ref.rot * hmdToTargetPos + sample.ref.trans)).squaredNorm();

	return std::sqrt(accum / (double)samples.size());
}

// Which device was missing on the last invalid CollectSample, for the abort message.
static const char *g_lastCollectFailure = nullptr;

Sample CollectSample(const CalibrationContext &ctx)
{
	vr::TrackedDevicePose_t reference, target;
	reference.bPoseIsValid = false;
	target.bPoseIsValid = false;

	reference = ctx.devicePoses[0];
	target = ctx.devicePoses[ctx.targetID];

	// A14. Per-sample math is bit-identical: an invalid pose is never appended in
	// either build, and a valid pair produces exactly the Sample upstream produced.
	// What moves out of here is the CONTROL FLOW. Upstream ended the run from inside
	// this collector -- `state = None` with no restore -- so the first untracked tick
	// killed the whole calibration AND left the context holding the new trackerSerial /
	// targetTrackingSystem that BeginSamplingPhase had just written over the old
	// profile's, with none of the calibrated values reloaded. Report which device is
	// missing and let the caller decide.
	//
	// No logging here: this runs at 20 Hz, and a per-tick message would bury the
	// in-VR message list.
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

// A13. The message built here is byte-for-byte the one upstream built at each call
// site -- the same six-argument SetDeviceTransform constructor, the same zero
// vector / identity quaternion / 1.0 scale on the disable path. Only the decision
// to send it at all is new.
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
		vr::HmdVector3d_t zeroV;
		zeroV.v[0] = zeroV.v[1] = zeroV.v[2] = 0;

		vr::HmdQuaternion_t zeroQ;
		zeroQ.x = 0; zeroQ.y = 0; zeroQ.z = 0; zeroQ.w = 1;

		req.setDeviceTransform = { id, false, zeroV, zeroQ, 1.0 };
	}

	// SendBlocking throws on a broken pipe, and this runs on the overlay main loop
	// with no handler above it -- an uncaught throw is std::terminate, killing the
	// overlay before its exit-time SaveProfile. A pipe break happens precisely when
	// the driver goes away (SteamVR shutdown, driver reload). On failure the cache is
	// left NOT-known so the next scan re-sends: caching a send that never arrived is
	// the silent-desync class this commit exists to prevent.
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
	vr::HmdVector3d_t zeroV;
	zeroV.v[0] = zeroV.v[1] = zeroV.v[2] = 0;

	vr::HmdQuaternion_t zeroQ;
	zeroQ.x = 0; zeroQ.y = 0; zeroQ.z = 0; zeroQ.w = 1;

	ApplyDeviceTransform(id, false, zeroV, zeroQ, 1.0);
}

void SendOneEuroParams()
{
	protocol::Request req(protocol::RequestSetOneEuro);
	req.setOneEuro.headEnabled = CalCtx.headFilterEnabled;
	req.setOneEuro.head = CalCtx.headFilterParams;
	req.setOneEuro.drift = CalCtx.driftFilterParams;

	// A13: same message, sent only when it would change what the driver holds.
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
		g_appliedOneEuroKnown = false;
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

	// A13: same message, sent only when it would change what the driver holds.
	if (g_appliedHmdKnown && HmdTrackerEq(g_appliedHmd, req.setHmdTracker))
		return;

	// Upstream let SendBlocking throw out of here; the only call sites are on the
	// overlay main loop with no handler above them.
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

	// CACHE-INVALIDATION SITE (A13, the recorded live failure fa7dc28). A disable
	// makes the driver wipe its slamSync set: the SetHmdTracker handler runs
	// ResetEstimators with clearSlamSync, which memsets the whole array. This cache
	// cannot see that happen, and the re-enrolment loop in ScanAndApplyProfile is
	// guarded on `overrideActive`, so it does not run while the override is off
	// either. Without this wipe the cache still claims every device is enrolled, so
	// when the override comes back every id hits `continue` and no SetSlamSync is
	// ever re-sent -- the body pucks then run with no drift correction while the UI
	// shows Continuous Sync on. Upstream re-sent unconditionally each scan, so it
	// could not reach this state.
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

	// A13 belt-and-braces: once a minute this scan pushes everything regardless of
	// the cache, restoring upstream's "cannot desynchronise" property with a 60 s
	// bound instead of a 1 s one.
	MaybeForceFullResend();

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

	// A13: write the desired transform once per device. Do NOT disable-all and then
	// re-enable -- that races the pose thread and briefly publishes uncalibrated
	// poses. The terminal state of each of the six branches below is exactly the
	// terminal state upstream reached after its disable-then-maybe-enable pair.
	for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
	{
		auto deviceClass = vr::VRSystem()->GetTrackedDeviceClass(id);
		if (deviceClass == vr::TrackedDeviceClass_Invalid)
			continue;

		// The HMD pose is replaced from the head tracker; never space-warp it.
		// (Upstream reached the same terminal state via ResetAndDisableOffsets
		// followed by the `id == HMD` continue.)
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

		// The head-mounted tracker is the HMD's pose source; keep its raw pose.
		if (deviceClass == vr::TrackedDeviceClass_GenericTracker && trackingSystem == ctx.targetTrackingSystem && id == ctx.targetID)
		{
			ResetAndDisableOffsets(id);
			continue;
		}

		if (trackingSystem == ctx.targetTrackingSystem) {
			// Unchanged, deliberately: same operands, same order, same divisor.
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
		// Same broken-pipe hazard as the device transform above, same cache rule on
		// failure. One failure means the pipe is dead for all of them, so stop the
		// loop rather than throwing up to 60 more times in this scan.
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

// A14: tracking-loss bookkeeping. g_lossStart is when the current gap began (0 while
// tracking); g_lossRunTicks counts the ticks dropped in the current gap.
static double g_lossStart = 0.0;
static int g_lossRunTicks = 0;

// How long either device may stay untracked before the run is abandoned.
//
// PHASE 1: DELIBERATELY ZERO, per the blueprint. Upstream ends the run on the very
// first untracked tick (it did so from inside CollectSample), and a coast would let
// runs complete that upstream would have abandoned -- a different set of datasets
// reaching the solver, which is NOT output-identical and so does not belong in a
// keep-guard commit. At 0.0 the run still ends on the first dropped tick, exactly as
// upstream; what changed is that it ends through AbortAndRestoreProfile, which puts
// the previous calibration back instead of leaving the context half-overwritten.
// Raising this to ~2.0 is a one-constant change once there is evidence for it.
static const double kMaxTrackingLossSec = 0.0;

static void BeginSamplingPhase(CalibrationContext &ctx, uint32_t targetID)
{
	ctx.targetID = targetID;
	ctx.targetTrackingSystem = GetDeviceTrackingSystem(targetID);
	ctx.hmdSerial = GetDeviceSerial(vr::k_unTrackedDeviceIndex_Hmd);

	// A14. An empty serial is not a new tracker. GetDeviceSerial discards the
	// ETrackedPropertyError and returns "" when the property read fails, which is
	// routine for a device that has just woken or re-enumerated. Upstream stored the
	// "" unconditionally, which breaks serial-based target recovery in
	// ScanAndApplyProfile and the connected-tracker check in the UI. On the happy path
	// the read succeeds and this assigns exactly what upstream assigned.
	std::string newTrackerSerial = GetDeviceSerial(targetID);
	if (!newTrackerSerial.empty())
		ctx.trackerSerial = newTrackerSerial;

	char buf[256];
	snprintf(buf, sizeof buf, "Using headset tracker: %s (id %d)\n", ctx.trackerSerial.c_str(), targetID);
	ctx.Log(buf);

	ResetAndDisableOffsets(targetID);
	SendHmdTrackerCommand(vr::k_unTrackedDeviceIndex_Hmd, vr::k_unTrackedDeviceIndexInvalid, false);

	ctx.state = CalibrationState::Sampling;
	ctx.wantedUpdateInterval = 0.0;
	g_lossStart = 0.0;
	g_lossRunTicks = 0;
	g_lastCollectFailure = nullptr;
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
}

static void AbortAndRestoreProfile(CalibrationContext &ctx)
{
	if (ctx.targetID != vr::k_unTrackedDeviceIndexInvalid)
		ResetAndDisableOffsets(ctx.targetID);

	// A14. An abort is supposed to restore the CALIBRATION. LoadProfile restores the
	// whole profile, and ParseProfile unconditionally rewrites every Settings/Smoothing
	// field with it -- native, fallbackSlam, eAngVel, continuousSync, predictionTime,
	// headFilterEnabled, both One-Euro sets, calibration_speed. None of those controls
	// call SaveProfile at the point of change (persistence happens at solve time or at
	// overlay exit), so an abort silently threw away whatever the user had just set.
	//
	// Not cosmetic: continuousSync reverting to the stored value makes the next 1 Hz
	// ScanAndApplyProfile push sync=false to every body puck, so the pucks lose drift
	// correction while the UI still shows the setting the user chose.
	//
	// This snapshot touches NO calibrated quantity -- not rotation, translation, scale,
	// hmdScale, targetModelScale, the rel_* offset or any serial. Those all come back
	// from the profile exactly as LoadProfile restored them.
	const bool sNative = ctx.enableNative;
	const bool sFallback = ctx.fallbackToSlam;
	const bool sEAngVel = ctx.enableAngularVelocity;
	const bool sSync = ctx.continuousSync;
	const float sPred = ctx.predictionTime;
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
			LogCalibrationOutcome(ctx, "abort", "no_tracking_hmd", 0, 0);
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
			LogCalibrationOutcome(ctx, "abort", "no_candidate_trackers", 0, 0);
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
			LogCalibrationOutcome(ctx, "abort", "insufficient_head_motion", 0, 0);
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
			LogCalibrationOutcome(ctx, "abort", "tracker_id_ambiguous", 0, 0);
			CalCtx.Log("Couldn't clearly identify the headset tracker, aborting! Make sure only the headset tracker moves with your head, then try again.\n");
			return;
		}

		uint32_t targetID = Detection.candidates[bestIdx];
		Detection.Clear();
		BeginSamplingPhase(ctx, targetID);
		return;
	}

	auto sample = CollectSample(ctx);
	if (!sample.valid)
	{
		// A14: the run now ends HERE, through the restore path, instead of inside
		// CollectSample with a bare `state = None`. With kMaxTrackingLossSec == 0 this
		// fires on the first untracked tick, i.e. at exactly the moment upstream ended
		// the run -- the difference is that the previous calibration comes back.
		if (g_lossStart == 0.0)
			g_lossStart = time;
		++g_lossRunTicks;

		const double lostSec = time - g_lossStart;
		if (lostSec >= kMaxTrackingLossSec)
		{
			char buf[256];
			snprintf(buf, sizeof buf,
				"%s stopped tracking - aborting calibration. Previous calibration restored.\n",
				g_lastCollectFailure ? g_lastCollectFailure : "A device");
			ctx.Log(buf);
			LogCalibrationOutcome(ctx, "abort", "tracking_lost", collectedSamples.size(), g_lossRunTicks);
			AbortAndRestoreProfile(ctx);
			return;
		}

		return;
	}

	if (g_lossRunTicks > 0)
	{
		// Recovered inside the coast window. Unreachable while kMaxTrackingLossSec is 0;
		// kept so raising that constant stays a one-line change.
		char buf[256];
		snprintf(buf, sizeof buf, "Tracking recovered after %d dropped ticks, continuing.\n", g_lossRunTicks);
		ctx.Log(buf);
		g_lossRunTicks = 0;
		g_lossStart = 0.0;
	}

	auto &samples = collectedSamples;
	samples.push_back(sample);

	CalCtx.Progress(samples.size(), CalCtx.SampleCount());

	if (samples.size() >= CalCtx.SampleCount())
	{
		CalCtx.Log("\n");

		double axisVariance = SecondAxisVariance(samples);
		if (axisVariance < AxisVarianceThreshold)
		{
			if (++coplanarRetries >= 10)
			{
				CalCtx.Log("Not enough rotation variety after several attempts, aborting calibration! Previous calibration restored.\n");
				LogCalibrationOutcome(ctx, "abort", "axis_variance", samples.size(), g_lossRunTicks);
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

		// B3. The count the rest of this solve actually ran on, taken before anything
		// below can touch the vector -- the coplanar branch above erases a quarter of the
		// set and returns, so samples.size() at the end of the run is not what the finish
		// gate saw.
		const size_t acceptedSamples = samples.size();

		// B3. Written HERE, between the scale fit and the rescale below, because this is
		// the last moment the pairs are raw: the loop that follows divides every reference
		// translation by hmdScale in place, and a dump taken afterwards could not be
		// re-solved with a different scale estimator. Reads the vector by const ref and
		// writes a file; the solve continues on exactly the samples it would have had.
		DumpCalibrationSamples(ctx, samples);

		for (auto &sample : samples)
			sample.ref.trans /= ctx.hmdScale;

		ctx.calibratedTranslation = CalibrateTranslation(samples, calRot, calScale);
		Eigen::Vector3d calTransM = ctx.calibratedTranslation * 0.01;

		Eigen::Vector3d hmdToTarget = ComputeRefToTargetOffset(samples, calRot, calTransM, calScale);
		double rmsError = RetargetingErrorRMS(samples, hmdToTarget, calRot, calTransM, calScale);

		char buf2[256];
		snprintf(buf2, sizeof buf2, "Calibration residual error (RMS): %.1f mm\n", rmsError * 1000.0);
		CalCtx.Log(buf2);

		// TODO: this is an problem for future considering automatic calibration fixing.
		// A14: written as !(x <= limit) rather than `x > limit` so a NaN residual FAILS
		// the gate. For every finite value the two are the same predicate, so no real
		// solve changes; `rmsError > 0.1` is false for NaN, which would have let a
		// degenerate solve through to SaveProfile and into the pose pipeline.
		if (!(rmsError <= 0.1))
		{
			CalCtx.Log("Calibration quality is too low, aborting! Previous calibration restored. Try again with a slower calibration speed, moving smoothly.\n");
			char reason[64];
			snprintf(reason, sizeof reason, "rms_too_high(%.1fmm)", rmsError * 1000.0);
			LogCalibrationOutcome(ctx, "abort", reason, acceptedSamples, g_lossRunTicks);
			AbortAndRestoreProfile(ctx);
			return;
		}

		ComputeRelativeOffset(ctx, samples, calRot, calTransM, calScale);

		ctx.validProfile = true;
		SaveProfile(ctx);
		CalCtx.Log("Finished calibration, profile saved\n");

		// B3. After SaveProfile, so the line records exactly what was persisted --
		// rotation, scale, trim and lever arm are all final by here.
		LogCalibrationResult(ctx, rmsError * 1000.0, acceptedSamples, g_lossRunTicks);

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
