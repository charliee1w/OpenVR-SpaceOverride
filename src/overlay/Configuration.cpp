// SPDX-License-Identifier: AGPL-3.0-only

#include "Configuration.h"

#include <Windows.h>

#include <picojson.h>

#include <string>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <limits>
#include <algorithm>
#include <ctime>
#include <vector>

static picojson::array FloatArray(const float *buf, int numFloats)
{
	picojson::array arr;

	for (int i = 0; i < numFloats; i++)
		arr.push_back(picojson::value(double(buf[i])));

	return arr;
}

static void LoadFloatArray(const picojson::value &obj, float *buf, int numFloats)
{
	if (!obj.is<picojson::array>())
		throw std::runtime_error("expected array, got " + obj.to_str());

	auto &arr = obj.get<picojson::array>();
	if (arr.size() != numFloats)
		throw std::runtime_error("wrong buffer size");

	for (int i = 0; i < numFloats; i++)
		buf[i] = (float) arr[i].get<double>();
}

static void ParseProfile(CalibrationContext &ctx, std::istream &stream)
{
	picojson::value v;
	std::string err = picojson::parse(v, stream);
	if (!err.empty())
		throw std::runtime_error(err);

	auto arr = v.get<picojson::array>();
	if (arr.size() < 1)
		throw std::runtime_error("no profiles in file");

	auto obj = arr[0].get<picojson::object>();

	// CORRECTED 2026-08-16 — the rationale this comment used to give was wrong, and the project's
	// own rule is that a claim goes in the record only after it has been checked against source.
	// It said get<T>() is "guarded only by assert(), so under NDEBUG it reads the wrong union
	// member and returns an indeterminate value". It does not. The vendored
	// 3rdparty/PicoJSON/picojson.h:93-99 defines PICOJSON_ASSERT(e) as
	// `if (!(e)) throw std::runtime_error(#e);` unconditionally, NDEBUG or not, and nothing in
	// this tree overrides it. A wrongly-typed read throws; it never returns garbage.
	//
	// What these guards actually buy is therefore a *better message* and a defined failure point,
	// not memory safety: "profile key 'yaw' missing or not a number" instead of a bare
	// `"type mismatch! ..." && is<ctype>()`. Worth keeping for that, since this file is the
	// documented recovery path when the registry is empty. But the dangerous-sounding version of
	// the story was fiction, and the one change here that fixed a real defect was making the
	// fork-added booleans optional below — those threw on legitimately older profiles.
	auto reqDouble = [&](const char *key) -> double {
		if (!obj[key].is<double>())
			throw std::runtime_error(std::string("profile key '") + key + "' missing or not a number");
		return obj[key].get<double>();
	};

	if (!obj["target_tracking_system"].is<std::string>())
		throw std::runtime_error("profile key 'target_tracking_system' missing or not a string");
	ctx.targetTrackingSystem = obj["target_tracking_system"].get<std::string>();

	if (obj["hmd_serial"].is<std::string>())
		ctx.hmdSerial = obj["hmd_serial"].get<std::string>();
	if (obj["tracker_serial"].is<std::string>())
		ctx.trackerSerial = obj["tracker_serial"].get<std::string>();
	ctx.calibratedRotation(0) = reqDouble("roll");
	ctx.calibratedRotation(1) = reqDouble("yaw");
	ctx.calibratedRotation(2) = reqDouble("pitch");
	ctx.calibratedTranslation(0) = reqDouble("x");
	ctx.calibratedTranslation(1) = reqDouble("y");
	ctx.calibratedTranslation(2) = reqDouble("z");

	if (obj["scale"].is<double>())
		ctx.calibratedScale = obj["scale"].get<double>();
	else
		ctx.calibratedScale = 1.0;

	if (obj["targetModelScale"].is<double>())
		ctx.targetModelScale = obj["targetModelScale"].get<double>();
	else
		ctx.targetModelScale = 1.0;

	if (obj["hmdScale"].is<double>())
		ctx.hmdScale = obj["hmdScale"].get<double>();
	else
		ctx.hmdScale = 1.0;

	if (ctx.targetModelScale <= 0.0)
		ctx.targetModelScale = 1.0;
	if (ctx.hmdScale <= 0.0)
		ctx.hmdScale = 1.0;

	// How many solves are averaged into hmdScale, and which headset that average describes.
	// Both optional: profiles written before scale averaging existed carry a single measured
	// scale, which is exactly one prior observation — the same fallback lever_n uses below.
	// Attributing a legacy average to the profile's own hmd_serial matches the assumption
	// those files were saved under, when the two serials could not legitimately differ.
	ctx.hmdScaleSamples = obj["hmdscale_n"].is<double>()
		? (int) obj["hmdscale_n"].get<double>()
		: (ctx.hmdScale != 1.0 ? 1 : 0);
	ctx.hmdScaleSerial = obj["hmdscale_serial"].is<std::string>()
		? obj["hmdscale_serial"].get<std::string>()
		: ctx.hmdSerial;
	if (ctx.hmdScaleSamples < 0)
		ctx.hmdScaleSamples = 0;

	// Optional, not required: these three keys are fork additions, so a profile saved by
	// upstream or an early fork build legitimately lacks them. Treating "missing because older
	// schema" like "missing because truncated" threw here and wiped an otherwise fully valid
	// calibration (both stores share the schema, so both candidates failed and LoadProfile fell
	// through to Clear()). Defaults match CalibrationContext's initialisers.
	auto optBool = [&](const char *key, bool def) -> bool {
		return obj[key].is<bool>() ? obj[key].get<bool>() : def;
	};
	ctx.enableNative = optBool("native", false);
	ctx.fallbackToSlam = optBool("fallbackSlam", true);
	ctx.enableAngularVelocity = optBool("eAngVel", false);

	if (obj["continuousSync"].is<bool>())
		ctx.continuousSync = obj["continuousSync"].get<bool>();
	else
		ctx.continuousSync = true;

	if (obj["predictionTime"].is<double>())
		ctx.predictionTime = obj["predictionTime"].get<double>();
	else
		ctx.predictionTime = 1.0;

	auto loadOneEuro = [&](const char *key, protocol::OneEuroParams &out, protocol::OneEuroParams def) {
		out = def;
		if (!obj[key].is<picojson::object>())
			return;
		auto o = obj[key].get<picojson::object>();
		if (o["minCutoff"].is<double>()) out.minCutoff = o["minCutoff"].get<double>();
		if (o["beta"].is<double>())      out.beta = o["beta"].get<double>();
		if (o["dCutoff"].is<double>())   out.dCutoff = o["dCutoff"].get<double>();
	};

	// Missing-key fallbacks MUST match the CalibrationContext member defaults (Calibration.h) —
	// they had drifted three retunings behind and, worse, fell back to headFilterEnabled=true
	// against a member default of false and the settings-v1 decision (2026-07-01) that the head
	// filter ships OFF. A profile written before these keys existed therefore silently
	// re-enabled the head One-Euro with stale parameters on every load.
	ctx.headFilterEnabled = obj["headFilterEnabled"].is<bool>() ? obj["headFilterEnabled"].get<bool>() : false;
	loadOneEuro("headFilter", ctx.headFilterParams, { 5.0, 0.8, 1.0 });
	loadOneEuro("driftFilter", ctx.driftFilterParams, { 3.0, 1.3, 0.6 });

	// All seven or none: rel_qw alone used to admit the block and the remaining six were then
	// read unchecked.
	if (obj["rel_qw"].is<double>() && obj["rel_qx"].is<double>() && obj["rel_qy"].is<double>()
		&& obj["rel_qz"].is<double>() && obj["rel_tx"].is<double>() && obj["rel_ty"].is<double>()
		&& obj["rel_tz"].is<double>())
	{
		ctx.relativeRotation.w = obj["rel_qw"].get<double>();
		ctx.relativeRotation.x = obj["rel_qx"].get<double>();
		ctx.relativeRotation.y = obj["rel_qy"].get<double>();
		ctx.relativeRotation.z = obj["rel_qz"].get<double>();
		ctx.relativeTranslation.v[0] = obj["rel_tx"].get<double>();
		ctx.relativeTranslation.v[1] = obj["rel_ty"].get<double>();
		ctx.relativeTranslation.v[2] = obj["rel_tz"].get<double>();
		ctx.validRelativeOffset = true;
		// Absent in profiles saved before this field existed; those still carry exactly
		// one prior lever-arm observation, same as ComputeRelativeOffset's own fallback.
		ctx.leverSamples = obj["lever_n"].is<double>() ? (int) obj["lever_n"].get<double>() : 1;
		// Which tracker the average belongs to. Legacy profiles predate the field and were
		// written when the two serials could not legitimately differ, so attribute the
		// average to the profile's tracker — the same assumption those files were saved under.
		ctx.leverSerial = obj["lever_serial"].is<std::string>()
			? obj["lever_serial"].get<std::string>()
			: ctx.trackerSerial;
	}
	else
	{
		ctx.validRelativeOffset = false;
		ctx.leverSamples = 0;
		ctx.leverSerial = "";
	}

	if (obj["calibration_speed"].is<double>())
		ctx.calibrationSpeed = (CalibrationContext::Speed)(int) obj["calibration_speed"].get<double>();

	if (obj["chaperone"].is<picojson::object>())
	{
		auto chaperone = obj["chaperone"].get<picojson::object>();
		// Optional, for the same reason native/fallbackSlam/eAngVel are above: get<bool>() throws
		// on a missing or wrongly-typed key, and a throw here escapes ParseProfile, which makes
		// LoadProfile discard this whole store and fall through to the other candidate or to
		// Clear(). Losing a valid rotation, translation and scale over an absent cosmetic flag is
		// not a trade worth making. Default matches CalibrationContext's initialiser.
		ctx.chaperone.autoApply = chaperone["auto_apply"].is<bool>()
			? chaperone["auto_apply"].get<bool>()
			: true;

		LoadFloatArray(chaperone["play_space_size"], ctx.chaperone.playSpaceSize.v, 2);

		LoadFloatArray(
			chaperone["standing_center"],
			(float *) ctx.chaperone.standingCenter.m,
			sizeof(ctx.chaperone.standingCenter.m) / sizeof(float)
		);

		if (!chaperone["geometry"].is<picojson::array>())
			throw std::runtime_error("chaperone geometry is not an array");

		auto &geometry = chaperone["geometry"].get<picojson::array>();

		// The element type is a quad of 12 floats, so the array length must be an
		// exact multiple of 12. The old sizing truncated (len*4/48 == len/12), which
		// under-allocated for any non-multiple length and then wrote the full array
		// into it -- a heap overflow (or a write through data() on an empty vector)
		// driven by user-writable registry content.
		const size_t floatsPerQuad = sizeof(ctx.chaperone.geometry[0]) / sizeof(float);
		if (geometry.size() > 0)
		{
			if (geometry.size() % floatsPerQuad != 0)
				throw std::runtime_error("chaperone geometry length is not a multiple of " + std::to_string(floatsPerQuad));

			ctx.chaperone.geometry.resize(geometry.size() / floatsPerQuad);
			LoadFloatArray(chaperone["geometry"], (float *) ctx.chaperone.geometry.data(), geometry.size());

			ctx.chaperone.valid = true;
		}
	}

	ctx.validProfile = true;
}

static void WriteProfile(CalibrationContext &ctx, std::ostream &out)
{
	if (!ctx.validProfile)
		return;

	picojson::object profile;
	profile["target_tracking_system"].set<std::string>(ctx.targetTrackingSystem);
	profile["hmd_serial"].set<std::string>(ctx.hmdSerial);
	profile["tracker_serial"].set<std::string>(ctx.trackerSerial);
	profile["roll"].set<double>(ctx.calibratedRotation(0));
	profile["yaw"].set<double>(ctx.calibratedRotation(1));
	profile["pitch"].set<double>(ctx.calibratedRotation(2));
	profile["x"].set<double>(ctx.calibratedTranslation(0));
	profile["y"].set<double>(ctx.calibratedTranslation(1));
	profile["z"].set<double>(ctx.calibratedTranslation(2));
	profile["scale"].set<double>(ctx.calibratedScale);
	profile["targetModelScale"].set<double>(ctx.targetModelScale);
	profile["hmdScale"].set<double>(ctx.hmdScale);
	// Companions to hmdScale, persisted for the same reason lever_n is: an average that
	// resets every relaunch never averages anything. Unconditional, unlike the rel_* block
	// below — hmdScale itself is written unconditionally, so gating its sample count would
	// let the two drift apart across an abandoned run.
	{
		double hmdScaleSamples = ctx.hmdScaleSamples;
		profile["hmdscale_n"].set<double>(hmdScaleSamples);
	}
	profile["hmdscale_serial"].set<std::string>(ctx.hmdScaleSerial);
	// Save time, so LoadProfile can tell which store holds the newer calibration when
	// the two disagree. Written as a plain unix timestamp; absent in legacy profiles,
	// which are then treated as older than anything carrying a stamp.
	double savedAt = (double) std::time(nullptr);
	profile["savedAt"].set<double>(savedAt);

	profile["native"].set<bool>(ctx.enableNative);
	profile["fallbackSlam"].set<bool>(ctx.fallbackToSlam);
	profile["eAngVel"].set<bool>(ctx.enableAngularVelocity);
	profile["continuousSync"].set<bool>(ctx.continuousSync);

	double time = ctx.predictionTime;
	profile["predictionTime"].set<double>(time);

	profile["headFilterEnabled"].set<bool>(ctx.headFilterEnabled);

	auto saveOneEuro = [](const protocol::OneEuroParams &p) {
		picojson::object o;
		o["minCutoff"].set<double>(p.minCutoff);
		o["beta"].set<double>(p.beta);
		o["dCutoff"].set<double>(p.dCutoff);
		return o;
	};
	profile["headFilter"].set<picojson::object>(saveOneEuro(ctx.headFilterParams));
	profile["driftFilter"].set<picojson::object>(saveOneEuro(ctx.driftFilterParams));

	if (ctx.validRelativeOffset)
	{
		profile["rel_qw"].set<double>(ctx.relativeRotation.w);
		profile["rel_qx"].set<double>(ctx.relativeRotation.x);
		profile["rel_qy"].set<double>(ctx.relativeRotation.y);
		profile["rel_qz"].set<double>(ctx.relativeRotation.z);
		profile["rel_tx"].set<double>(ctx.relativeTranslation.v[0]);
		profile["rel_ty"].set<double>(ctx.relativeTranslation.v[1]);
		profile["rel_tz"].set<double>(ctx.relativeTranslation.v[2]);
		double leverSamples = ctx.leverSamples;
		profile["lever_n"].set<double>(leverSamples);
		// The average's owner — may lag tracker_serial when a swap run was started but never
		// solved, which is exactly the state the solve-time reset needs to see.
		profile["lever_serial"].set<std::string>(ctx.leverSerial);
	}

	double speed = (int) ctx.calibrationSpeed;
	profile["calibration_speed"].set<double>(speed);

	if (ctx.chaperone.valid)
	{
		picojson::object chaperone;
		chaperone["auto_apply"].set<bool>(ctx.chaperone.autoApply);
		chaperone["play_space_size"].set<picojson::array>(FloatArray(ctx.chaperone.playSpaceSize.v, 2));

		chaperone["standing_center"].set<picojson::array>(FloatArray(
			(float *) ctx.chaperone.standingCenter.m,
			sizeof(ctx.chaperone.standingCenter.m) / sizeof(float)
		));

		chaperone["geometry"].set<picojson::array>(FloatArray(
			(float *) ctx.chaperone.geometry.data(),
			sizeof(ctx.chaperone.geometry[0]) / sizeof(float) * ctx.chaperone.geometry.size()
		));

		profile["chaperone"].set<picojson::object>(chaperone);
	}

	picojson::value profileV;
	profileV.set<picojson::object>(profile);

	picojson::array profiles;
	profiles.push_back(profileV);

	picojson::value profilesV;
	profilesV.set<picojson::array>(profiles);

	out << profilesV.serialize(true);
}

static void LogRegistryResult(LSTATUS result)
{
	char *message;
	FormatMessageA(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_ALLOCATE_BUFFER, 0, result, LANG_USER_DEFAULT, (LPSTR)&message, 0, NULL);
	std::cerr << "Opening registry key: " << message << std::endl;
}

static const char *RegistryKey = "Software\\OpenVR-SpaceOverride";

static std::string ReadRegistryKey()
{
	DWORD size = 0;
	auto result = RegGetValueA(HKEY_CURRENT_USER_LOCAL_SETTINGS, RegistryKey, "Config", RRF_RT_REG_SZ, 0, 0, &size);
	if (result != ERROR_SUCCESS)
	{
		LogRegistryResult(result);
		return "";
	}

	std::string str;
	str.resize(size);

	result = RegGetValueA(HKEY_CURRENT_USER_LOCAL_SETTINGS, RegistryKey, "Config", RRF_RT_REG_SZ, 0, &str[0], &size);
	if (result != ERROR_SUCCESS)
	{
		LogRegistryResult(result);
		return "";
	}
	
	str.resize(size - 1);
	return str;
}

// Returns whether the value is actually readable back as written. A registry write
// that fails leaves the previous profile in place, and the old code reported that
// only to stderr (invisible for a windowed app) while still writing the backup file
// -- so the two stores could silently drift apart and the next launch would load the
// stale one, reverting the user's calibration.
static bool WriteRegistryKey(std::string str)
{
	HKEY hkey;
	auto result = RegCreateKeyExA(HKEY_CURRENT_USER_LOCAL_SETTINGS, RegistryKey, 0, REG_NONE, 0, KEY_ALL_ACCESS, 0, &hkey, 0);
	if (result != ERROR_SUCCESS)
	{
		LogRegistryResult(result);
		return false;
	}

	DWORD size = str.size() + 1;

	result = RegSetValueExA(hkey, "Config", 0, REG_SZ, reinterpret_cast<const BYTE*>(str.c_str()), size);
	if (result != ERROR_SUCCESS)
		LogRegistryResult(result);

	RegCloseKey(hkey);

	if (result != ERROR_SUCCESS)
		return false;

	return ReadRegistryKey() == str;
}

// Save timestamp of a serialized profile, or -1 when it is missing/unparseable and
// 0 when it parses but predates the stamp (legacy profile).
static double ProfileSavedAt(const std::string &str)
{
	if (str.empty())
		return -1.0;

	picojson::value v;
	std::stringstream io(str);
	if (!picojson::parse(v, io).empty() || !v.is<picojson::array>())
		return -1.0;

	auto &arr = v.get<picojson::array>();
	if (arr.empty() || !arr[0].is<picojson::object>())
		return -1.0;

	auto obj = arr[0].get<picojson::object>();
	if (obj["savedAt"].is<double>())
		return obj["savedAt"].get<double>();

	return 0.0;
}

static std::string BackupFilePath()
{
	char localAppData[MAX_PATH] = {};
	DWORD n = GetEnvironmentVariableA("LOCALAPPDATA", localAppData, MAX_PATH);
	if (n == 0 || n >= MAX_PATH)
		return "";
	return std::string(localAppData) + "\\OpenVR-SpaceOverride\\profile-backup.json";
}

static std::string ReadBackupFile()
{
	std::string path = BackupFilePath();
	if (path.empty())
		return "";

	std::ifstream in(path);
	if (!in)
		return "";

	std::stringstream ss;
	ss << in.rdbuf();
	return ss.str();
}

void LoadProfile(CalibrationContext &ctx)
{
	ctx.validProfile = false;

	// SaveProfile writes the registry and the on-disk mirror from one string, so they
	// normally agree. If one write silently fails they drift apart, and always
	// preferring the registry then loads a stale calibration -- observed in the wild
	// with the two stores holding calibrations ~92 degrees apart in yaw, which presents
	// as "my calibration reverted on its own". Load whichever store was saved most
	// recently instead of a fixed priority, and fall through to the other if the newer
	// one does not parse.
	const std::string reg = ReadRegistryKey();
	const std::string bak = ReadBackupFile();

	struct Candidate
	{
		const std::string *str;
		const char *name;
		double savedAt;
	};

	std::vector<Candidate> candidates;
	if (!reg.empty())
		candidates.push_back({ &reg, "registry", ProfileSavedAt(reg) });
	if (!bak.empty())
		candidates.push_back({ &bak, "profile-backup.json", ProfileSavedAt(bak) });

	if (candidates.empty())
	{
		std::cout << "Profile is empty" << std::endl;
		ctx.Clear();
		return;
	}

	// Newest first. stable_sort keeps the registry ahead of the backup on a tie, so
	// the ordinary case (both stores identical) behaves exactly as before.
	std::stable_sort(candidates.begin(), candidates.end(),
		[](const Candidate &a, const Candidate &b) { return a.savedAt > b.savedAt; });

	for (const auto &candidate : candidates)
	{
		try
		{
			std::stringstream io(*candidate.str);
			ParseProfile(ctx, io);
			std::cout << "Loaded profile from " << candidate.name << std::endl;

			// Re-sync the stores whenever they disagree, so the divergence does not
			// persist and the next launch is not a coin flip.
			if (reg != bak)
			{
				std::cout << "Profile stores disagreed; rewriting both from "
					<< candidate.name << std::endl;
				SaveProfile(ctx);
			}
			return;
		}
		catch (const std::runtime_error &e)
		{
			std::cerr << "Error loading profile from " << candidate.name << ": " << e.what() << std::endl;
		}
	}

	std::cerr << "No usable profile found in registry or backup" << std::endl;
	ctx.Clear();
}

// V3-b: mirror the profile to a plain file so a lost registry key is recoverable.
static void WriteBackupFile(const std::string &str)
{
	char localAppData[MAX_PATH] = {};
	DWORD n = GetEnvironmentVariableA("LOCALAPPDATA", localAppData, MAX_PATH);
	if (n == 0 || n >= MAX_PATH)
		return;

	std::string dir = std::string(localAppData) + "\\OpenVR-SpaceOverride";
	CreateDirectoryA(dir.c_str(), nullptr);

	std::ofstream out(dir + "\\profile-backup.json", std::ios::trunc);
	if (out)
		out << str;
}

void RemoveProfile(CalibrationContext &ctx)
{
	// Deliberate deletion. SaveProfile refuses to write an invalid profile (so a clean
	// exit mid-setup cannot wipe a good calibration), which also meant the Remove
	// button silently did nothing: Clear() invalidates the profile, then SaveProfile
	// declined to persist it and the old registry value survived the restart. And with
	// the backup now being read on load, a cleared registry would restore from disk.
	// Removal therefore has to clear both stores explicitly.
	ctx.Clear();

	LSTATUS result = RegDeleteKeyValueA(HKEY_CURRENT_USER_LOCAL_SETTINGS, RegistryKey, "Config");
	if (result != ERROR_SUCCESS && result != ERROR_FILE_NOT_FOUND)
		LogRegistryResult(result);

	std::string backup = BackupFilePath();
	if (!backup.empty())
		DeleteFileA(backup.c_str());

	std::cout << "Removed calibration profile (registry + backup)" << std::endl;
}

void SaveProfile(CalibrationContext &ctx)
{
	// Never overwrite a good registry profile with empty/invalid state (e.g. clean exit mid-setup).
	if (!ctx.validProfile)
	{
		std::cout << "Skipping profile save (no valid profile)" << std::endl;
		return;
	}

	std::stringstream io;
	WriteProfile(ctx, io);
	const std::string str = io.str();

	const bool registryOk = WriteRegistryKey(str);
	WriteBackupFile(str);

	if (registryOk)
	{
		std::cout << "Saved profile (registry + backup)" << std::endl;
	}
	else
	{
		// Not fatal: the mirror carries the newer save timestamp, so LoadProfile will
		// prefer it on the next launch rather than silently reverting the calibration.
		std::cerr << "WARNING: registry profile write did not verify - "
			<< "profile-backup.json holds the current calibration" << std::endl;
	}
}
