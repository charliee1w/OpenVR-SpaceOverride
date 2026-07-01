// SPDX-License-Identifier: AGPL-3.0-only

#include "Configuration.h"
#include "Calibration.h"
#include "FilterDefaults.h"

#include <Windows.h>

#include <picojson.h>

#include <string>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <limits>
#include <cmath>
#include <algorithm>
#include <cstddef>

static picojson::array FloatArray(const float *buf, int numFloats)
{
	picojson::array arr;

	for (int i = 0; i < numFloats; i++)
		arr.push_back(picojson::value(double(buf[i])));

	return arr;
}

static constexpr double ProfileMinScale = 0.01;
static constexpr double ProfileMaxScale = 100.0;
static constexpr std::size_t MaxChaperoneGeometryFloats = 4096;
static constexpr std::size_t MaxRegistryConfigBytes = 1024 * 1024;

static bool IsFiniteNumber(double value)
{
	return std::isfinite(value);
}

static void RequireFinite(double value, const char *field)
{
	if (!IsFiniteNumber(value))
		throw std::runtime_error(std::string("non-finite value in profile field: ") + field);
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

static void LoadOneEuroField(const picojson::object &obj, const char *key, protocol::OneEuroParams &out, protocol::OneEuroParams def)
{
	out = def;
	if (!obj.count(key) || !obj.at(key).is<picojson::object>())
		return;
	auto o = obj.at(key).get<picojson::object>();
	if (o.count("minCutoff") && o.at("minCutoff").is<double>()) out.minCutoff = o.at("minCutoff").get<double>();
	if (o.count("beta") && o.at("beta").is<double>())      out.beta = o.at("beta").get<double>();
	if (o.count("dCutoff") && o.at("dCutoff").is<double>())   out.dCutoff = o.at("dCutoff").get<double>();
}

static void LoadSettingsFields(CalibrationContext &ctx, const picojson::object &obj)
{
	if (obj.count("preferred_tracker_serial") && obj.at("preferred_tracker_serial").is<std::string>())
		ctx.preferredTrackerSerial = obj.at("preferred_tracker_serial").get<std::string>();

	if (obj.count("native") && obj.at("native").is<bool>())
		ctx.enableNative = obj.at("native").get<bool>();
	if (obj.count("fallbackSlam") && obj.at("fallbackSlam").is<bool>())
		ctx.fallbackToSlam = obj.at("fallbackSlam").get<bool>();
	else if (!obj.count("fallbackSlam"))
		ctx.fallbackToSlam = false;

	if (obj.count("eAngVel") && obj.at("eAngVel").is<bool>())
		ctx.enableAngularVelocity = obj.at("eAngVel").get<bool>();
	else if (!obj.count("eAngVel"))
		ctx.enableAngularVelocity = true;

	if (obj.count("continuousSync") && obj.at("continuousSync").is<bool>())
		ctx.continuousSync = obj.at("continuousSync").get<bool>();
	else if (!obj.count("continuousSync"))
		ctx.continuousSync = false;

	if (obj.count("syncHmdDrift") && obj.at("syncHmdDrift").is<bool>())
		ctx.syncHmdDrift = obj.at("syncHmdDrift").get<bool>();
	else if (!obj.count("syncHmdDrift"))
		ctx.syncHmdDrift = false;

	if (obj.count("applyCalToStandable") && obj.at("applyCalToStandable").is<bool>())
		ctx.applyCalToStandable = obj.at("applyCalToStandable").get<bool>();
	else if (!obj.count("applyCalToStandable"))
		ctx.applyCalToStandable = false;

	if (obj.count("quitStandableOnExit") && obj.at("quitStandableOnExit").is<bool>())
		ctx.quitStandableOnExit = obj.at("quitStandableOnExit").get<bool>();
	else if (!obj.count("quitStandableOnExit"))
		ctx.quitStandableOnExit = true;

	if (obj.count("predictionTime") && obj.at("predictionTime").is<double>())
		ctx.predictionTime = obj.at("predictionTime").get<double>();
	else if (!obj.count("predictionTime"))
		ctx.predictionTime = 1.0;
	RequireFinite(ctx.predictionTime, "predictionTime");
	ctx.predictionTime = std::clamp(ctx.predictionTime, 0.0f, 4.0f);

	if (obj.count("predictionAuto") && obj.at("predictionAuto").is<bool>())
		ctx.predictionAuto = obj.at("predictionAuto").get<bool>();
	else if (!obj.count("predictionAuto"))
		ctx.predictionAuto = false;

	if (obj.count("autoPartialRecalOnMountDrift") && obj.at("autoPartialRecalOnMountDrift").is<bool>())
		ctx.autoPartialRecalOnMountDrift = obj.at("autoPartialRecalOnMountDrift").get<bool>();
	else if (!obj.count("autoPartialRecalOnMountDrift"))
		ctx.autoPartialRecalOnMountDrift = false;

	if (obj.count("tundraMode") && obj.at("tundraMode").is<bool>())
		ctx.tundraMode = obj.at("tundraMode").get<bool>();
	else if (!obj.count("tundraMode"))
		ctx.tundraMode = false;

	ctx.headFilterEnabled = obj.count("headFilterEnabled") && obj.at("headFilterEnabled").is<bool>()
		? obj.at("headFilterEnabled").get<bool>()
		: false;
	if (obj.count("headFilter") && obj.at("headFilter").is<picojson::object>()
		|| obj.count("driftFilter") && obj.at("driftFilter").is<picojson::object>())
	{
		LoadOneEuroField(obj, "headFilter", ctx.headFilterParams, filter_defaults::Head);
		LoadOneEuroField(obj, "driftFilter", ctx.driftFilterParams, filter_defaults::Drift);
	}
	else
		ApplyFilterPresetsFromModes(ctx);

	if (obj.count("calibration_speed") && obj.at("calibration_speed").is<double>())
	{
		int speed = (int)obj.at("calibration_speed").get<double>();
		if (speed < 0 || speed > 2)
			throw std::runtime_error("invalid calibration_speed");
		ctx.calibrationSpeed = (CalibrationContext::Speed)speed;
	}

	if (obj.count("chaperone") && obj.at("chaperone").is<picojson::object>())
	{
		auto chaperone = obj.at("chaperone").get<picojson::object>();
		if (chaperone.count("auto_apply") && chaperone.at("auto_apply").is<bool>())
			ctx.chaperone.autoApply = chaperone.at("auto_apply").get<bool>();
	}
}

static picojson::object SaveOneEuroField(const protocol::OneEuroParams &p)
{
	picojson::object o;
	o["minCutoff"].set<double>(p.minCutoff);
	o["beta"].set<double>(p.beta);
	o["dCutoff"].set<double>(p.dCutoff);
	return o;
}

static void WriteSettingsFields(picojson::object &out, const CalibrationContext &ctx)
{
	const double predictionTime = ctx.predictionTime;
	const double calibrationSpeed = static_cast<double>(static_cast<int>(ctx.calibrationSpeed));

	out["preferred_tracker_serial"].set<std::string>(ctx.preferredTrackerSerial);
	out["native"].set<bool>(ctx.enableNative);
	out["fallbackSlam"].set<bool>(ctx.fallbackToSlam);
	out["eAngVel"].set<bool>(ctx.enableAngularVelocity);
	out["continuousSync"].set<bool>(ctx.continuousSync);
	out["syncHmdDrift"].set<bool>(ctx.syncHmdDrift);
	out["applyCalToStandable"].set<bool>(ctx.applyCalToStandable);
	out["quitStandableOnExit"].set<bool>(ctx.quitStandableOnExit);
	out["predictionTime"].set<double>(predictionTime);
	out["predictionAuto"].set<bool>(ctx.predictionAuto);
	out["autoPartialRecalOnMountDrift"].set<bool>(ctx.autoPartialRecalOnMountDrift);
	out["tundraMode"].set<bool>(ctx.tundraMode);
	out["headFilterEnabled"].set<bool>(ctx.headFilterEnabled);
	out["headFilter"].set<picojson::object>(SaveOneEuroField(ctx.headFilterParams));
	out["driftFilter"].set<picojson::object>(SaveOneEuroField(ctx.driftFilterParams));
	out["calibration_speed"].set<double>(calibrationSpeed);

	picojson::object chaperone;
	chaperone["auto_apply"].set<bool>(ctx.chaperone.autoApply);
	out["chaperone"].set<picojson::object>(chaperone);
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

	ctx.targetTrackingSystem = obj["target_tracking_system"].get<std::string>();

	if (obj["hmd_serial"].is<std::string>())
		ctx.hmdSerial = obj["hmd_serial"].get<std::string>();
	if (obj["hmd_tracking_system"].is<std::string>())
		ctx.hmdTrackingSystem = obj["hmd_tracking_system"].get<std::string>();
	if (obj["tracker_serial"].is<std::string>())
		ctx.trackerSerial = obj["tracker_serial"].get<std::string>();
	if (obj["preferred_tracker_serial"].is<std::string>())
		ctx.preferredTrackerSerial = obj["preferred_tracker_serial"].get<std::string>();
	if (obj["last_calibration_time"].is<double>())
		ctx.lastCalibrationTime = obj["last_calibration_time"].get<double>();
	if (obj["last_calibration_rms_mm"].is<double>())
		ctx.lastCalibrationRmsMm = obj["last_calibration_rms_mm"].get<double>();
	if (obj["baseline_mount_rms_mm"].is<double>())
		ctx.baselineMountRmsMm = obj["baseline_mount_rms_mm"].get<double>();
	if (obj["last_partial_mount_rms_mm"].is<double>())
		ctx.lastPartialMountRmsMm = obj["last_partial_mount_rms_mm"].get<double>();
	ctx.calibratedRotation(0) = obj["roll"].get<double>();
	ctx.calibratedRotation(1) = obj["yaw"].get<double>();
	ctx.calibratedRotation(2) = obj["pitch"].get<double>();
	ctx.calibratedTranslation(0) = obj["x"].get<double>();
	ctx.calibratedTranslation(1) = obj["y"].get<double>();
	ctx.calibratedTranslation(2) = obj["z"].get<double>();
	for (int i = 0; i < 3; ++i)
	{
		RequireFinite(ctx.calibratedRotation(i), "rotation");
		RequireFinite(ctx.calibratedTranslation(i), "translation");
	}

	if (obj["scale"].is<double>())
		ctx.calibratedScale = obj["scale"].get<double>();
	else
		ctx.calibratedScale = 1.0;
	RequireFinite(ctx.calibratedScale, "scale");
	if (ctx.calibratedScale < ProfileMinScale || ctx.calibratedScale > ProfileMaxScale)
		throw std::runtime_error("profile scale out of range");

	LoadSettingsFields(ctx, obj);

	if (obj["rel_qw"].is<double>())
	{
		ctx.relativeRotation.w = obj["rel_qw"].get<double>();
		ctx.relativeRotation.x = obj["rel_qx"].get<double>();
		ctx.relativeRotation.y = obj["rel_qy"].get<double>();
		ctx.relativeRotation.z = obj["rel_qz"].get<double>();
		ctx.relativeTranslation.v[0] = obj["rel_tx"].get<double>();
		ctx.relativeTranslation.v[1] = obj["rel_ty"].get<double>();
		ctx.relativeTranslation.v[2] = obj["rel_tz"].get<double>();
		RequireFinite(ctx.relativeRotation.w, "rel_qw");
		RequireFinite(ctx.relativeRotation.x, "rel_qx");
		RequireFinite(ctx.relativeRotation.y, "rel_qy");
		RequireFinite(ctx.relativeRotation.z, "rel_qz");
		RequireFinite(ctx.relativeTranslation.v[0], "rel_tx");
		RequireFinite(ctx.relativeTranslation.v[1], "rel_ty");
		RequireFinite(ctx.relativeTranslation.v[2], "rel_tz");
		const double relNorm = std::sqrt(
			ctx.relativeRotation.w * ctx.relativeRotation.w +
			ctx.relativeRotation.x * ctx.relativeRotation.x +
			ctx.relativeRotation.y * ctx.relativeRotation.y +
			ctx.relativeRotation.z * ctx.relativeRotation.z);
		ctx.validRelativeOffset = relNorm > 1e-8;
	}
	else
	{
		ctx.validRelativeOffset = false;
	}

	if (obj["chaperone"].is<picojson::object>())
	{
		auto chaperone = obj["chaperone"].get<picojson::object>();
		if (chaperone["auto_apply"].is<bool>())
			ctx.chaperone.autoApply = chaperone["auto_apply"].get<bool>();

		LoadFloatArray(chaperone["play_space_size"], ctx.chaperone.playSpaceSize.v, 2);

		LoadFloatArray(
			chaperone["standing_center"],
			(float *) ctx.chaperone.standingCenter.m,
			sizeof(ctx.chaperone.standingCenter.m) / sizeof(float)
		);

		if (!chaperone["geometry"].is<picojson::array>())
			throw std::runtime_error("chaperone geometry is not an array");

		auto &geometry = chaperone["geometry"].get<picojson::array>();

		if (geometry.size() > MaxChaperoneGeometryFloats)
			throw std::runtime_error("chaperone geometry too large");

		if (geometry.size() > 0)
		{
			ctx.chaperone.geometry.resize(geometry.size() * sizeof(float) / sizeof(ctx.chaperone.geometry[0]));
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
	profile["hmd_tracking_system"].set<std::string>(ctx.hmdTrackingSystem);
	profile["tracker_serial"].set<std::string>(ctx.trackerSerial);
	if (ctx.lastCalibrationTime > 0.0)
		profile["last_calibration_time"].set<double>(ctx.lastCalibrationTime);
	if (ctx.lastCalibrationRmsMm > 0.0)
		profile["last_calibration_rms_mm"].set<double>(ctx.lastCalibrationRmsMm);
	if (ctx.baselineMountRmsMm > 0.0)
		profile["baseline_mount_rms_mm"].set<double>(ctx.baselineMountRmsMm);
	if (ctx.lastPartialMountRmsMm > 0.0)
		profile["last_partial_mount_rms_mm"].set<double>(ctx.lastPartialMountRmsMm);
	profile["roll"].set<double>(ctx.calibratedRotation(0));
	profile["yaw"].set<double>(ctx.calibratedRotation(1));
	profile["pitch"].set<double>(ctx.calibratedRotation(2));
	profile["x"].set<double>(ctx.calibratedTranslation(0));
	profile["y"].set<double>(ctx.calibratedTranslation(1));
	profile["z"].set<double>(ctx.calibratedTranslation(2));
	profile["scale"].set<double>(ctx.calibratedScale);

	WriteSettingsFields(profile, ctx);

	if (ctx.validRelativeOffset)
	{
		profile["rel_qw"].set<double>(ctx.relativeRotation.w);
		profile["rel_qx"].set<double>(ctx.relativeRotation.x);
		profile["rel_qy"].set<double>(ctx.relativeRotation.y);
		profile["rel_qz"].set<double>(ctx.relativeRotation.z);
		profile["rel_tx"].set<double>(ctx.relativeTranslation.v[0]);
		profile["rel_ty"].set<double>(ctx.relativeTranslation.v[1]);
		profile["rel_tz"].set<double>(ctx.relativeTranslation.v[2]);
	}

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
static const char *RegistryConfigValue = "Config";
static const char *RegistryPrefsValue = "ConfigPrefs";

static std::string ReadRegistryValue(const char *valueName)
{
	DWORD size = 0;
	auto result = RegGetValueA(HKEY_CURRENT_USER_LOCAL_SETTINGS, RegistryKey, valueName, RRF_RT_REG_SZ, 0, 0, &size);
	if (result != ERROR_SUCCESS)
	{
		if (result != ERROR_FILE_NOT_FOUND)
			LogRegistryResult(result);
		return "";
	}

	if (size == 0 || size > MaxRegistryConfigBytes)
	{
		std::cerr << "Registry value size out of range (" << valueName << "): " << size << std::endl;
		return "";
	}

	std::string str;
	str.resize(size);

	result = RegGetValueA(HKEY_CURRENT_USER_LOCAL_SETTINGS, RegistryKey, valueName, RRF_RT_REG_SZ, 0, &str[0], &size);
	if (result != ERROR_SUCCESS)
	{
		LogRegistryResult(result);
		return "";
	}

	str.resize(size - 1);
	return str;
}

static void WriteRegistryValue(const char *valueName, const std::string &str)
{
	HKEY hkey;
	auto result = RegCreateKeyExA(HKEY_CURRENT_USER_LOCAL_SETTINGS, RegistryKey, 0, REG_NONE, 0, KEY_ALL_ACCESS, 0, &hkey, 0);
	if (result != ERROR_SUCCESS)
	{
		LogRegistryResult(result);
		return;
	}

	DWORD size = (DWORD)str.size() + 1;

	result = RegSetValueExA(hkey, valueName, 0, REG_SZ, reinterpret_cast<const BYTE*>(str.c_str()), size);
	if (result != ERROR_SUCCESS)
		LogRegistryResult(result);

	RegCloseKey(hkey);
}

static void DeleteRegistryValue(const char *valueName)
{
	HKEY hkey;
	auto result = RegOpenKeyExA(HKEY_CURRENT_USER_LOCAL_SETTINGS, RegistryKey, 0, KEY_ALL_ACCESS, &hkey);
	if (result != ERROR_SUCCESS)
		return;

	RegDeleteValueA(hkey, valueName);
	RegCloseKey(hkey);
}

static void LoadUserPreferences(CalibrationContext &ctx)
{
	auto str = ReadRegistryValue(RegistryPrefsValue);
	if (str.empty())
		return;

	try
	{
		picojson::value v;
		std::string err = picojson::parse(v, str);
		if (!err.empty())
			throw std::runtime_error(err);
		if (!v.is<picojson::object>())
			throw std::runtime_error("preferences root is not an object");

		LoadSettingsFields(ctx, v.get<picojson::object>());
		std::cout << "Loaded user preferences" << std::endl;
	}
	catch (const std::runtime_error &e)
	{
		std::cerr << "Error loading user preferences: " << e.what() << std::endl;
	}
}

static void SaveUserPreferences(const CalibrationContext &ctx)
{
	picojson::object prefs;
	WriteSettingsFields(prefs, ctx);
	picojson::value prefsV;
	prefsV.set<picojson::object>(prefs);
	WriteRegistryValue(RegistryPrefsValue, prefsV.serialize(true));
}

void LoadProfile(CalibrationContext &ctx)
{
	ctx.validProfile = false;
	InvalidateAppliedDriverState();
	LoadUserPreferences(ctx);

	auto str = ReadRegistryValue(RegistryConfigValue);
	if (str.empty())
	{
		std::cout << "Profile is empty" << std::endl;
		if (ReadRegistryValue(RegistryPrefsValue).empty())
			ctx.Clear();
		return;
	}

	try
	{
		std::stringstream io(str);
		ParseProfile(ctx, io);
		std::cout << "Loaded profile" << std::endl;
	}
	catch (const std::runtime_error &e)
	{
		std::cerr << "Error loading profile: " << e.what() << std::endl;
		ctx.Clear();
	}
}

void SaveProfile(CalibrationContext &ctx)
{
	SaveUserPreferences(ctx);

	if (!ctx.validProfile)
	{
		DeleteRegistryValue(RegistryConfigValue);
		return;
	}

	std::cout << "Saving profile to registry" << std::endl;

	std::stringstream io;
	WriteProfile(ctx, io);
	WriteRegistryValue(RegistryConfigValue, io.str());
}

bool ExportProfileToFile(const CalibrationContext &ctx, const std::string &path)
{
	if (!ctx.validProfile || path.empty())
		return false;

	std::ofstream out(path, std::ios::binary);
	if (!out)
	{
		std::cerr << "Failed to open profile export path: " << path << std::endl;
		return false;
	}

	CalibrationContext copy = ctx;
	std::stringstream io;
	WriteProfile(copy, io);
	out << io.str();
	return out.good();
}

bool ImportProfileFromFile(CalibrationContext &ctx, const std::string &path)
{
	if (path.empty())
		return false;

	std::ifstream in(path, std::ios::binary);
	if (!in)
	{
		std::cerr << "Failed to open profile import path: " << path << std::endl;
		return false;
	}

	try
	{
		ParseProfile(ctx, in);
		InvalidateAppliedDriverState();
		SaveProfile(ctx);
		std::cout << "Imported profile from " << path << std::endl;
		return ctx.validProfile;
	}
	catch (const std::runtime_error &e)
	{
		std::cerr << "Error importing profile: " << e.what() << std::endl;
		return false;
	}
}
