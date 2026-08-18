// SPDX-License-Identifier: AGPL-3.0-only

#include "Configuration.h"

#include <Windows.h>

#include <picojson.h>

#include <string>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <limits>

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

	// ACCEPTANCE (A11): the vendored 3rdparty/PicoJSON/picojson.h:93-99 defines
	// PICOJSON_ASSERT(e) as `if (!(e)) throw std::runtime_error(#e);` unconditionally,
	// NDEBUG or not, and nothing in this tree overrides it. So a bare get<T>() on a
	// missing or wrongly-typed key already throws; it never returns garbage. When the
	// key IS present and correctly typed -- the case for every profile this build
	// writes -- reqDouble returns exactly what get<double>() returned, bit for bit.
	// What this buys is a defined failure point and a readable message on the recovery
	// path, not memory safety.
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
		ctx.targetModelScale = ctx.calibratedScale;

	if (obj["hmdScale"].is<double>())
		ctx.hmdScale = obj["hmdScale"].get<double>();
	else
		ctx.hmdScale = 1.0;

	if (ctx.targetModelScale <= 0.0)
		ctx.targetModelScale = 1.0;
	if (ctx.hmdScale <= 0.0)
		ctx.hmdScale = 1.0;

	// ACCEPTANCE (A11): when the key is present -- which it is in every profile this
	// build writes, since WriteProfile emits all three unconditionally -- optBool
	// returns exactly what the bare get<bool>() returned. The defaults are copied
	// byte-for-byte from CalibrationContext's own member initialisers
	// (Calibration.h:47-49: false / true / false), so a profile that predates these
	// fork-era keys loads with precisely the state a fresh context would have had.
	// Upstream threw here instead, which discarded an otherwise fully valid rotation,
	// translation and scale over three absent booleans.
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

	ctx.headFilterEnabled = obj["headFilterEnabled"].is<bool>() ? obj["headFilterEnabled"].get<bool>() : true;
	loadOneEuro("headFilter", ctx.headFilterParams, { 2.0, 0.5, 1.0 });
	loadOneEuro("driftFilter", ctx.driftFilterParams, { 1.0, 0.4, 0.85 });

	// ACCEPTANCE (A11): all seven or none. A complete rel_* block -- what every solve
	// writes -- takes the same branch and assigns the same seven doubles as upstream.
	// Upstream admitted the block on rel_qw alone and then read the other six
	// unchecked, so a truncated profile threw out of ParseProfile from inside the
	// block, after calibratedRotation/Translation had already been written.
	const bool relPresent[7] = {
		obj["rel_qw"].is<double>(), obj["rel_qx"].is<double>(), obj["rel_qy"].is<double>(),
		obj["rel_qz"].is<double>(), obj["rel_tx"].is<double>(), obj["rel_ty"].is<double>(),
		obj["rel_tz"].is<double>()
	};
	int relCount = 0;
	for (bool present : relPresent)
		relCount += present ? 1 : 0;

	if (relCount == 7)
	{
		ctx.relativeRotation.w = obj["rel_qw"].get<double>();
		ctx.relativeRotation.x = obj["rel_qx"].get<double>();
		ctx.relativeRotation.y = obj["rel_qy"].get<double>();
		ctx.relativeRotation.z = obj["rel_qz"].get<double>();
		ctx.relativeTranslation.v[0] = obj["rel_tx"].get<double>();
		ctx.relativeTranslation.v[1] = obj["rel_ty"].get<double>();
		ctx.relativeTranslation.v[2] = obj["rel_tz"].get<double>();
		ctx.validRelativeOffset = true;
	}
	else
	{
		ctx.validRelativeOffset = false;

		// HAZARD (A11, blueprint-mandated). Surviving a truncated profile is the point
		// of this commit, but it is not free: where upstream threw and refused the whole
		// profile, the profile now loads with no lever arm at all. In this tree
		// ScanAndApplyProfile computes
		//   overrideActive = enabled && validRelativeOffset && targetID != invalid
		// so the head override is silently switched OFF while the body devices in the
		// same pass still receive RequestSetDeviceTransform with the calibration -- a
		// half-applied calibration that looks like tracking drift rather than a load
		// failure. The blueprint quantifies the equivalent fork-side composition at
		// ~10 cm of head-vs-body error. Worse, WriteProfile gates the rel_* block on
		// validRelativeOffset, so the next save drops the surviving keys for good.
		// Loud on the console AND on a persistent banner in the UI (UserInterface.cpp);
		// this must never be a silent recovery.
		if (relCount > 0)
			std::cerr << "WARNING: profile has only " << relCount << " of 7 rel_* keys - "
				<< "head/tracker offset discarded, head override will stay OFF. Re-calibrate."
				<< std::endl;
		else
			std::cerr << "WARNING: profile carries no head/tracker offset (rel_*) - "
				<< "head override will stay OFF. Re-calibrate." << std::endl;
	}

	if (obj["calibration_speed"].is<double>())
		ctx.calibrationSpeed = (CalibrationContext::Speed)(int) obj["calibration_speed"].get<double>();

	if (obj["chaperone"].is<picojson::object>())
	{
		auto chaperone = obj["chaperone"].get<picojson::object>();
		// ACCEPTANCE (A11): same value whenever the key is present, and WriteProfile
		// writes it unconditionally inside this block. The default is
		// CalibrationContext's own initialiser (Calibration.h:72, autoApply = true).
		// Same reasoning as native/fallbackSlam/eAngVel: throwing here escapes
		// ParseProfile and costs the whole calibration over an absent cosmetic flag.
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

		// ACCEPTANCE (A11): the element type is vr::HmdQuad_t, a quad of 12 floats, so
		// for any array this build ever wrote size is an exact multiple of 12 and
		// `size * 4 / 48` == `size / 12` -- the same element count, the same resize,
		// the same LoadFloatArray. Upstream's expression truncated on a non-multiple,
		// under-allocating and then writing the full array into it: a heap overflow
		// driven by user-writable registry content (and a write through data() on an
		// empty vector when size < 12).
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

static void WriteRegistryKey(std::string str)
{
	HKEY hkey;
	auto result = RegCreateKeyExA(HKEY_CURRENT_USER_LOCAL_SETTINGS, RegistryKey, 0, REG_NONE, 0, KEY_ALL_ACCESS, 0, &hkey, 0);
	if (result != ERROR_SUCCESS)
	{
		LogRegistryResult(result);
		return;
	}

	DWORD size = str.size() + 1;

	result = RegSetValueExA(hkey, "Config", 0, REG_SZ, reinterpret_cast<const BYTE*>(str.c_str()), size);
	if (result != ERROR_SUCCESS)
		LogRegistryResult(result);

	RegCloseKey(hkey);
}

void LoadProfile(CalibrationContext &ctx)
{
	ctx.validProfile = false;

	auto str = ReadRegistryKey();
	if (str == "")
	{
		std::cout << "Profile is empty" << std::endl;
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
	}
}

void RemoveProfile(CalibrationContext &ctx)
{
	// Deliberate deletion. SaveProfile now refuses to write an invalid profile, which
	// also means the Remove button would silently do nothing: Clear() invalidates the
	// profile, SaveProfile declines to persist it, and the old registry value survives
	// the restart. Removal therefore has to clear the store explicitly.
	ctx.Clear();

	LSTATUS result = RegDeleteKeyValueA(HKEY_CURRENT_USER_LOCAL_SETTINGS, RegistryKey, "Config");
	if (result != ERROR_SUCCESS && result != ERROR_FILE_NOT_FOUND)
		LogRegistryResult(result);

	std::cout << "Removed calibration profile" << std::endl;
}

void SaveProfile(CalibrationContext &ctx)
{
	// ACCEPTANCE (A10): identical whenever validProfile is true, which is every save
	// that follows a solve or an edit -- WriteProfile's own first statement is the
	// same test, so the serialized bytes and the registry write are unchanged.
	// What changes is only the invalid case. Upstream's WriteProfile early-returned
	// leaving the stream EMPTY, and SaveProfile then wrote that empty string to the
	// registry, wiping a good calibration on any invalid-context save -- including the
	// unconditional exit-time SaveProfile in Main.cpp.
	if (!ctx.validProfile)
	{
		std::cout << "Skipping profile save (no valid profile)" << std::endl;
		return;
	}

	std::cout << "Saving profile to registry" << std::endl;

	std::stringstream io;
	WriteProfile(ctx, io);
	WriteRegistryKey(io.str());
}
