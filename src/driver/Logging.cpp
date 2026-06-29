// SPDX-License-Identifier: AGPL-3.0-only

#define _CRT_SECURE_NO_DEPRECATE
#include "Logging.h"
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <unordered_map>
#include <windows.h>

FILE *LogFile;
static bool logFileOwned = false;
static std::filesystem::path g_logPath;
static std::uintmax_t g_bytesSinceRotateCheck = 0;

static constexpr const char* kLogFileName = "space_calibrator_driver.log";
static constexpr std::uintmax_t kMaxLogBytes = 50u * 1024u * 1024u;
static constexpr std::uintmax_t kRotateCheckIntervalBytes = 256u * 1024u;

static std::filesystem::path ResolveLogPath()
{
	const char* localAppData = std::getenv("LOCALAPPDATA");
	if (localAppData && localAppData[0] != '\0')
	{
		std::filesystem::path path(localAppData);
		path /= "openvr";
		path /= "logs";
		path /= kLogFileName;
		return path;
	}

	return std::filesystem::path(kLogFileName);
}

static void RotateLogFileIfNeeded(const std::filesystem::path& logPath)
{
	if (!std::filesystem::exists(logPath))
		return;

	if (std::filesystem::file_size(logPath) <= kMaxLogBytes)
		return;

	const std::filesystem::path rotatedPath = logPath.string() + ".old";
	std::filesystem::remove(rotatedPath);
	std::filesystem::rename(logPath, rotatedPath);
}

static bool ReopenLogFile()
{
	if (logFileOwned && LogFile != nullptr && LogFile != stderr)
		fclose(LogFile);

	try
	{
		std::filesystem::create_directories(g_logPath.parent_path());
		RotateLogFileIfNeeded(g_logPath);
	}
	catch (...) {}

	LogFile = fopen(g_logPath.string().c_str(), "a");
	logFileOwned = (LogFile != nullptr);
	if (!logFileOwned)
	{
		fprintf(stderr, "[OpenVR-SpaceOverride] Failed to open %s — logging to stderr\n", g_logPath.string().c_str());
		LogFile = stderr;
	}

	g_bytesSinceRotateCheck = 0;
	return logFileOwned;
}

void OpenLogFile()
{
	g_logPath = ResolveLogPath();

	try
	{
		RotateLogFileIfNeeded(g_logPath);
	}
	catch (...) {}

	ReopenLogFile();
}

void CloseLogFile()
{
	if (logFileOwned && LogFile != nullptr && LogFile != stderr)
		fclose(LogFile);
	LogFile = nullptr;
	logFileOwned = false;
}

tm TimeForLog()
{
	auto now = std::chrono::system_clock::now();
	auto nowTime = std::chrono::system_clock::to_time_t(now);
	tm value{};
	if (localtime_s(&value, &nowTime) != 0)
	{
		value.tm_hour = 0;
		value.tm_min = 0;
		value.tm_sec = 0;
	}
	return value;
}

void LogFlush()
{
	if (!LogFile)
		return;

	fflush(LogFile);

	if (!logFileOwned || LogFile == stderr)
		return;

	g_bytesSinceRotateCheck += 256;
	if (g_bytesSinceRotateCheck < kRotateCheckIntervalBytes)
		return;

	g_bytesSinceRotateCheck = 0;

	try
	{
		if (!std::filesystem::exists(g_logPath))
			return;

		if (std::filesystem::file_size(g_logPath) <= kMaxLogBytes)
			return;
	}
	catch (...)
	{
		return;
	}

	ReopenLogFile();
}

bool LogRateLimited(const char* key, unsigned intervalMs)
{
	static std::unordered_map<std::string, ULONGLONG> lastLogMs;
	const ULONGLONG now = GetTickCount64();
	const std::string id = key ? key : "";
	const auto it = lastLogMs.find(id);
	if (it != lastLogMs.end() && (now - it->second) < intervalMs)
		return false;

	lastLogMs[id] = now;
	return true;
}