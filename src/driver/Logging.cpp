// SPDX-License-Identifier: AGPL-3.0-only

#define _CRT_SECURE_NO_DEPRECATE
#include "Logging.h"
#include <chrono>
#include <filesystem>

FILE *LogFile;
static bool logFileOwned = false;

static constexpr const char* kLogFileName = "space_calibrator_driver.log";
static constexpr std::uintmax_t kMaxLogBytes = 50u * 1024u * 1024u;

void OpenLogFile()
{
	try
	{
		const std::filesystem::path logPath(kLogFileName);
		if (std::filesystem::exists(logPath) && std::filesystem::file_size(logPath) > kMaxLogBytes)
		{
			const std::filesystem::path rotatedPath = std::string(kLogFileName) + ".old";
			std::filesystem::remove(rotatedPath);
			std::filesystem::rename(logPath, rotatedPath);
		}
	}
	catch (...) {}

	LogFile = fopen(kLogFileName, "a");
	logFileOwned = (LogFile != nullptr);
	if (!logFileOwned)
	{
		fprintf(stderr, "[OpenVR-SpaceOverride] Failed to open space_calibrator_driver.log — logging to stderr\n");
		LogFile = stderr;
	}
}

void CloseLogFile()
{
	if (logFileOwned && LogFile != nullptr)
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
	if (LogFile)
		fflush(LogFile);
}