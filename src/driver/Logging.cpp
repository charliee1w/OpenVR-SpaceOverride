// SPDX-License-Identifier: AGPL-3.0-only

#define _CRT_SECURE_NO_DEPRECATE
#include "Logging.h"
#include <chrono>

FILE *LogFile = stderr;
static bool logFileOwned = false;

void OpenLogFile()
{
	FILE *file = fopen("space_calibrator_driver.log", "a");
	if (file == nullptr)
	{
		LogFile = stderr;
		logFileOwned = false;
		return;
	}

	if (logFileOwned && LogFile != nullptr && LogFile != stderr)
		fclose(LogFile);

	LogFile = file;
	logFileOwned = true;
}

void CloseLogFile()
{
	if (logFileOwned && LogFile != nullptr && LogFile != stderr)
		fclose(LogFile);

	LogFile = stderr;
	logFileOwned = false;
}

tm TimeForLog()
{
	auto now = std::chrono::system_clock::now();
	auto nowTime = std::chrono::system_clock::to_time_t(now);
	tm value;
	auto tm = localtime_s(&value, &nowTime);
	return value;
}

void LogFlush()
{
	if (LogFile)
		fflush(LogFile);
}