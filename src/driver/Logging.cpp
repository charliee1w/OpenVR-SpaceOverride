// SPDX-License-Identifier: AGPL-3.0-only

#define _CRT_SECURE_NO_DEPRECATE
#include "Logging.h"
#include <chrono>

FILE *LogFile;

void OpenLogFile()
{
	LogFile = fopen("space_calibrator_driver.log", "a");
	if (LogFile == nullptr)
	{
		LogFile = stderr;
	}
}

void CloseLogFile()
{
	if (LogFile == nullptr || LogFile == stderr)
	{
		LogFile = nullptr;
		return;
	}
	fclose(LogFile);
	LogFile = nullptr;
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
	fflush(LogFile);
}