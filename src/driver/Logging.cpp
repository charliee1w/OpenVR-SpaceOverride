// SPDX-License-Identifier: AGPL-3.0-only

#define _CRT_SECURE_NO_DEPRECATE
#include "Logging.h"

#include <cstdarg>
#include <chrono>
#include <cstring>
#include <windows.h>
#include <direct.h>

FILE *LogFile = nullptr;
static FILE *g_sessionLog = nullptr;
static char g_sessionPath[MAX_PATH] = {};
static char g_rollingPath[MAX_PATH] = {};

static void EnsureDir(const char *path)
{
	char tmp[MAX_PATH];
	strncpy(tmp, path, MAX_PATH - 1);
	tmp[MAX_PATH - 1] = 0;
	for (char *p = tmp + 3; *p; ++p)
	{
		if (*p == '\\' || *p == '/')
		{
			char c = *p;
			*p = 0;
			_mkdir(tmp);
			*p = c;
		}
	}
	_mkdir(tmp);
}

void OpenLogFile()
{
	g_sessionPath[0] = 0;
	g_rollingPath[0] = 0;
	g_sessionLog = nullptr;

	char localAppData[MAX_PATH] = {};
	DWORD n = GetEnvironmentVariableA("LOCALAPPDATA", localAppData, MAX_PATH);
	if (n == 0 || n >= MAX_PATH)
	{
		LogFile = fopen("spaceoverride_driver.log", "a");
		if (!LogFile)
			LogFile = stderr;
		return;
	}

	char logDir[MAX_PATH] = {};
	snprintf(logDir, MAX_PATH, "%s\\OpenVR-SpaceOverride\\logs", localAppData);
	EnsureDir(logDir);

	snprintf(g_rollingPath, MAX_PATH, "%s\\spaceoverride_driver.log", logDir);

	SYSTEMTIME st;
	GetLocalTime(&st);
	snprintf(g_sessionPath, MAX_PATH,
		"%s\\session_%04d%02d%02d_%02d%02d%02d.log",
		logDir, st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);

	LogFile = fopen(g_rollingPath, "a");
	if (!LogFile)
		LogFile = stderr;

	g_sessionLog = fopen(g_sessionPath, "w");

	if (LogFile && LogFile != stderr)
	{
		fprintf(LogFile, "\n======== session start ========\n");
		fprintf(LogFile, "session_file=%s\n", g_sessionPath);
		fprintf(LogFile, "rolling_file=%s\n", g_rollingPath);
		fflush(LogFile);
	}
	if (g_sessionLog)
	{
		fprintf(g_sessionLog, "session_file=%s\n", g_sessionPath);
		fprintf(g_sessionLog, "rolling_file=%s\n", g_rollingPath);
		fflush(g_sessionLog);
	}
}

void CloseLogFile()
{
	if (LogFile && LogFile != stderr)
	{
		fprintf(LogFile, "======== session end ========\n\n");
		fflush(LogFile);
		fclose(LogFile);
	}
	LogFile = nullptr;

	if (g_sessionLog)
	{
		fprintf(g_sessionLog, "======== session end ========\n");
		fflush(g_sessionLog);
		fclose(g_sessionLog);
		g_sessionLog = nullptr;
	}
}

const char *GetSessionLogPath()
{
	return g_sessionPath;
}

void LogSession(const char *fmt, ...)
{
	if (!g_sessionLog)
		return;

	tm logNow = TimeForLog();
	fprintf(g_sessionLog, "[%02d:%02d:%02d] ", logNow.tm_hour, logNow.tm_min, logNow.tm_sec);
	va_list args;
	va_start(args, fmt);
	vfprintf(g_sessionLog, fmt, args);
	va_end(args);
	fputc('\n', g_sessionLog);
	fflush(g_sessionLog);
}

tm TimeForLog()
{
	auto now = std::chrono::system_clock::now();
	auto nowTime = std::chrono::system_clock::to_time_t(now);
	tm value{};
	localtime_s(&value, &nowTime);
	return value;
}

void LogFlush()
{
	if (LogFile)
		fflush(LogFile);
}
