// SPDX-License-Identifier: AGPL-3.0-only

#define _CRT_SECURE_NO_DEPRECATE
#include "Logging.h"

#include <cstdarg>
#include <chrono>
#include <cstring>
#include <mutex>
#include <windows.h>
#include <direct.h>

FILE *LogFile = nullptr;
static FILE *g_sessionLog = nullptr;
static FILE *g_diagCsv = nullptr;
// The CSV can be opened/closed live from the server thread (overlay toggle) while
// pose threads write rows — guard the handle.
static std::mutex g_diagMutex;
static char g_sessionPath[MAX_PATH] = {};
static char g_rollingPath[MAX_PATH] = {};
static char g_diagPath[MAX_PATH] = {};

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

// V3-a: one session file per SteamVR start adds up — drop sessions older than 14 days.
static void PruneOldSessionLogs(const char *logDir)
{
	char pattern[MAX_PATH];
	snprintf(pattern, MAX_PATH, "%s\\session_*.log", logDir);

	FILETIME ftNow;
	GetSystemTimeAsFileTime(&ftNow);
	ULARGE_INTEGER now;
	now.LowPart = ftNow.dwLowDateTime;
	now.HighPart = ftNow.dwHighDateTime;
	const unsigned long long maxAge100ns = 14ULL * 24 * 60 * 60 * 10000000ULL;

	WIN32_FIND_DATAA fd;
	HANDLE h = FindFirstFileA(pattern, &fd);
	if (h == INVALID_HANDLE_VALUE)
		return;
	do
	{
		ULARGE_INTEGER wt;
		wt.LowPart = fd.ftLastWriteTime.dwLowDateTime;
		wt.HighPart = fd.ftLastWriteTime.dwHighDateTime;
		if (now.QuadPart > wt.QuadPart && (now.QuadPart - wt.QuadPart) > maxAge100ns)
		{
			char full[MAX_PATH];
			snprintf(full, MAX_PATH, "%s\\%s", logDir, fd.cFileName);
			DeleteFileA(full);
		}
	} while (FindNextFileA(h, &fd));
	FindClose(h);
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
	PruneOldSessionLogs(logDir);

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

void OpenDiagCsv()
{
	std::lock_guard<std::mutex> lock(g_diagMutex);
	if (g_diagCsv)
		return;

	char localAppData[MAX_PATH] = {};
	DWORD n = GetEnvironmentVariableA("LOCALAPPDATA", localAppData, MAX_PATH);
	if (n == 0 || n >= MAX_PATH)
		return;

	char logDir[MAX_PATH] = {};
	snprintf(logDir, MAX_PATH, "%s\\OpenVR-SpaceOverride\\logs", localAppData);

	SYSTEMTIME st;
	GetLocalTime(&st);
	snprintf(g_diagPath, MAX_PATH,
		"%s\\fusion_diag_%04d%02d%02d_%02d%02d%02d.csv",
		logDir, st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);

	g_diagCsv = fopen(g_diagPath, "w");
	if (g_diagCsv)
	{
		fprintf(g_diagCsv, "t_ms,speed_mps,angspeed_rps,disp_cm,Kt,sig_cm,gated,event\n");
		fflush(g_diagCsv);
	}
}

void CloseDiagCsv()
{
	std::lock_guard<std::mutex> lock(g_diagMutex);
	if (g_diagCsv)
	{
		fflush(g_diagCsv);
		fclose(g_diagCsv);
		g_diagCsv = nullptr;
	}
}

bool DiagCsvOpen()
{
	std::lock_guard<std::mutex> lock(g_diagMutex);
	return g_diagCsv != nullptr;
}

void LogDiagCsv(const char *fmt, ...)
{
	std::lock_guard<std::mutex> lock(g_diagMutex);
	if (!g_diagCsv)
		return;

	va_list args;
	va_start(args, fmt);
	vfprintf(g_diagCsv, fmt, args);
	va_end(args);
	fputc('\n', g_diagCsv);
	fflush(g_diagCsv);
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
