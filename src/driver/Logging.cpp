// SPDX-License-Identifier: AGPL-3.0-only

#define _CRT_SECURE_NO_DEPRECATE
#include "Logging.h"

#include <chrono>
#include <cstdarg>
#include <cstring>
#include <mutex>
#include <windows.h>
#include <direct.h>

// ACCEPTANCE (why the happy-path output is bit-identical): every line in this file is
// log-only. The driver never reads a log back -- there is no parse, no seek, no size check
// feeding any decision -- so where the bytes land, how long they are kept and how they are
// serialised cannot reach a pose, a calibration value or an IPC message. The only observable
// effects are on disk, plus the two defects fixed below (a null/dangling FILE* dereference
// and an exit(EXIT_FAILURE) taken inside vrserver).
//
// Threading note, recorded because this file owns the FILE* that four threads reach through
// (the IPC server thread, the HMD pose thread, the tracker pose thread and the main thread):
// there is exactly ONE mutex here, g_sessionMutex, and it is never held across a call to
// anything that could take another lock. A single lock cannot participate in a lock-order
// inversion, which is why this file states its rule as a count rather than as an ordering.

FILE *LogFile = nullptr;
static FILE *g_sessionLog = nullptr;
// Serialises whole lines into the session log; see LogSession.
static std::mutex g_sessionMutex;
static char g_sessionPath[MAX_PATH] = {};
static char g_rollingPath[MAX_PATH] = {};

static void EnsureDir(const char *path)
{
	char tmp[MAX_PATH];
	strncpy(tmp, path, MAX_PATH - 1);
	tmp[MAX_PATH - 1] = 0;

	// Start past the drive prefix ("C:\") so the first separator does not produce a bare
	// "C:" _mkdir. Guarded rather than assumed: a shorter string must not be walked.
	const size_t len = strlen(tmp);
	if (len > 3)
	{
		for (char *p = tmp + 3; *p; ++p)
		{
			if (*p == '\\' || *p == '/')
			{
				const char c = *p;
				*p = 0;
				_mkdir(tmp);
				*p = c;
			}
		}
	}
	_mkdir(tmp);
}

// The rolling log is opened "a" and appended to for the life of the install. Upstream gave it
// no bound at all, in the SteamVR install directory.
static const long kRollingMaxBytes = 8L * 1024 * 1024;

// Drop matching files older than 14 days. One session log is written per SteamVR start, so
// the directory would otherwise grow without bound.
static void PruneOldLogs(const char *logDir, const char *glob)
{
	char pattern[MAX_PATH];
	snprintf(pattern, MAX_PATH, "%s\\%s", logDir, glob);

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
		if (fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)
			continue;
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

// Keep the rolling log bounded. Renaming to a single .1 rather than deleting preserves the
// most recent history across the rollover, which is the part anyone reads.
static void RollIfLarge(const char *path, long maxBytes)
{
	WIN32_FILE_ATTRIBUTE_DATA fad;
	if (!GetFileAttributesExA(path, GetFileExInfoStandard, &fad))
		return;
	ULARGE_INTEGER sz;
	sz.LowPart = fad.nFileSizeLow;
	sz.HighPart = fad.nFileSizeHigh;
	if (sz.QuadPart < (unsigned long long)maxBytes)
		return;

	char prev[MAX_PATH];
	snprintf(prev, MAX_PATH, "%s.1", path);
	DeleteFileA(prev);
	MoveFileA(path, prev);
}

void OpenLogFile()
{
	g_sessionPath[0] = 0;
	g_rollingPath[0] = 0;
	g_sessionLog = nullptr;

	// Upstream wrote space_calibrator_driver.log into the process working directory, which
	// for vrserver is under Program Files -- so on a default install the open fails on
	// permissions and every line goes to stderr, i.e. nowhere. LOCALAPPDATA is writable and
	// is where the overlay already keeps its own state.
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
	PruneOldLogs(logDir, "session_*.log");

	snprintf(g_rollingPath, MAX_PATH, "%s\\spaceoverride_driver.log", logDir);
	RollIfLarge(g_rollingPath, kRollingMaxBytes);

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
	// Upstream called std::exit(EXIT_FAILURE) when fclose returned nonzero. This runs inside
	// vrserver during driver teardown: a full-disk or network-share hiccup on the log file
	// would terminate SteamVR's server process. A log sink cannot be allowed to decide that.
	if (LogFile && LogFile != stderr)
	{
		fprintf(LogFile, "======== session end ========\n\n");
		fflush(LogFile);
		fclose(LogFile);
	}
	LogFile = nullptr;

	std::lock_guard<std::mutex> lock(g_sessionMutex);
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

	// One write per line, under a mutex. The rolling log's equivalent is three unsynchronised
	// fprintf calls issued from at least four threads -- the IPC server, the tracker pose
	// thread, the HMD pose thread and the main thread. The MSVC CRT locks each FILE, so
	// nothing corrupts, but the calls can interleave and splice two lines together. Session
	// logs are this project's stated authority on what a build actually did, so a spliced
	// line is a corrupted record rather than a cosmetic defect.
	tm logNow = TimeForLog();
	char body[1024];
	va_list args;
	va_start(args, fmt);
	vsnprintf(body, sizeof body, fmt, args);
	va_end(args);

	std::lock_guard<std::mutex> lock(g_sessionMutex);
	// Re-checked under the lock: CloseLogFile can have closed the handle between the early
	// return above and here.
	if (!g_sessionLog)
		return;
	fprintf(g_sessionLog, "[%02d:%02d:%02d] %s\n", logNow.tm_hour, logNow.tm_min, logNow.tm_sec, body);
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
