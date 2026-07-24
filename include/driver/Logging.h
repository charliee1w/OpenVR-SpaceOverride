// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include <cstdio>
#include <ctime>

extern FILE *LogFile;

// Opens under %LOCALAPPDATA%\OpenVR-SpaceOverride\logs\ (auto on driver Init).
// Writes spaceoverride_driver.log (append) and a per-session file.
void OpenLogFile();
void CloseLogFile();

// Absolute path of the active session log, or "" if not open.
const char *GetSessionLogPath();

// Also writes to the per-session file (for jump/event diagnostics).
void LogSession(const char *fmt, ...);

// Per-frame fusion diagnostic CSV (fusion_diag_<ts>.csv in the logs dir).
// Opened only when fusionMode && fusionDiag; header written on open.
void OpenDiagCsv();
void CloseDiagCsv();
bool DiagCsvOpen();
void LogDiagCsv(const char *fmt, ...);

tm TimeForLog();
void LogFlush();

#ifndef LOG
#define LOG(fmt, ...) do { \
	if (LogFile) { \
		tm logNow = TimeForLog(); \
		fprintf(LogFile, "[%02d:%02d:%02d] " fmt "\n", logNow.tm_hour, logNow.tm_min, logNow.tm_sec, __VA_ARGS__); \
		LogFlush(); \
	} \
	LogSession(fmt, ##__VA_ARGS__); \
} while (0)
#endif

#define TRACE(...) {}

#ifndef TRACE
#define TRACE LOG
#endif
