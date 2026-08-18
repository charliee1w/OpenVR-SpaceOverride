// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include <cstdio>
#include <ctime>

extern FILE *LogFile;

// Opens under %LOCALAPPDATA%\OpenVR-SpaceOverride\logs\ (called from driver Init, in the
// same position as upstream's open). Writes two files: spaceoverride_driver.log, appended
// for the life of the install and rolled at a size cap, and session_<stamp>.log, one per
// SteamVR start. Falls back to the process working directory if LOCALAPPDATA is unusable.
void OpenLogFile();
void CloseLogFile();

// Absolute path of the active session log, or "" if not open.
const char *GetSessionLogPath();

// Mirrors a line into the per-session file. Called for you by LOG(); there is no reason to
// call it directly.
void LogSession(const char *fmt, ...);

tm TimeForLog();
void LogFlush();

// ACCEPTANCE (why the happy-path output is bit-identical): this macro is log-only. Nothing
// in the driver reads a log line back, and the two changes below cannot alter any pose,
// calibration or IPC value:
//   - `if (LogFile)` is a real fix, not a behaviour change. Upstream's FILE* is a
//     zero-initialised global that is null before OpenLogFile and dangling after
//     CloseLogFile, and LOG() fires in both windows (Cleanup logs "unloaded" through the
//     handle it is about to close, and hook teardown can log after that). fprintf to a null
//     or dangling FILE* is UB; taking the branch only when the handle is live is the same
//     output whenever upstream was well-defined.
//   - LogSession() appends the same text to a second file and returns immediately when that
//     file is not open.
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
