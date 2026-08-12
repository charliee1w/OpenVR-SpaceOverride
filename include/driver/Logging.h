// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include <cstdio>
#include <ctime>
#include <cstdint>

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
// Opened by the driver only while both fusionMode and fusionDiag are set (the CSV
// only records fusion-path rows); header written on open.
void OpenDiagCsv();
void CloseDiagCsv();
bool DiagCsvOpen();
void LogDiagCsv(const char *fmt, ...);

// ---------------------------------------------------------------------------------------------
// Unified capture (capture_<ts>.sor) — the one machine-readable sink. See CaptureFormat.h for
// why this replaces per-question CSVs and how it stays forward-compatible.
//
// SetCaptureEnabled() is the ONLY entry point: idempotent, drives open and close, and is safe to
// call from anywhere at any rate. Diagnostics previously had Open*/Close* pairs invoked from two
// different places, and a sink wired to only one of them shipped silently broken. One function
// with the desired state cannot have that failure mode.
void SetCaptureEnabled(bool enabled);
bool CaptureEnabled();

// Which capture FILE is open: increments on each open, 0 while closed. For records that must
// appear once per file rather than once per driver session (Rec_DeviceInfo) — a re-opened sink
// starts a new file, and that file has to describe its devices from scratch.
uint32_t CaptureGeneration();

// Append one record. `len` is payload bytes; framing is written for you. No-op when disabled, so
// call sites need no guard. Buffered — the pose thread does not touch the disk per record.
void CaptureWrite(uint16_t type, const void *payload, uint16_t len);

// Convenience for the variable-length event record (mirrors a session-log line into the capture
// so a file is self-contained for time correlation).
void CaptureEvent(int64_t clockNow, const char *text);

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
