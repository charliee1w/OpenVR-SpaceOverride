// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include <cstdio>
#include <ctime>

extern FILE *LogFile;

void OpenLogFile();
void CloseLogFile();

tm TimeForLog();
void LogFlush();

// Returns true when enough time elapsed since the last log with this key.
bool LogRateLimited(const char* key, unsigned intervalMs = 5000);

inline FILE* LogOutput()
{
	return LogFile ? LogFile : stderr;
}

#ifndef LOG
#define LOG(fmt, ...) do { \
	tm logNow = TimeForLog(); \
	fprintf(LogOutput(), "[%02d:%02d:%02d] " fmt "\n", logNow.tm_hour, logNow.tm_min, logNow.tm_sec, ##__VA_ARGS__); \
	LogFlush(); \
} while (0)
#endif

#ifndef TRACE
#define TRACE LOG
#endif
