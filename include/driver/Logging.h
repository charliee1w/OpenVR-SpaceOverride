// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include <cstdio>
#include <ctime>

extern FILE *LogFile;

void OpenLogFile();
void CloseLogFile();

tm TimeForLog();
void LogFlush();
void LogMessage(const char* fmt, ...);

// Returns true when enough time elapsed since the last log with this key.
bool LogRateLimited(const char* key, unsigned intervalMs = 5000);

inline FILE* LogOutput()
{
	return LogFile ? LogFile : stderr;
}

#ifndef LOG
#define LOG(fmt, ...) LogMessage(fmt, ##__VA_ARGS__)
#endif

#ifndef TRACE
#define TRACE LOG
#endif
