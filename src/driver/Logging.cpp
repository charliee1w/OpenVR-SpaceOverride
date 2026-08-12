// SPDX-License-Identifier: AGPL-3.0-only

#define _CRT_SECURE_NO_DEPRECATE
#include "Logging.h"
#include "CaptureFormat.h"

#include <algorithm>
#include <atomic>
#include <cstdarg>
#include <chrono>
#include <cstring>
#include <mutex>
#include <string>
#include <vector>
#include <windows.h>
#include <direct.h>

FILE *LogFile = nullptr;
static FILE *g_sessionLog = nullptr;
// Serialises whole lines into the session log; see LogSession.
static std::mutex g_sessionMutex;
static FILE *g_diagCsv = nullptr;
// The CSV can be opened/closed live from the server thread (overlay toggle) while
// pose threads write rows — guard the handle.
static std::mutex g_diagMutex;
// Per-frame rows are accumulated in memory and written in batches. The previous
// fflush-per-row hit the pose thread with a syscall + disk write on every row (up
// to ~90/s on a sustained-gate stretch). Buffering turns that into one write per
// ~few thousand rows; the tail (< kDiagFlushBytes) is flushed on close.
static std::string g_diagBuf;
static const size_t kDiagFlushBytes = 64 * 1024;
static char g_sessionPath[MAX_PATH] = {};
static char g_rollingPath[MAX_PATH] = {};
static char g_diagPath[MAX_PATH] = {};

// Unified capture (CaptureFormat.h). One handle, one buffer, one lifecycle — and TWO mutexes,
// split so the pose path never waits on the disk:
//
//   g_captureMutex   guards the handle pointer and the in-memory buffer. Held only for
//                    appends and pointer swaps — never across file I/O.
//   g_captureIoMutex serialises the actual fwrite/fflush/fclose. Taken BEFORE g_captureMutex
//                    wherever both are needed (open/close/flush); never the other way round.
//
// The previous single-mutex design held g_captureMutex across the 64 KiB flush (~every 1.5 s at
// the measured 149 MB/h), so an AV scan or HDD spin-up stalling that fwrite blocked EVERY pose
// thread in vrserver at once — HMD, controllers and pucks freezing together — against
// Logging.h's own "the pose thread does not touch the disk per record" contract. Now a stall
// blocks only the one thread that crossed the flush threshold; everyone else keeps appending.
static FILE *g_capture = nullptr;
static std::mutex g_captureMutex;
static std::mutex g_captureIoMutex;
static std::string g_captureBuf;
static char g_capturePath[MAX_PATH] = {};
// Incremented on every open. Callers that must emit a record once PER FILE (Rec_DeviceInfo)
// compare against this instead of a bool, so re-opening the sink mid-session re-describes every
// device — a capture file has to be self-contained, not a continuation of the previous one.
static uint32_t g_captureGen = 0;
// Lock-free mirror for the hot-path readers: the current generation while open, 0 while closed.
// CaptureEnabled()/CaptureGeneration() used to take g_captureMutex, which put every pose
// callback's capture test behind the same lock a flush could be stalled on.
static std::atomic<uint32_t> g_capturePublishedGen{ 0 };

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

// Total bytes of capture files to keep. Captures are by far the largest artefact this driver
// writes — measured at 149 MB/hour (159,943,151 bytes over 64.2 min, session_20260806_200502) —
// and the age rule alone let a fortnight of daily play accumulate tens of GB in LOCALAPPDATA.
// Age still applies; this is the second bound, oldest-first.
static const unsigned long long kCaptureKeepBytes = 4ULL * 1024 * 1024 * 1024;
// The rolling log is opened "a" and appended to for the life of the install. It was the only
// sink with no rule at all.
static const long kRollingMaxBytes = 8L * 1024 * 1024;

// Drop matching files older than 14 days. One session log and (in fusion) one
// per-frame CSV are written per SteamVR start, so both would grow without bound.
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

// Second bound for captures: keep total bytes under a cap, deleting oldest first. Age-based
// pruning cannot bound a sink whose rate is 149 MB/hour.
static void PruneBySize(const char *logDir, const char *glob, unsigned long long keepBytes)
{
	char pattern[MAX_PATH];
	snprintf(pattern, MAX_PATH, "%s\\%s", logDir, glob);

	struct Entry
	{
		unsigned long long write;
		unsigned long long size;
		char name[MAX_PATH];
	};
	std::vector<Entry> files;

	WIN32_FIND_DATAA fd;
	HANDLE h = FindFirstFileA(pattern, &fd);
	if (h == INVALID_HANDLE_VALUE)
		return;
	do
	{
		if (fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)
			continue;
		Entry e{};
		ULARGE_INTEGER wt;
		wt.LowPart = fd.ftLastWriteTime.dwLowDateTime;
		wt.HighPart = fd.ftLastWriteTime.dwHighDateTime;
		ULARGE_INTEGER sz;
		sz.LowPart = fd.nFileSizeLow;
		sz.HighPart = fd.nFileSizeHigh;
		e.write = wt.QuadPart;
		e.size = sz.QuadPart;
		strncpy(e.name, fd.cFileName, MAX_PATH - 1);
		files.push_back(e);
	} while (FindNextFileA(h, &fd));
	FindClose(h);

	unsigned long long total = 0;
	for (auto &f : files)
		total += f.size;
	if (total <= keepBytes)
		return;

	std::sort(files.begin(), files.end(),
		[](const Entry &a, const Entry &b) { return a.write < b.write; });

	for (auto &f : files)
	{
		if (total <= keepBytes)
			break;
		char full[MAX_PATH];
		snprintf(full, MAX_PATH, "%s\\%s", logDir, f.name);
		if (DeleteFileA(full))
			total -= f.size;
	}
}

// Keep the rolling log bounded. Renaming to a single .1 rather than deleting preserves the most
// recent history across the rollover, which is the part anyone reads.
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
	PruneOldLogs(logDir, "fusion_diag_*.csv");
	PruneOldLogs(logDir, "capture_*.sor");
	PruneOldLogs(logDir, "body_diag_*.csv");
	PruneBySize(logDir, "capture_*.sor", kCaptureKeepBytes);

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
	// Mirror EVERY session-log line into the capture, and do it BEFORE the g_sessionLog guard.
	// Rec_Event was defined but had no call site, so the first real capture contained zero
	// events and could not explain its own 32 s frame gap. Keying this off the other sink's
	// handle would be the same two-lifecycle mistake that shipped body_diag broken — the
	// capture must not go quiet because a different file failed to open.
	//
	// CaptureEvent is a no-op while the sink is closed, so this costs a mutex on log lines only
	// (~1/s at heartbeat rate), never on the pose path.
	{
		va_list capArgs;
		va_start(capArgs, fmt);
		char line[1024];
		const int len = vsnprintf(line, sizeof line, fmt, capArgs);
		va_end(capArgs);
		if (len >= 0)
		{
			LARGE_INTEGER now{};
			QueryPerformanceCounter(&now);   // same clock domain as Rec_Frame.clockNow
			CaptureEvent(now.QuadPart, line);
		}
	}

	if (!g_sessionLog)
		return;

	// One write per line, under a mutex. This used to be three unsynchronised calls (timestamp,
	// body, newline) issued from at least four threads — the IPC server, the tracker pose
	// thread (R9), the HMD pose thread, and the main thread. The MSVC CRT locks each FILE, so
	// nothing corrupted, but the three calls could interleave and splice two lines together.
	// The session log is this project's stated authority on what a build actually did, so a
	// spliced line is a corrupted record, not a cosmetic defect.
	tm logNow = TimeForLog();
	char body[1024];
	va_list args;
	va_start(args, fmt);
	vsnprintf(body, sizeof body, fmt, args);
	va_end(args);

	std::lock_guard<std::mutex> lock(g_sessionMutex);
	fprintf(g_sessionLog, "[%02d:%02d:%02d] %s\n", logNow.tm_hour, logNow.tm_min, logNow.tm_sec, body);
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
		g_diagBuf.clear();
		g_diagBuf.reserve(kDiagFlushBytes + 1024);
		// Columns: time; mode; head motion; estimator innovation/gain/uncertainty;
		// how large the correction currently is; the world head position (so a
		// trajectory and its spatial coverage can be reconstructed); gate outcome.
		fputs("t_ms,mode,speed_mps,angspeed_rps,disp_cm,Kt,sig_cm,corr_cm,corr_yaw_deg,wx,wy,wz,gated,event\n", g_diagCsv);
		fflush(g_diagCsv);
	}
}

// Caller must hold g_diagMutex.
static void FlushDiagBufLocked()
{
	if (g_diagCsv && !g_diagBuf.empty())
	{
		fwrite(g_diagBuf.data(), 1, g_diagBuf.size(), g_diagCsv);
		fflush(g_diagCsv);
		g_diagBuf.clear();
	}
}

void CloseDiagCsv()
{
	std::lock_guard<std::mutex> lock(g_diagMutex);
	if (g_diagCsv)
	{
		FlushDiagBufLocked();
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
	// Format into a small stack buffer, then append to the in-memory batch. No
	// per-row file I/O: the batch is written only when it crosses kDiagFlushBytes
	// (and on close), so the pose thread does at most one write per few thousand rows.
	char row[256];
	va_list args;
	va_start(args, fmt);
	int len = vsnprintf(row, sizeof row - 1, fmt, args);
	va_end(args);
	if (len < 0)
		return;
	if (len > (int)sizeof row - 2)
		len = (int)sizeof row - 2;
	row[len++] = '\n';

	std::lock_guard<std::mutex> lock(g_diagMutex);
	if (!g_diagCsv)
		return;
	g_diagBuf.append(row, len);
	if (g_diagBuf.size() >= kDiagFlushBytes)
		FlushDiagBufLocked();
}

// ---- Unified capture --------------------------------------------------------------------
// One sink, one lifecycle. Callers state the DESIRED state via SetCaptureEnabled(); they never
// open or close directly, which is what makes a half-wired second call site impossible.

// Caller must hold g_captureIoMutex AND g_captureMutex (open/close path only).
static void FlushCaptureLocked()
{
	if (g_capture && !g_captureBuf.empty())
	{
		fwrite(g_captureBuf.data(), 1, g_captureBuf.size(), g_capture);
		fflush(g_capture);
		g_captureBuf.clear();
	}
}

// Hot-path flush: swap the buffer out under g_captureMutex, then write with only the io mutex
// held, so appends from other pose threads proceed during the disk write. try_lock — if another
// thread is mid-flush (or the sink is being toggled), skip; the data stays buffered and the
// next threshold crossing retries. The io mutex being held across the fwrite is what keeps
// CloseCaptureLocked (whose callers take io first) from fclosing the handle under us.
static void TryFlushCapture()
{
	if (!g_captureIoMutex.try_lock())
		return;
	std::lock_guard<std::mutex> io(g_captureIoMutex, std::adopt_lock);

	std::string out;
	FILE *f = nullptr;
	{
		std::lock_guard<std::mutex> lock(g_captureMutex);
		if (!g_capture || g_captureBuf.empty())
			return;
		out.swap(g_captureBuf);
		f = g_capture;
	}
	fwrite(out.data(), 1, out.size(), f);
	fflush(f);
}

// Caller must hold g_captureIoMutex AND g_captureMutex.
static void OpenCaptureLocked()
{
	if (g_capture)
		return;

	char localAppData[MAX_PATH] = {};
	DWORD n = GetEnvironmentVariableA("LOCALAPPDATA", localAppData, MAX_PATH);
	if (n == 0 || n >= MAX_PATH)
		return;

	char logDir[MAX_PATH] = {};
	snprintf(logDir, MAX_PATH, "%s\\OpenVR-SpaceOverride\\logs", localAppData);

	SYSTEMTIME st;
	GetLocalTime(&st);
	snprintf(g_capturePath, MAX_PATH,
		"%s\\capture_%04d%02d%02d_%02d%02d%02d.sor",
		logDir, st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);

	g_capture = fopen(g_capturePath, "wb");
	if (!g_capture)
		return;

	g_captureBuf.clear();
	g_captureBuf.reserve(kDiagFlushBytes + 4096);

	// File header: magic, versions, then the embedded schema. A capture is interpretable years
	// from now without this repo, and a reader can reject a format it cannot parse.
	const uint32_t schemaLen = (uint32_t)strlen(capture::kSchemaText);
	fwrite(capture::kMagic, 1, sizeof capture::kMagic, g_capture);
	fwrite(&capture::kFormatVersion, sizeof(uint32_t), 1, g_capture);
	fwrite(&capture::kSchemaRevision, sizeof(uint32_t), 1, g_capture);
	fwrite(&schemaLen, sizeof(uint32_t), 1, g_capture);
	fwrite(capture::kSchemaText, 1, schemaLen, g_capture);
	fflush(g_capture);

	g_captureGen++;
	g_capturePublishedGen.store(g_captureGen, std::memory_order_release);
}

// Caller must hold g_captureIoMutex AND g_captureMutex.
static void CloseCaptureLocked()
{
	if (!g_capture)
		return;
	FlushCaptureLocked();
	fclose(g_capture);
	g_capture = nullptr;
	g_capturePublishedGen.store(0, std::memory_order_release);
}

void SetCaptureEnabled(bool enabled)
{
	std::lock_guard<std::mutex> io(g_captureIoMutex);
	std::lock_guard<std::mutex> lock(g_captureMutex);
	const bool isOpen = (g_capture != nullptr);
	if (enabled == isOpen)
		return;                  // idempotent: safe to call every frame
	if (enabled)
		OpenCaptureLocked();
	else
		CloseCaptureLocked();
}

bool CaptureEnabled()
{
	return g_capturePublishedGen.load(std::memory_order_acquire) != 0;
}

uint32_t CaptureGeneration()
{
	return g_capturePublishedGen.load(std::memory_order_acquire);   // 0 = closed
}

void CaptureWrite(uint16_t type, const void *payload, uint16_t len)
{
	bool needFlush = false;
	{
		std::lock_guard<std::mutex> lock(g_captureMutex);
		if (!g_capture)
			return;              // no-op when disabled: call sites need no guard
		const capture::RecHeader h{ type, len };
		g_captureBuf.append(reinterpret_cast<const char *>(&h), sizeof h);
		if (len)
			g_captureBuf.append(static_cast<const char *>(payload), len);
		needFlush = g_captureBuf.size() >= kDiagFlushBytes;
	}
	// Disk I/O strictly outside g_captureMutex — see the mutex-split comment at the top.
	if (needFlush)
		TryFlushCapture();
}

void CaptureEvent(int64_t clockNow, const char *text)
{
	if (!text)
		return;
	size_t textLen = strlen(text);
	// Records are u16-framed; clamp rather than truncate the stream.
	if (textLen > 0xFFFF - sizeof(capture::RecEventHeader))
		textLen = 0xFFFF - sizeof(capture::RecEventHeader);

	char buf[1024];
	const size_t total = sizeof(capture::RecEventHeader) + textLen;
	if (total > sizeof buf)
		textLen = sizeof buf - sizeof(capture::RecEventHeader);

	capture::RecEventHeader hdr{ clockNow };
	memcpy(buf, &hdr, sizeof hdr);
	memcpy(buf + sizeof hdr, text, textLen);
	CaptureWrite(capture::Rec_Event, buf, (uint16_t)(sizeof hdr + textLen));
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
