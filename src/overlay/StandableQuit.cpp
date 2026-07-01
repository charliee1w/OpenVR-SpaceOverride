// SPDX-License-Identifier: AGPL-3.0-only

#include "StandableQuit.h"

#include <chrono>
#include <thread>
#include <vector>

#ifdef _WIN32
#include <Windows.h>
#include <TlHelp32.h>

namespace
{
	static BOOL CALLBACK CloseProcessWindows(HWND hwnd, LPARAM lParam)
	{
		DWORD pid = 0;
		GetWindowThreadProcessId(hwnd, &pid);
		if (pid == static_cast<DWORD>(lParam))
			PostMessage(hwnd, WM_CLOSE, 0, 0);
		return TRUE;
	}

	static std::vector<DWORD> FindPidsByExeName(const wchar_t* exeName)
	{
		std::vector<DWORD> pids;
		HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
		if (snap == INVALID_HANDLE_VALUE)
			return pids;

		PROCESSENTRY32W pe{};
		pe.dwSize = sizeof(pe);
		if (Process32FirstW(snap, &pe))
		{
			do
			{
				if (_wcsicmp(pe.szExeFile, exeName) == 0)
					pids.push_back(pe.th32ProcessID);
			} while (Process32NextW(snap, &pe));
		}
		CloseHandle(snap);
		return pids;
	}

	static bool IsProcessAlive(DWORD pid)
	{
		HANDLE process = OpenProcess(SYNCHRONIZE, FALSE, pid);
		if (!process)
			return false;
		const DWORD wait = WaitForSingleObject(process, 0);
		CloseHandle(process);
		return wait == WAIT_TIMEOUT;
	}
}

void QuitStandableOnSteamVRExit()
{
	constexpr wchar_t kExeName[] = L"Standable.exe";
	const auto pids = FindPidsByExeName(kExeName);
	if (pids.empty())
		return;

	for (DWORD pid : pids)
		EnumWindows(CloseProcessWindows, static_cast<LPARAM>(pid));

	const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(2000);
	while (std::chrono::steady_clock::now() < deadline)
	{
		bool alive = false;
		for (DWORD pid : pids)
		{
			if (IsProcessAlive(pid))
			{
				alive = true;
				break;
			}
		}
		if (!alive)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(150));
			return;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}

	for (DWORD pid : pids)
	{
		HANDLE process = OpenProcess(PROCESS_TERMINATE, FALSE, pid);
		if (!process)
			continue;
		TerminateProcess(process, 0);
		CloseHandle(process);
	}
	std::this_thread::sleep_for(std::chrono::milliseconds(150));
}

#else

void QuitStandableOnSteamVRExit()
{
}

#endif