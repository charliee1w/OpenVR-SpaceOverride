// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include "Protocol.h"

#include <atomic>
#include <thread>
#include <set>
#include <mutex>

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

class ServerTrackedDeviceProvider;

class IPCServer
{
public:
	IPCServer(ServerTrackedDeviceProvider *driver) : driver(driver) { }
	~IPCServer();

	void Run();
	void Stop();

private:
	void HandleRequest(const protocol::Request &request, protocol::Response &response);

	struct PipeInstance
	{
		OVERLAPPED overlap; // Used by the API
		HANDLE pipe;
		IPCServer *server;

		protocol::Request request;
		protocol::Response response;
	};

	PipeInstance *CreatePipeInstance(HANDLE pipe);
	void ClosePipeInstance(PipeInstance *pipeInst);

	static void RunThread(IPCServer *_this);
	static BOOL CreateAndConnectInstance(LPOVERLAPPED overlap, HANDLE &pipe);
	static void WINAPI CompletedReadCallback(DWORD err, DWORD bytesRead, LPOVERLAPPED overlap);
	static void WINAPI CompletedWriteCallback(DWORD err, DWORD bytesWritten, LPOVERLAPPED overlap);

	std::thread mainThread;

	// Written by the server thread, read by the caller of Stop(): plain bools here were a data
	// race, and `running` in particular gated the join. Stop() could observe it still false
	// (the thread had not reached its first statement), return without joining, and leave a
	// joinable std::thread to be destroyed — std::terminate(), inside vrserver.
	std::atomic<bool> running{ false };
	std::atomic<bool> stop{ false };

	std::set<PipeInstance *> pipes;
	// Same cross-thread pattern as running/stop, and the wake Stop() depends on: a stale read
	// here skips the SetEvent and leaves join() waiting on a thread parked in an INFINITE wait.
	std::atomic<HANDLE> connectEvent{ nullptr };

	ServerTrackedDeviceProvider *driver;
};
