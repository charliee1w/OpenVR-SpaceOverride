// SPDX-License-Identifier: AGPL-3.0-only

#include "IPCServer.h"
#include "Logging.h"
#include "ServerTrackedDeviceProvider.h"

void IPCServer::HandleRequest(const protocol::Request &request, protocol::Response &response)
{
	switch (request.type)
	{
	case protocol::RequestHandshake:
		response.type = protocol::ResponseHandshake;
		response.protocol.version = protocol::Version;
		break;

	case protocol::RequestSetDeviceTransform:
		driver->SetDeviceTransform(request.setDeviceTransform);
		response.type = protocol::ResponseSuccess;
		break;

	case protocol::RequestSetHmdTracker:
		driver->SetHmdTracker(request.setHmdTracker);
		response.type = protocol::ResponseSuccess;
		break;

	case protocol::RequestSetSlamSync:
		driver->SetSlamSync(request.setSlamSync);
		response.type = protocol::ResponseSuccess;
		break;

	case protocol::RequestSetOneEuro:
		driver->SetOneEuro(request.setOneEuro);
		response.type = protocol::ResponseSuccess;
		break;

	default:
		LOG("Invalid IPC request: %d", request.type);
		response.type = protocol::ResponseInvalid;
		break;
	}
}

IPCServer::~IPCServer()
{
	Stop();
}

void IPCServer::Run()
{
	// Clear the latch before starting. Stop() sets `stop` and nothing else ever cleared it, so a
	// same-process Cleanup->Init -- which this driver explicitly supports, and where member
	// initialisers do NOT re-run -- restarted the thread with stop already true. RunThread's
	// `while (!stop)` then exited on its first iteration, the pipe drained, and the overlay could
	// never reconnect for the life of the process, with no error anywhere. InjectHooks was given
	// the matching g_driverShuttingDown.store(false) reset for exactly this reason.
	stop = false;
	running = false;
	mainThread = std::thread(RunThread, this);
}

void IPCServer::Stop()
{
	TRACE("IPCServer::Stop()");

	// Join on joinable(), never on `running`. `running` is set by the thread itself, so testing
	// it here raced the thread's own startup: a fast Init->Cleanup would see false, skip the
	// join, and destroy a joinable std::thread -> std::terminate(). joinable() is a property of
	// the handle this side owns, so it cannot race that way.
	if (!mainThread.joinable())
		return;

	stop = true;
	if (HANDLE ev = connectEvent.load())
		SetEvent(ev);
	mainThread.join();
	running = false;
	TRACE("IPCServer::Stop() finished");
}

IPCServer::PipeInstance *IPCServer::CreatePipeInstance(HANDLE pipe)
{
	// Value-initialised: OVERLAPPED is handed straight to ReadFileEx/WriteFileEx, and `request`
	// is the buffer a short read would leave partly stale. `new PipeInstance` (no braces) left
	// both indeterminate.
	auto pipeInst = new PipeInstance{};
	pipeInst->pipe = pipe;
	pipeInst->server = this;
	pipes.insert(pipeInst);
	return pipeInst;
}

void IPCServer::ClosePipeInstance(PipeInstance *pipeInst)
{
	DisconnectNamedPipe(pipeInst->pipe);
	CloseHandle(pipeInst->pipe);
	pipes.erase(pipeInst);
	delete pipeInst;
}

void IPCServer::RunThread(IPCServer *_this)
{
	_this->running = true;
	LPCTSTR pipeName = TEXT(OPENVR_SPACECALIBRATOR_PIPE_NAME);

	HANDLE connectEvent = CreateEvent(0, TRUE, TRUE, 0);
	_this->connectEvent = connectEvent;
	if (!connectEvent)
	{
		LOG("CreateEvent failed in RunThread. Error: %d", GetLastError());
		return;
	}

	OVERLAPPED connectOverlap;
	connectOverlap.hEvent = connectEvent;

	HANDLE nextPipe;
	BOOL connectPending = CreateAndConnectInstance(&connectOverlap, nextPipe);

	while (!_this->stop)
	{
		DWORD wait = WaitForSingleObjectEx(connectEvent, INFINITE, TRUE);

		if (_this->stop)
		{
			break;
		}
		else if (wait == 0)
		{
			// When connectPending is false, the last call to CreateAndConnectInstance
			// picked up a connected client and triggered this event, so we can simply
			// create a new pipe instance for it. If true, the client was still pending
			// connection when CreateAndConnectInstance returned, so this event was triggered
			// internally and we need to flush out the result, or something like that.
			if (connectPending)
			{
				DWORD bytesConnect;
				BOOL success = GetOverlappedResult(nextPipe, &connectOverlap, &bytesConnect, FALSE);
				if (!success)
				{
					LOG("GetOverlappedResult failed in RunThread. Error: %d", GetLastError());
					return;
				}
			}

			LOG("IPC client connected");

			auto pipeInst = _this->CreatePipeInstance(nextPipe);
			CompletedWriteCallback(0, sizeof protocol::Response, (LPOVERLAPPED) pipeInst);

			connectPending = CreateAndConnectInstance(&connectOverlap, nextPipe);
		}
		else if (wait != WAIT_IO_COMPLETION)
		{
			printf("WaitForSingleObjectEx failed in RunThread. Error %d", GetLastError());
			return;
		}
	}

	// ClosePipeInstance erases from `pipes`, which invalidates the iterator the
	// range-for is holding; incrementing it afterwards walked a freed node. Drain
	// the set instead so no iterator outlives the element it points at.
	while (!_this->pipes.empty())
	{
		_this->ClosePipeInstance(*_this->pipes.begin());
	}
}

BOOL IPCServer::CreateAndConnectInstance(LPOVERLAPPED overlap, HANDLE &pipe)
{
	pipe = CreateNamedPipe(
		TEXT(OPENVR_SPACECALIBRATOR_PIPE_NAME),
		PIPE_ACCESS_DUPLEX | FILE_FLAG_OVERLAPPED,
		PIPE_TYPE_MESSAGE | PIPE_READMODE_MESSAGE | PIPE_WAIT,
		PIPE_UNLIMITED_INSTANCES,
		sizeof protocol::Request,
		sizeof protocol::Response,
		1000,
		0
	);

	if (pipe == INVALID_HANDLE_VALUE)
	{
		LOG("CreateNamedPipe failed. Error: %d", GetLastError());
		return FALSE;
	}

	ConnectNamedPipe(pipe, overlap);

	switch(GetLastError())
	{
	case ERROR_IO_PENDING:
		// Mark a pending connection by returning true, and when the connection
		// completes an event will trigger automatically.
		return TRUE;

	case ERROR_PIPE_CONNECTED:
		// Signal the event loop that a client is connected.
		if (SetEvent(overlap->hEvent))
			return FALSE;
	}

	LOG("ConnectNamedPipe failed. Error: %d", GetLastError());
	return FALSE;
}

void IPCServer::CompletedReadCallback(DWORD err, DWORD bytesRead, LPOVERLAPPED overlap)
{
	PipeInstance *pipeInst = (PipeInstance *) overlap;
	BOOL success = FALSE;

	// Length-checked, matching what CompletedWriteCallback already does for the response. A
	// short message used to be dispatched anyway, so `request` was read past what the client
	// actually wrote — stale bytes from the previous request on this pipe instance, or (before
	// the value-init above) uninitialised heap.
	if (err == 0 && bytesRead == sizeof protocol::Request)
	{
		pipeInst->server->HandleRequest(pipeInst->request, pipeInst->response);
		success = WriteFileEx(
			pipeInst->pipe,
			&pipeInst->response,
			sizeof protocol::Response,
			overlap,
			(LPOVERLAPPED_COMPLETION_ROUTINE) CompletedWriteCallback
		);
	}

	if (!success)
	{
		if (err == ERROR_BROKEN_PIPE)
		{
			LOG("IPC client disconnecting normally");
		}
		else if (err == 0 && bytesRead != sizeof protocol::Request)
		{
			LOG("IPC client sent a short/oversized request (%u bytes, expected %u) - dropping connection",
				(unsigned)bytesRead, (unsigned)sizeof protocol::Request);
		}
		else
		{
			LOG("IPC client disconnecting due to error (via CompletedReadCallback), error: %d, bytesRead: %d", err, bytesRead);
		}
		pipeInst->server->ClosePipeInstance(pipeInst);
	}
}

void IPCServer::CompletedWriteCallback(DWORD err, DWORD bytesWritten, LPOVERLAPPED overlap)
{
	PipeInstance *pipeInst = (PipeInstance *) overlap;
	BOOL success = FALSE;

	if (err == 0 && bytesWritten == sizeof protocol::Response)
	{
		success = ReadFileEx(
			pipeInst->pipe,
			&pipeInst->request,
			sizeof protocol::Request,
			overlap,
			(LPOVERLAPPED_COMPLETION_ROUTINE) CompletedReadCallback
		);
	}

	if (!success)
	{
		LOG("IPC client disconnecting due to error (via CompletedWriteCallback), error: %d, bytesWritten: %d", err, bytesWritten);
		pipeInst->server->ClosePipeInstance(pipeInst);
	}
}
