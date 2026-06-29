// SPDX-License-Identifier: AGPL-3.0-only

#include "IPCServer.h"
#include "Logging.h"
#include "ServerTrackedDeviceProvider.h"

#include <algorithm>
#include <cstring>

static bool IsKnownRequestType(protocol::RequestType type)
{
	switch (type)
	{
	case protocol::RequestHandshake:
	case protocol::RequestSetDeviceTransform:
	case protocol::RequestSetHmdTracker:
	case protocol::RequestSetSlamSync:
	case protocol::RequestSetOneEuro:
	case protocol::RequestGetPredictionTelemetry:
	case protocol::RequestGetDriverTelemetry:
	case protocol::RequestApplyBatch:
		return true;
	default:
		return false;
	}
}

static void SetErrorResponse(protocol::Response& response, protocol::IpcErrorCode code)
{
	response.type = protocol::ResponseError;
	response.errorCode = code;
}

void IPCServer::HandleRequest(const protocol::Request &request, protocol::Response &response)
{
	memset(&response, 0, sizeof response);
	response.type = protocol::ResponseInvalid;

	if (!IsKnownRequestType(request.type))
	{
		LOG("Invalid IPC request type: %d", request.type);
		SetErrorResponse(response, protocol::IpcErrorInvalidRequest);
		return;
	}

	switch (request.type)
	{
	case protocol::RequestHandshake:
		response.type = protocol::ResponseHandshake;
		response.protocol.version = protocol::Version;
		break;

	case protocol::RequestSetDeviceTransform:
		if (driver->SetDeviceTransform(request.setDeviceTransform))
			response.type = protocol::ResponseSuccess;
		else
			SetErrorResponse(response, protocol::IpcErrorInvalidTransform);
		break;

	case protocol::RequestSetHmdTracker:
		if (driver->SetHmdTracker(request.setHmdTracker))
			response.type = protocol::ResponseSuccess;
		else
			SetErrorResponse(response, protocol::IpcErrorInvalidDeviceIndex);
		break;

	case protocol::RequestSetSlamSync:
		if (driver->SetSlamSync(request.setSlamSync))
			response.type = protocol::ResponseSuccess;
		else
			SetErrorResponse(response, protocol::IpcErrorInvalidDeviceIndex);
		break;

	case protocol::RequestSetOneEuro:
		if (driver->SetOneEuro(request.setOneEuro))
			response.type = protocol::ResponseSuccess;
		else
			SetErrorResponse(response, protocol::IpcErrorInvalidRequest);
		break;

	case protocol::RequestGetPredictionTelemetry:
		response.type = protocol::ResponsePredictionTelemetry;
		response.predictionTelemetry = driver->GetPredictionTelemetry();
		break;

	case protocol::RequestGetDriverTelemetry:
		response.type = protocol::ResponseDriverTelemetry;
		response.driverTelemetry = driver->GetDriverTelemetry();
		break;

	case protocol::RequestApplyBatch:
	{
		const auto& batch = request.applyBatch;
		bool truncated = false;
		bool failed = false;

		if (batch.deviceTransformCount > protocol::MaxTrackedDevices)
		{
			LOG("ApplyBatch deviceTransformCount %u exceeds max %u — truncating",
				batch.deviceTransformCount, protocol::MaxTrackedDevices);
			truncated = true;
		}
		if (batch.slamSyncCount > protocol::MaxTrackedDevices)
		{
			LOG("ApplyBatch slamSyncCount %u exceeds max %u — truncating",
				batch.slamSyncCount, protocol::MaxTrackedDevices);
			truncated = true;
		}

		const uint32_t transformCount = (std::min)(batch.deviceTransformCount, protocol::MaxTrackedDevices);
		for (uint32_t i = 0; i < transformCount; ++i)
		{
			if (!driver->SetDeviceTransform(batch.deviceTransforms[i]))
				failed = true;
		}

		const uint32_t slamCount = (std::min)(batch.slamSyncCount, protocol::MaxTrackedDevices);
		for (uint32_t i = 0; i < slamCount; ++i)
		{
			if (!driver->SetSlamSync(batch.slamSync[i]))
				failed = true;
		}

		if (batch.applyHmdTracker && !driver->SetHmdTracker(batch.hmdTracker))
			failed = true;
		if (batch.applyOneEuro && !driver->SetOneEuro(batch.oneEuro))
			failed = true;

		if (failed)
			SetErrorResponse(response, protocol::IpcErrorBatchPartialFailure);
		else if (truncated)
			SetErrorResponse(response, protocol::IpcErrorBatchTruncated);
		else
			response.type = protocol::ResponseSuccess;
		break;
	}

	default:
		LOG("Unhandled IPC request: %d", request.type);
		SetErrorResponse(response, protocol::IpcErrorInvalidRequest);
		break;
	}
}

IPCServer::~IPCServer()
{
	Stop();
}

void IPCServer::Run()
{
	mainThread = std::thread(RunThread, this);
}

void IPCServer::Stop()
{
	TRACE("IPCServer::Stop()");
	if (!running)
		return;

	stop = true;
	SetEvent(connectEvent);
	mainThread.join();
	running = false;
	TRACE("IPCServer::Stop() finished");
}

IPCServer::PipeInstance *IPCServer::CreatePipeInstance(HANDLE pipe)
{
	auto pipeInst = new PipeInstance;
	pipeInst->pipe = pipe;
	pipeInst->server = this;
	{
		std::lock_guard<std::mutex> lock(pipesMutex);
		pipes.insert(pipeInst);
	}
	return pipeInst;
}

void IPCServer::ClosePipeInstance(PipeInstance *pipeInst)
{
	DisconnectNamedPipe(pipeInst->pipe);
	CloseHandle(pipeInst->pipe);
	{
		std::lock_guard<std::mutex> lock(pipesMutex);
		pipes.erase(pipeInst);
	}
	delete pipeInst;
}

void IPCServer::RunThread(IPCServer *_this)
{
	_this->running = true;
	LPCTSTR pipeName = TEXT(OPENVR_SPACEOVERRIDE_PIPE_NAME);

	HANDLE connectEvent = _this->connectEvent = CreateEvent(0, TRUE, TRUE, 0);
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
			if (connectPending)
			{
				DWORD bytesConnect;
				BOOL success = GetOverlappedResult(nextPipe, &connectOverlap, &bytesConnect, FALSE);
				if (!success)
				{
					LOG("GetOverlappedResult failed in RunThread. Error: %d — retrying", GetLastError());
					if (nextPipe != INVALID_HANDLE_VALUE)
						CloseHandle(nextPipe);
					connectPending = CreateAndConnectInstance(&connectOverlap, nextPipe);
					continue;
				}
			}

			LOG("IPC client connected");

			auto pipeInst = _this->CreatePipeInstance(nextPipe);
			CompletedWriteCallback(0, protocol::IpcResponseSize, (LPOVERLAPPED) pipeInst);

			connectPending = CreateAndConnectInstance(&connectOverlap, nextPipe);
		}
		else if (wait != WAIT_IO_COMPLETION)
		{
			LOG("WaitForSingleObjectEx failed in RunThread. Error: %d — continuing", GetLastError());
		}
	}

	std::vector<PipeInstance *> pipeList;
	{
		std::lock_guard<std::mutex> lock(_this->pipesMutex);
		pipeList.assign(_this->pipes.begin(), _this->pipes.end());
		_this->pipes.clear();
	}
	for (auto *pipeInst : pipeList)
		_this->ClosePipeInstance(pipeInst);
}

BOOL IPCServer::CreateAndConnectInstance(LPOVERLAPPED overlap, HANDLE &pipe)
{
	constexpr DWORD kMaxPipeInstances = 8;
	constexpr int kMaxCreateAttempts = 20;
	constexpr DWORD kRetryDelayMs = 25;

	for (int attempt = 0; attempt < kMaxCreateAttempts; ++attempt)
	{
		pipe = CreateNamedPipe(
			TEXT(OPENVR_SPACEOVERRIDE_PIPE_NAME),
			PIPE_ACCESS_DUPLEX | FILE_FLAG_OVERLAPPED,
			PIPE_TYPE_MESSAGE | PIPE_READMODE_MESSAGE | PIPE_WAIT,
			kMaxPipeInstances,
			protocol::IpcRequestSize,
			protocol::IpcResponseSize,
			1000,
			0
		);

		if (pipe != INVALID_HANDLE_VALUE)
			break;

		const DWORD err = GetLastError();
		if (err == ERROR_PIPE_BUSY && attempt + 1 < kMaxCreateAttempts)
		{
			Sleep(kRetryDelayMs);
			continue;
		}

		LOG("CreateNamedPipe failed. Error: %d", err);
		return FALSE;
	}

	ConnectNamedPipe(pipe, overlap);

	switch(GetLastError())
	{
	case ERROR_IO_PENDING:
		return TRUE;

	case ERROR_PIPE_CONNECTED:
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

	if (err == 0 && bytesRead == protocol::IpcRequestSize)
	{
		pipeInst->server->HandleRequest(pipeInst->request, pipeInst->response);
		success = WriteFileEx(
			pipeInst->pipe,
			&pipeInst->response,
			protocol::IpcResponseSize,
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

	if (err == 0 && bytesWritten == protocol::IpcResponseSize)
	{
		success = ReadFileEx(
			pipeInst->pipe,
			&pipeInst->request,
			protocol::IpcRequestSize,
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