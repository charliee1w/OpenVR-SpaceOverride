// SPDX-License-Identifier: AGPL-3.0-only

#include "IPCClient.h"

#include <string>
#include <stdexcept>

static std::string LastErrorString(DWORD lastError)
{
	LPSTR buffer = nullptr;
	size_t size = FormatMessageA(
		FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
		NULL, lastError, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), (LPSTR)&buffer, 0, NULL
	);

	std::string message(buffer, size);
	LocalFree(buffer);
	return message;
}

IPCClient::~IPCClient()
{
	Disconnect();
}

bool IPCClient::IsConnected() const
{
	return pipe != INVALID_HANDLE_VALUE && pipe != nullptr;
}

void IPCClient::Disconnect()
{
	if (pipe && pipe != INVALID_HANDLE_VALUE)
		CloseHandle(pipe);
	pipe = INVALID_HANDLE_VALUE;
}

void IPCClient::Connect()
{
	const int maxAttempts = 5;
	DWORD delayMs = 1000;

	for (int attempt = 1; attempt <= maxAttempts; ++attempt)
	{
		try
		{
			ConnectInternal();
			return;
		}
		catch (const std::runtime_error& e)
		{
			Disconnect();

			if (attempt >= maxAttempts)
				throw;

			fprintf(stderr, "IPC connect failed (attempt %d/%d), retrying in %lums: %s\n", attempt, maxAttempts, delayMs, e.what());
			Sleep(delayMs);
			delayMs *= 2;
		}
	}
}

void IPCClient::EnsureConnected()
{
	if (!IsConnected())
		Connect();
}

protocol::Response IPCClient::SendBlocking(const protocol::Request &request)
{
	for (int attempt = 0; attempt < 2; ++attempt)
	{
		try
		{
			EnsureConnected();
			Send(request);
			return Receive();
		}
		catch (const std::runtime_error& e)
		{
			fprintf(stderr, "IPC send failed (attempt %d/2): %s\n", attempt + 1, e.what());
			Disconnect();
			if (attempt >= 1)
				throw;
		}
	}

	throw std::runtime_error("IPC send failed after reconnect");
}

void IPCClient::Send(const protocol::Request &request)
{
	DWORD bytesWritten;
	BOOL success = WriteFile(pipe, &request, protocol::IpcRequestSize, &bytesWritten, 0);
	if (!success || bytesWritten != protocol::IpcRequestSize)
	{
		throw std::runtime_error("Error writing IPC request. Error: " + LastErrorString(GetLastError()));
	}
}

protocol::Response IPCClient::Receive()
{
	protocol::Response response(protocol::ResponseInvalid);
	DWORD bytesRead;

	BOOL success = ReadFile(pipe, &response, protocol::IpcResponseSize, &bytesRead, 0);
	if (!success)
	{
		DWORD lastError = GetLastError();
		if (lastError != ERROR_MORE_DATA)
		{
			throw std::runtime_error("Error reading IPC response. Error: " + LastErrorString(lastError));
		}
	}

	if (bytesRead != protocol::IpcResponseSize)
	{
		throw std::runtime_error("Invalid IPC response with size " + std::to_string(bytesRead));
	}

	return response;
}

void IPCClient::ConnectInternal()
{
	LPCTSTR pipeName = TEXT(OPENVR_SPACEOVERRIDE_PIPE_NAME);
	WaitNamedPipe(pipeName, 3000);
	pipe = CreateFile(pipeName, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);
	if (pipe == INVALID_HANDLE_VALUE)
	{
		throw std::runtime_error("Space Override driver unavailable. Ensure SteamVR is running and the spaceoverride driver is enabled.");
	}

	DWORD mode = PIPE_READMODE_MESSAGE;
	if (!SetNamedPipeHandleState(pipe, &mode, 0, 0))
	{
		throw std::runtime_error("Couldn't set pipe mode. Error: " + LastErrorString(GetLastError()));
	}

	Send(protocol::Request(protocol::RequestHandshake));
	auto response = Receive();

	if (response.type != protocol::ResponseHandshake || response.protocol.version != protocol::Version)
	{
		throw std::runtime_error(
			"Incorrect driver version installed, try reinstalling OpenVR-SpaceOverride. (Client: " +
			std::to_string(protocol::Version) + ", Driver: " +
			std::to_string(response.protocol.version) + ")"
		);
	}
}