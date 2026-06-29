// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include <Windows.h>

#include "Protocol.h"

class IPCClient
{
public:
	~IPCClient();

	void Connect();
	bool IsConnected() const;
	void Disconnect();
	void EnsureConnected();

	protocol::Response SendBlocking(const protocol::Request &request);

	void Send(const protocol::Request &request);
	protocol::Response Receive();

private:
	void ConnectInternal();

	HANDLE pipe = INVALID_HANDLE_VALUE;
};