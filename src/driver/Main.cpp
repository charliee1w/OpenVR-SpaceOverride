// SPDX-License-Identifier: AGPL-3.0-only

#include "Main.h"
#include "ServerTrackedDeviceProvider.h"
#include "Logging.h"

#include <cstdio>
#include <cstring>
#include <openvr_driver.h>

#ifdef _WIN32
#define EXPORT_FUNC extern "C" __declspec(dllexport)
#else
#define EXPORT_FUNC extern "C"
#endif

ServerTrackedDeviceProvider g_server;

EXPORT_FUNC void* HmdDriverFactory(const char *pInterfaceName, int *pReturnCode)
{
	OpenLogFile();
	TRACE("HmdDriverFactory(%s)", pInterfaceName ? pInterfaceName : "(null)");

	if (std::strcmp(vr::IServerTrackedDeviceProvider_Version, pInterfaceName) == 0)
	{
		return &g_server;
	}

	if (pReturnCode)
	{
		*pReturnCode = vr::VRInitError_Init_InterfaceNotFound;
	}
	return nullptr;
}