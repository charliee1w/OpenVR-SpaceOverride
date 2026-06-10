#include "OpenVR-SpaceCalibratorDriver.h"
#include "ServerTrackedDeviceProvider.h"
#include "Logging.h"

#include <cstdio>
#include <cstring>
#include <openvr_driver.h>

OPENVRSPACECALIBRATORDRIVER_API void *HmdDriverFactory(const char *pInterfaceName, int *pReturnCode)
{
	TRACE("HmdDriverFactory(%s)", pInterfaceName);

	static ServerTrackedDeviceProvider server;

	if (std::strcmp(vr::IServerTrackedDeviceProvider_Version, pInterfaceName) == 0)
	{
		return &server;
	}

	if (pReturnCode)
	{
		*pReturnCode = vr::VRInitError_Init_InterfaceNotFound;
	}
	return nullptr;
}