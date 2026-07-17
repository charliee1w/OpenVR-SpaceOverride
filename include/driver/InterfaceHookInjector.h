// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include <openvr_driver.h>

void InjectHooks(vr::IVRDriverContext *pDriverContext);
void DisableHooks();

// Pose hooks pass through unmutated while the driver is tearing down.
void SetDriverShuttingDown(bool shuttingDown);
