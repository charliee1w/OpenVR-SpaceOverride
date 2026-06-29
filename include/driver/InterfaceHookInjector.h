// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include <openvr_driver.h>

static void DetourTrackedDevicePoseUpdated(void* _this, uint32_t unWhichDevice, const vr::DriverPose_t & newPose, uint32_t unPoseStructSize);

bool InjectHooks(vr::IVRDriverContext *pDriverContext);
void DisableHooks();
bool PoseHooksInstalled();
void SetDriverShuttingDown(bool shuttingDown);
bool IsDriverShuttingDown();