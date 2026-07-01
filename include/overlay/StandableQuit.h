// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

// Gracefully close Standable.exe before SteamVR teardown to avoid vrserver
// shutdown crashes from standable driver PoseUpdated spam.
void QuitStandableOnSteamVRExit();