// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include "Calibration.h"

void LoadProfile(CalibrationContext &ctx);
void SaveProfile(CalibrationContext &ctx);
// Clears the context and deletes the stored profile. SaveProfile cannot express deletion,
// because it refuses to persist an invalid profile by design.
void RemoveProfile(CalibrationContext &ctx);
