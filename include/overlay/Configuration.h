// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include "Calibration.h"

void LoadProfile(CalibrationContext &ctx);
void SaveProfile(CalibrationContext &ctx);
bool ExportProfileToFile(const CalibrationContext &ctx, const std::string &path);
bool ImportProfileFromFile(CalibrationContext &ctx, const std::string &path);
