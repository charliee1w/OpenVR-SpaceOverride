// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include "Protocol.h"

namespace filter_defaults
{
	constexpr protocol::OneEuroParams Head{ 5.0, 0.8, 1.0 };
	constexpr protocol::OneEuroParams Drift{ 3.0, 1.3, 0.6 };
	// Stronger smoothing for jitter-prone head mounts (e.g. Tundra trackers).
	constexpr protocol::OneEuroParams TundraHead{ 2.5, 0.4, 0.8 };
	constexpr protocol::OneEuroParams TundraDrift{ 2.0, 0.8, 0.5 };
}