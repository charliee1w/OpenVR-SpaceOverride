// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#define SPACECAL_VERSION_STRING "8"

// Which build of this codebase you are looking at. The overlay footer and the driver's
// first session-log line both carry it, so a running stack can be identified from inside
// the headset and from the log without cross-checking a hash against a deploy manifest.
//
// This matters here specifically: several builds of this driver exist (pristine upstream,
// this one, the fusion fork), they are swapped in and out of the same SteamVR install for
// A/B comparison, and they share a protocol version -- so a mismatched or misremembered
// pair connects and runs rather than refusing. An A/B whose arms cannot be told apart is
// not an A/B; a previous comparison was invalidated by exactly that kind of unrecorded
// difference.
//
// "so-lts" = upstream 6604e42 plus math-neutral quality-of-life guards, no fusion.
#define SPACECAL_BUILD_VARIANT "so-lts"