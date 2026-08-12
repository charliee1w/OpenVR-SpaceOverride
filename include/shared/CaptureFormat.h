// SPDX-License-Identifier: AGPL-3.0-only
#pragma once

#include <cstdint>
#include <cstddef>

// =============================================================================================
// Unified capture format — the ONE machine-readable sink.
//
// WHY THIS EXISTS
// ---------------
// Diagnostics used to be a per-question printf: want a new number, edit a format string, edit an
// arg list, edit a CSV header, rebuild, deploy, play a session. Every question cost a release
// cycle, and the sinks multiplied (rolling log, session log, fusion_diag, body_diag) until they
// had separate open/close lifecycles — which is exactly how body_diag shipped broken (its open
// was wired to one of OpenDiagCsv's two call sites).
//
// The fix is not "one more CSV". It is to record the ESTIMATOR'S INPUTS and derive everything
// else offline:
//
//   * A new derived metric costs ZERO driver work. disp / corr / sig / Kt / NIS / gate outcome
//     are all outputs of pose_est::ProcessHmdFrame. Replay the inputs and compute whatever you
//     want, at any rate, with whatever definition — no rebuild, no deploy, no play session.
//   * It structurally kills the N0-d class of defect. `disp_cm` mixed pre-fit innovation (gated
//     rows) with post-fit residual (accepted rows) in one column. You cannot conflate two
//     quantities in a column you never write; offline you compute both and name them separately.
//   * No sampling throttle. The old CSVs were throttled to ~20 Hz because writing derived rows
//     on the pose thread was expensive. Inputs are written once per frame and derived later, so
//     there are no aliasing artifacts to reason about.
//
// The human-readable session log STAYS. It is a different job: events a person greps while
// debugging (SLAM_STEP, tracker BAD, HMD_JUMP_HOLD, heartbeat). Two sinks, not five.
//
// FORWARD COMPATIBILITY — the property that keeps this from needing dev work
// -------------------------------------------------------------------------
// Every record is {type, len, payload}. A reader dispatches on `type` and SKIPS `len` bytes for
// anything it does not recognise. That gives, without any reader changes:
//
//   * New record types are ignorable by old readers.
//   * Appending fields to the END of an existing record is readable by old readers (they parse
//     the prefix they know and skip the rest, because `len` is authoritative — never sizeof()).
//
// Rules for changing this file:
//   1. APPEND fields to the end of a record. Never insert, reorder, or resize an existing field.
//   2. Adding a field: bump kSchemaRevision. Do NOT bump kFormatVersion.
//   3. Bump kFormatVersion ONLY for a genuinely incompatible change (reordering, removing, or
//      changing the framing). That tells old readers to refuse the file rather than misread it.
//   4. Readers MUST use the record's `len`, never sizeof(), to advance.
//
// This header is the single source of truth and is shared by the driver and the offline runner,
// so the two cannot drift. kSchemaText below is a human/forensic description embedded in every
// capture; keep it in step when you append a field.
// =============================================================================================

namespace capture {

// "SORCAP01" — magic identifies the stream even if the file is renamed or truncated.
constexpr char kMagic[8] = { 'S', 'O', 'R', 'C', 'A', 'P', '0', '1' };

// Incompatible framing/layout changes only. See rule 3 above.
constexpr uint32_t kFormatVersion = 1;

// Bumped when a field is APPENDED. Readers may use it to know whether a trailing field exists,
// but should prefer checking the record's `len`.
//
// rev2 (2026-08-06): appended `enabled` + `trackerID` to Rec_Config; added Rec_DeviceInfo. Both
// are backward-compatible — a rev1 reader parses the config prefix it knows and skips record
// type 5 by `len`. Driven by what the first real capture could NOT answer; see N2-d.
constexpr uint32_t kSchemaRevision = 2;

enum RecordType : uint16_t
{
	Rec_Config     = 1,   // estimator config; written on change, not per frame
	Rec_Frame      = 2,   // one per HMD frame: everything ProcessHmdFrame reads
	Rec_Device     = 3,   // one per tracked device per tick, lighthouse world
	Rec_Event      = 4,   // text mirror of a session-log event, for time correlation
	Rec_DeviceInfo = 5,   // id -> serial/model/class, once per device per capture file
};

#pragma pack(push, 1)

// Record framing. `len` counts PAYLOAD bytes only, excluding this header.
struct RecHeader
{
	uint16_t type;
	uint16_t len;
};

struct CapQuat { double w, x, y, z; };
struct CapVec3 { double x, y, z; };

// --- Rec_Config -------------------------------------------------------------------------------
// Mirrors PoseConfig. Written whenever any field changes; `seq` is referenced by Rec_Frame so a
// replay knows which config was live for a given frame without repeating it 90 times a second.
struct RecConfigPayload
{
	uint32_t seq;
	uint8_t  fusionMode;
	uint8_t  native;
	uint8_t  slamFallback;
	uint8_t  enableAngularVelocity;
	uint8_t  headFilterEnabled;
	uint8_t  _pad[3];
	float    predictionTime;
	float    _pad2;
	CapQuat  offsetRotation;
	CapVec3  offsetTranslation;
	CapQuat  calibrationRotation;
	CapVec3  calibrationTranslation;
	double   calibrationScale;
	double   hmdScale;

	// --- appended in rev2 ---
	// `enabled` is 1 BY CONSTRUCTION: this record is only ever emitted from the pose path, which
	// runs inside `if (hmdTracker.enabled)`. It is here so a replay reads the flag instead of
	// assuming it. The disable TRANSITION is not visible here and never will be — it is carried
	// by the Rec_Event mirror of the `SetHmdTracker enabled=0` log line. A capture whose frames
	// stop for 32 s is explained by that event, not by this field.
	uint8_t  enabled;
	uint8_t  _pad3[3];
	uint32_t trackerID;         // vr::k_unTrackedDeviceIndexInvalid when none
};

// --- Rec_Frame --------------------------------------------------------------------------------
// Everything ProcessHmdFrame reads, plus the INCOMING pose it will overwrite. Replay reconstructs
// the call exactly: ProcessHmdFrame(cfg[configSeq], clock, displayHz, hmd, tracker, pose, ...).
struct RecFramePayload
{
	uint32_t configSeq;         // which Rec_Config was live
	// Incremented WHEN A RECORD IS WRITTEN, not when the hook fires. Contiguity therefore proves
	// FILE INTEGRITY (nothing was lost between writer and disk) and NOT sampling completeness —
	// it cannot detect a pose callback that never reached the writer. To reason about the sample
	// rate, use clockNow deltas. (First capture: contiguous 0..84016, yet the mean rate was
	// 32.6 Hz against a displayHz of 80.)
	uint32_t frameIndex;
	int64_t  clockNow;          // PoseClock.now (ticks)
	int64_t  clockFreq;         // PoseClock.freq (ticks/sec)
	double   displayHz;

	// HmdInput
	uint8_t  hmdRawValid;
	uint8_t  _pad[7];
	CapQuat  hmdRotation;
	CapVec3  hmdPosition;

	// TrackerInput
	uint8_t  trkPoseOk;
	uint8_t  trkOkForOverride;
	uint8_t  trkSpeedReject;
	uint8_t  _pad2[5];
	int32_t  trkResult;         // vr::ETrackingResult
	int32_t  _pad3;
	CapQuat  trkRotation;
	CapVec3  trkPosition;
	CapVec3  trkVelocity;
	CapVec3  trkAngularVelocity;
	double   trkLinSpeed;
	double   trkAgeSec;

	// Incoming DriverPose_t fields ProcessHmdFrame reads or overwrites. Captured BEFORE the call.
	uint8_t  inPoseIsValid;
	uint8_t  inDeviceIsConnected;
	uint8_t  _pad4[6];
	int32_t  inResult;
	int32_t  _pad5;
	CapQuat  inRotation;
	CapVec3  inPosition;
	CapQuat  inWorldFromDriverRotation;
	CapVec3  inWorldFromDriverTranslation;
	CapQuat  inDriverFromHeadRotation;
	CapVec3  inDriverFromHeadTranslation;
	CapVec3  inVelocity;
	CapVec3  inAngularVelocity;
};

// --- Rec_Device -------------------------------------------------------------------------------
// Any tracked device that is not the HMD, in LIGHTHOUSE world space — so head and body share a
// frame and head-vs-body drift is a subtraction, not an inference. Generic: a new device class
// needs no new code, it just appears with its own id and role.
//
//   Role_Head — rigid head tracker, lighthouse-native (never SO-corrected).
//   Role_Sync — slamSync device, captured AFTER ApplySharedDrift so the row is lighthouse-world
//               (the published pose). Pre-drift capture was SLAM-space and poisoned B6 math.
//   Role_LH   — lighthouse-native, captured BEFORE the overlay transforms[] block so calibration
//               rot/trans/scale are not folded into a "drift" residual.
enum DeviceRole : uint8_t
{
	Role_Head = 0,   // the rigid head-mounted tracker
	Role_Sync = 1,   // slamSync: drift already applied in the captured pose
	Role_LH   = 2,   // lighthouse-native, pre-transform
};

struct RecDevicePayload
{
	int64_t  clockNow;
	uint32_t deviceId;
	uint8_t  role;
	uint8_t  poseIsValid;
	uint8_t  _pad[2];
	int32_t  result;
	int32_t  _pad2;
	CapQuat  rotation;
	CapVec3  position;
};

// --- Rec_DeviceInfo ---------------------------------------------------------------------------
// Names a device. Rec_Device carries only `deviceId`, which is an OpenVR ENUMERATION INDEX: it is
// assigned in connection order and is NOT stable across SteamVR restarts, so "id 9" means nothing
// across two captures and nothing at all to a person. Without this record the first real capture
// held 16 unnameable devices and could not distinguish a base station from a foot puck — which is
// exactly what B6 and the body-drift question need.
//
// Emitted once per device per CAPTURE FILE (not per driver session): the file must be
// self-describing on its own, so re-opening the sink re-emits for every device still present.
// Always precedes that device's first Rec_Device row in the same file.
struct RecDeviceInfoPayload
{
	int64_t  clockNow;
	uint32_t deviceId;
	int32_t  deviceClass;       // vr::ETrackedDeviceClass (2 = Controller, 3 = GenericTracker,
	                            // 4 = TrackingReference/base station). 0 if the read failed.
	uint8_t  role;              // DeviceRole, same meaning as Rec_Device
	uint8_t  _pad[7];
	char     serial[64];        // Prop_SerialNumber_String, NUL-padded; empty if unreadable
	char     model[64];         // Prop_ModelNumber_String,  NUL-padded; empty if unreadable
};

// --- Rec_Event --------------------------------------------------------------------------------
// Text mirror of a session-log line so a capture is self-contained for correlation. Variable
// length: `len` is 8 + strlen(text); text is NOT null-terminated.
struct RecEventHeader
{
	int64_t clockNow;
	// char text[] follows
};

#pragma pack(pop)

// Layout guards. If you append a field the assert fires — update the size AND kSchemaText, which
// is the point: the schema cannot silently drift from the structs.
static_assert(sizeof(RecHeader) == 4, "RecHeader framing changed — bump kFormatVersion");
static_assert(sizeof(RecConfigPayload) == 156, "RecConfigPayload changed — bump kSchemaRevision and update kSchemaText");
static_assert(sizeof(RecFramePayload) == 464, "RecFramePayload changed — bump kSchemaRevision and update kSchemaText");
static_assert(sizeof(RecDevicePayload) == 80, "RecDevicePayload changed — bump kSchemaRevision and update kSchemaText");
static_assert(sizeof(RecDeviceInfoPayload) == 152, "RecDeviceInfoPayload changed — bump kSchemaRevision and update kSchemaText");

// Embedded in every capture's header so a file is interpretable years later without this repo.
constexpr const char* kSchemaText =
	"SORCAP v1 rev2 — OpenVR-SpaceOverride estimator input capture\n"
	"Framing: [u16 type][u16 len][payload len bytes]. Skip unknown types by len. All LE, packed.\n"
	"1 Config(156): seq u32; fusionMode,native,slamFallback,enableAngularVelocity,headFilterEnabled u8;\n"
	"   pad3; predictionTime f32; pad4; offsetRot q4; offsetTrans v3; calRot q4; calTrans v3;\n"
	"   calibrationScale f64; hmdScale f64; [rev2] enabled u8; pad3; trackerID u32\n"
	"2 Frame(464): configSeq u32; frameIndex u32; clockNow i64; clockFreq i64; displayHz f64;\n"
	"   hmdRawValid u8; pad7; hmdRot q4; hmdPos v3;\n"
	"   trkPoseOk,trkOkForOverride,trkSpeedReject u8; pad5; trkResult i32; pad4;\n"
	"   trkRot q4; trkPos v3; trkVel v3; trkAngVel v3; trkLinSpeed f64; trkAgeSec f64;\n"
	"   inPoseIsValid,inDeviceIsConnected u8; pad6; inResult i32; pad4; inRot q4; inPos v3;\n"
	"   inWorldFromDriverRot q4; inWorldFromDriverTrans v3; inDriverFromHeadRot q4;\n"
	"   inDriverFromHeadTrans v3; inVel v3; inAngVel v3\n"
	"3 Device(80): clockNow i64; deviceId u32; role u8 (0=head,1=sync,2=lh); poseIsValid u8; pad2;\n"
	"   result i32; pad4; rot q4; pos v3\n"
	"4 Event(8+n): clockNow i64; text (n bytes, not terminated)\n"
	"5 [rev2] DeviceInfo(152): clockNow i64; deviceId u32; deviceClass i32 (2=Controller,\n"
	"   3=GenericTracker, 4=TrackingReference); role u8; pad7; serial char[64]; model char[64].\n"
	"   Once per device per FILE, before that device's first Device row. deviceId alone is an\n"
	"   enumeration index and is NOT stable across SteamVR restarts — join on serial, not id.\n"
	"q4 = {w,x,y,z} f64. v3 = {x,y,z} f64.\n"
	"Poses in Device are LIGHTHOUSE WORLD SPACE (Role_Head raw tracker; Role_Sync post-drift;\n"
	"Role_LH pre-transform). Frame holds estimator INPUTS;\n"
	"all derived metrics (disp/corr/sig/Kt/gate) are computed by replaying, never stored.\n"
	"Frame.frameIndex counts RECORDS WRITTEN, not hook calls: contiguity proves file integrity,\n"
	"not sampling completeness. Use clockNow deltas for rate. Every session-log line is mirrored\n"
	"as an Event, so gaps in Frame (e.g. the tracker being disabled) are explained in-file.\n";

} // namespace capture
