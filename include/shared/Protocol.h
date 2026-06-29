// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include <cstddef>
#include <cstdint>

#ifndef _OPENVR_API
#include <openvr_driver.h>
#endif

#define OPENVR_SPACEOVERRIDE_PIPE_NAME "\\\\.\\pipe\\OpenVRSpaceOverrideCom"

namespace protocol
{
	constexpr uint32_t Version = 9;
	constexpr uint32_t MaxTrackedDevices = 64;

	enum RequestType
	{
		RequestInvalid,
		RequestHandshake,
		RequestSetDeviceTransform,
		RequestSetHmdTracker,
		RequestSetSlamSync,
		RequestSetOneEuro,
		RequestGetPredictionTelemetry,
		RequestGetDriverTelemetry,
		RequestApplyBatch,
	};

	enum ResponseType
	{
		ResponseInvalid,
		ResponseHandshake,
		ResponseSuccess,
		ResponseError,
		ResponsePredictionTelemetry,
		ResponseDriverTelemetry,
	};

	enum IpcErrorCode : uint32_t
	{
		IpcErrorNone = 0,
		IpcErrorInvalidRequest,
		IpcErrorInvalidDeviceIndex,
		IpcErrorInvalidTransform,
		IpcErrorBatchTruncated,
		IpcErrorBatchPartialFailure,
	};

	enum DriverOverrideInactiveReason : uint8_t
	{
		DriverOverrideActive = 0,
		DriverOverrideDisabled,
		DriverOverrideHooksMissing,
		DriverOverrideTrackerInvalid,
		DriverOverrideTrackerLost,
	};

	struct Protocol
	{
		uint32_t version = Version;
	};

	struct SetDeviceTransform
	{
		uint32_t openVRID;
		bool enabled;
		bool updateTranslation;
		bool updateRotation;
		bool updateScale;
		vr::HmdVector3d_t translation;
		vr::HmdQuaternion_t rotation;
		double scale;

		SetDeviceTransform() :
			openVRID(0), enabled(false), updateTranslation(false), updateRotation(false), updateScale(false),
			translation{}, rotation{ 1, 0, 0, 0 }, scale(1.0) {}

		SetDeviceTransform(uint32_t id, bool enabled) :
			openVRID(id), enabled(enabled), updateTranslation(false), updateRotation(false), updateScale(false) { }

		SetDeviceTransform(uint32_t id, bool enabled, vr::HmdVector3d_t translation) :
			openVRID(id), enabled(enabled), updateTranslation(true), updateRotation(false), updateScale(false), translation(translation) { }

		SetDeviceTransform(uint32_t id, bool enabled, vr::HmdQuaternion_t rotation) :
			openVRID(id), enabled(enabled), updateTranslation(false), updateRotation(true), updateScale(false), rotation(rotation) { }

		SetDeviceTransform(uint32_t id, bool enabled, double scale) :
			openVRID(id), enabled(enabled), updateTranslation(false), updateRotation(false), updateScale(true), scale(scale) { }

		SetDeviceTransform(uint32_t id, bool enabled, vr::HmdVector3d_t translation, vr::HmdQuaternion_t rotation) :
			openVRID(id), enabled(enabled), updateTranslation(true), updateRotation(true), updateScale(false), translation(translation), rotation(rotation) { }

		SetDeviceTransform(uint32_t id, bool enabled, vr::HmdVector3d_t translation, vr::HmdQuaternion_t rotation, double scale) :
			openVRID(id), enabled(enabled), updateTranslation(true), updateRotation(true), updateScale(true), translation(translation), rotation(rotation), scale(scale) { }
	};

	struct SetHmdTracker
	{
		uint32_t hmdID;
		uint32_t trackerID;
		bool enabled;
		bool native;
		bool slamFallback;
		bool enableAngularVelocity;
		float predictionTime;
		bool predictionAuto;
		vr::HmdQuaternion_t offsetRotation;
		vr::HmdVector3d_t offsetTranslation;
		vr::HmdQuaternion_t calibrationRotation;
		vr::HmdVector3d_t calibrationTranslation;
	};

	struct SetSlamSync
	{
		uint32_t openVRID;
		bool enabled;
	};

	struct OneEuroParams
	{
		double minCutoff;
		double beta;
		double dCutoff;
	};

	struct SetOneEuro
	{
		bool headEnabled;
		OneEuroParams head;
		OneEuroParams drift;
	};

	struct PredictionTelemetry
	{
		bool valid;
		float estimatedLagMs;
		float estimatedLagFrames;
		float appliedPredictionFrames;
		float displayHz;
	};

	struct DriverTelemetry
	{
		bool valid = false;
		bool overrideEnabled = false;
		bool overrideActive = false;
		DriverOverrideInactiveReason overrideInactiveReason = DriverOverrideDisabled;
		bool poseHooksInstalled = false;
		bool trackerValid = false;
		float trackerLostSeconds = 0.0f;
		float driftYawDeg = 0.0f;
		float driftTranslationMm = 0.0f;
		float appliedPredictionFrames = 0.0f;
		float displayHz = 90.0f;
		bool driftValid = false;
		bool trackerBlendActive = false;
	};

	struct ApplyBatch
	{
		uint32_t deviceTransformCount = 0;
		SetDeviceTransform deviceTransforms[MaxTrackedDevices]{};

		uint32_t slamSyncCount = 0;
		SetSlamSync slamSync[MaxTrackedDevices]{};

		bool applyHmdTracker = false;
		SetHmdTracker hmdTracker{};

		bool applyOneEuro = false;
		SetOneEuro oneEuro{};
	};

	struct Request
	{
		RequestType type;

		union {
			SetDeviceTransform setDeviceTransform;
			SetHmdTracker setHmdTracker;
			SetSlamSync setSlamSync;
			SetOneEuro setOneEuro;
			ApplyBatch applyBatch;
		};

		Request() : type(RequestInvalid) { }
		Request(RequestType type) : type(type) { }
	};

	struct Response
	{
		ResponseType type;

		union {
			Protocol protocol;
			IpcErrorCode errorCode;
			PredictionTelemetry predictionTelemetry;
			DriverTelemetry driverTelemetry;
		};

		Response() : type(ResponseInvalid) { }
		Response(ResponseType type) : type(type) { }
	};

	inline constexpr size_t IpcRequestSize = sizeof(Request);
	inline constexpr size_t IpcResponseSize = sizeof(Response);

	static_assert(IpcRequestSize > 0, "IPC request size must be non-zero");
	static_assert(IpcResponseSize > 0, "IPC response size must be non-zero");
}