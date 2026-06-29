// SPDX-License-Identifier: AGPL-3.0-only

#include "UserInterface.h"
#include "Calibration.h"
#include "Configuration.h"
#include "Version.h"

#include <string>
#include <vector>
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <imgui.h>

const ImGuiWindowFlags bareWindowFlags =
		ImGuiWindowFlags_NoTitleBar |
		ImGuiWindowFlags_NoResize |
		ImGuiWindowFlags_NoMove |
		ImGuiWindowFlags_NoScrollbar |
		ImGuiWindowFlags_NoScrollWithMouse;

const ImGuiWindowFlags modalWindowFlags =
		ImGuiWindowFlags_NoTitleBar |
		ImGuiWindowFlags_NoResize |
		ImGuiWindowFlags_NoMove;

void UserInterface::Render(bool runningInOverlay)
{
	auto textWithWidth = [](const char *label, const char *text, float width) {
		ImGui::BeginChild(label, ImVec2(width, ImGui::GetTextLineHeightWithSpacing()));
		ImGui::Text(text);
		ImGui::EndChild();
	};

	auto &io = ImGui::GetIO();
	ImGuiStyle &style = ImGui::GetStyle();

	ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f));
	ImGui::SetNextWindowSize(io.DisplaySize);

	if (!ImGui::Begin("MainWindow", nullptr, bareWindowFlags))
	{
		ImGui::End();
		return;
	}

	ImGui::PushStyleColor(ImGuiCol_PlotHistogram, ImGui::GetStyleColorVec4(ImGuiCol_Button));

	if (ImGui::BeginTabBar("##tabs")) {

		if (ImGui::BeginTabItem("Calibration")) {

			VRState state;
			{
				auto &trackingSystems = state.trackingSystems;
				char buffer[vr::k_unMaxPropertyStringSize];

				for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
				{
					vr::ETrackedPropertyError err = vr::TrackedProp_Success;
					auto deviceClass = vr::VRSystem()->GetTrackedDeviceClass(id);
					if (deviceClass == vr::TrackedDeviceClass_Invalid)
						continue;

					if (deviceClass != vr::TrackedDeviceClass_TrackingReference)
					{
						vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_TrackingSystemName_String, buffer, vr::k_unMaxPropertyStringSize, &err);

						if (err == vr::TrackedProp_Success)
						{
							std::string system(buffer);
							auto existing = std::find(trackingSystems.begin(), trackingSystems.end(), system);
							if (existing != trackingSystems.end())
							{
								if (deviceClass == vr::TrackedDeviceClass_HMD)
								{
									trackingSystems.erase(existing);
									trackingSystems.insert(trackingSystems.begin(), system);
								}
							}
							else
							{
								trackingSystems.push_back(system);
							}

							VRDevice device;
							device.id = id;
							device.deviceClass = deviceClass;
							device.trackingSystem = system;

							vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_ModelNumber_String, buffer, vr::k_unMaxPropertyStringSize, &err);
							device.model = std::string(buffer);

							vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_SerialNumber_String, buffer, vr::k_unMaxPropertyStringSize, &err);
							device.serial = std::string(buffer);

							device.controllerRole = (vr::ETrackedControllerRole) vr::VRSystem()->GetInt32TrackedDeviceProperty(id, vr::Prop_ControllerRoleHint_Int32, &err);
							state.devices.push_back(device);
						}
						else
						{
							printf("failed to get tracking system name for id %d\n", id);
						}
					}
				}
			}

			if (CalCtx.validProfile)
				CalCtx.overrideStatus = EvaluateOverrideStatus(CalCtx);

			const VRDevice *hmd = nullptr;
			const VRDevice *tracker = nullptr;
			for (auto &device : state.devices)
			{
				if (device.id == vr::k_unTrackedDeviceIndex_Hmd)
					hmd = &device;
				if (!CalCtx.trackerSerial.empty() && device.serial == CalCtx.trackerSerial)
					tracker = &device;
			}

			if (hmd)
				ImGui::Text("HMD: %s (%s)", hmd->serial.c_str(), hmd->trackingSystem.c_str());
			else
				ImGui::TextColored(ImColor(0.8f, 0.2f, 0.2f), "No HMD detected");

			if (CalCtx.overrideStatus.active && tracker)
				ImGui::TextColored(ImColor(0.2f, 0.7f, 0.2f), "Override active: HMD driven by %s (%s)", tracker->serial.c_str(), tracker->trackingSystem.c_str());
			else if (!CalCtx.validProfile)
				ImGui::TextColored(ImColor(0.5f, 0.5f, 0.5f), "No calibration. Press Calibrate, then move your head to identify the headset tracker.");
			else
				ImGui::TextColored(ImColor(0.8f, 0.2f, 0.2f), "Override inactive: %s", OverrideInactiveReasonText(CalCtx.overrideStatus.inactiveReason));

			if (!CalCtx.overrideStatus.active)
			{
				ImGui::Spacing();
				if (ImGui::CollapsingHeader("Setup checklist", ImGuiTreeNodeFlags_DefaultOpen))
				{
					const auto driverOk = CalCtx.driverTelemetryValid && CalCtx.driverTelemetry.poseHooksInstalled;
					const auto ipcOk = CalCtx.ipcHealthy;
					const auto profileOk = CalCtx.validProfile;
					const auto overrideOk = CalCtx.overrideStatus.active;

					ImGui::BulletText("%s Driver loaded with pose hooks", driverOk ? "[OK]" : "[--]");
					if (!driverOk)
						ImGui::TextDisabled("    Enable the spaceoverride driver in SteamVR settings, then restart SteamVR.");

					ImGui::BulletText("%s IPC to driver", ipcOk ? "[OK]" : "[--]");
					if (!ipcOk)
						ImGui::TextDisabled("    Only one overlay instance; restart SteamVR if driver was just updated.");

					ImGui::BulletText("%s Calibration profile saved", profileOk ? "[OK]" : "[--]");
					if (!profileOk)
						ImGui::TextDisabled("    Press Calibrate (mount rigid tracker on headset first).");

					ImGui::BulletText("%s Override active", overrideOk ? "[OK]" : "[--]");
					if (profileOk && !overrideOk)
						ImGui::TextDisabled("    %s", OverrideInactiveReasonText(CalCtx.overrideStatus.inactiveReason));

					if (!profileOk)
					{
						ImGui::Spacing();
						ImGui::TextWrapped(
							"First time: mount a Vive tracker on your headset, press Calibrate, "
							"move your head when asked, then follow the look left/center/right prompts. "
							"Use Slow speed if quality is poor.");
					}
				}
			}

			ImGui::Text("");

			std::vector<const VRDevice*> connectedTrackers;
			for (auto& device : state.devices)
			{
				if (device.deviceClass == vr::TrackedDeviceClass_GenericTracker)
					connectedTrackers.push_back(&device);
			}

			if (!connectedTrackers.empty() && CalCtx.state == CalibrationState::None)
			{
				ImGui::SeparatorText("Headset Tracker");
				const char* preview = CalCtx.preferredTrackerSerial.empty()
					? "Auto-detect (move head during calibration)"
					: CalCtx.preferredTrackerSerial.c_str();
				if (ImGui::BeginCombo("Preferred tracker", preview))
				{
					const bool autoSelected = CalCtx.preferredTrackerSerial.empty();
					if (ImGui::Selectable("Auto-detect (move head during calibration)", autoSelected))
					{
						if (!CalCtx.preferredTrackerSerial.empty())
						{
							CalCtx.preferredTrackerSerial.clear();
							SaveProfile(CalCtx);
						}
					}
					for (const VRDevice* tracker : connectedTrackers)
					{
						const bool selected = CalCtx.preferredTrackerSerial == tracker->serial;
						std::string label = tracker->serial + " (" + tracker->model + ")";
						if (ImGui::Selectable(label.c_str(), selected))
						{
							if (CalCtx.preferredTrackerSerial != tracker->serial)
							{
								CalCtx.preferredTrackerSerial = tracker->serial;
								SaveProfile(CalCtx);
							}
						}
					}
					ImGui::EndCombo();
				}
				ImGui::SetItemTooltip(
					"Pick which Vive tracker is mounted on your headset.\n"
					"Auto-detect correlates head motion with tracker motion when multiple trackers are connected.");
				ImGui::Text("");
			}

			if (CalCtx.calibrationFailureOffer && CalCtx.state == CalibrationState::None)
			{
				ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.35f, 1.0f),
					"Last calibration failed (RMS %.1f mm). Your previous profile was restored.",
					CalCtx.lastCalibrationRmsMm);

				float failButtonWidth = (ImGui::GetContentRegionAvail().x - style.ItemSpacing.x) / 2.0f;
				if (ImGui::Button("Retry Calibration", ImVec2(failButtonWidth, ImGui::GetTextLineHeight() * 2)))
				{
					ImGui::OpenPopup("Calibration Progress");
					RetryCalibrationAfterFailure();
				}
				ImGui::SameLine();
				if (ImGui::Button("Restore Previous", ImVec2(failButtonWidth, ImGui::GetTextLineHeight() * 2)))
					RestoreCalibrationAfterFailure();

				ImGui::Text("");
			}

			if (CalCtx.state == CalibrationState::None)
			{
				const bool stageSpace = IsStageTrackingSpace();
				const bool nativeBlocked = stageSpace && CalCtx.enableNative;

				if (nativeBlocked)
				{
					ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.35f, 1.0f),
						"Discard Calibrated Offset requires Local (seated) tracking space. Switch SteamVR to seated/local mode or disable the option.");
				}

				float buttonWidth = ImGui::GetContentRegionAvail().x;
				if (CalCtx.validProfile)
					buttonWidth = (buttonWidth - style.ItemSpacing.x * 2.0f) / 3.0f;

				ImGui::BeginDisabled(nativeBlocked);
				if (ImGui::Button("Calibrate", ImVec2(buttonWidth, ImGui::GetTextLineHeight() * 2)))
				{
					ImGui::OpenPopup("Calibration Progress");
					StartCalibration();
				}
				ImGui::EndDisabled();

				if (CalCtx.validProfile)
				{
					ImGui::SameLine();
					if (ImGui::Button("Edit Calibration", ImVec2(buttonWidth, ImGui::GetTextLineHeight() * 2)))
					{
						CalCtx.state = CalibrationState::Editing;
					}

					ImGui::SameLine();
					if (ImGui::Button("Remove Calibration", ImVec2(buttonWidth, ImGui::GetTextLineHeight() * 2)))
					{
						CalCtx.Clear();
						SaveProfile(CalCtx);
					}
				}

				ImGui::Text("");
				ImGui::SeparatorText("Play Area");
				ImGui::TextWrapped(
					"Save or restore your SteamVR chaperone bounds with your calibration profile. "
					"Restoring bounds changes your play area — confirm before applying.");

				float chapWidth = ImGui::GetContentRegionAvail().x;
				if (CalCtx.chaperone.valid)
					chapWidth = (chapWidth - style.ItemSpacing.x) / 2.0f;

				if (ImGui::Button("Save Chaperone to Profile", ImVec2(chapWidth, ImGui::GetTextLineHeight() * 2)))
				{
					LoadChaperoneBounds();
					SaveProfile(CalCtx);
				}

				if (CalCtx.chaperone.valid)
				{
					ImGui::SameLine();
					if (ImGui::Button("Restore Chaperone from Profile", ImVec2(chapWidth, ImGui::GetTextLineHeight() * 2)))
						ImGui::OpenPopup("Confirm Chaperone Restore");
				}

				if (ImGui::Checkbox("Auto-restore chaperone when SteamVR resets geometry", &CalCtx.chaperone.autoApply))
					SaveProfile(CalCtx);

				if (ImGui::BeginPopupModal("Confirm Chaperone Restore", nullptr, modalWindowFlags))
				{
					ImGui::TextWrapped("This will overwrite your current SteamVR play area with the saved profile bounds.");
					ImGui::Spacing();
					if (ImGui::Button("Restore", ImVec2(ImGui::GetContentRegionAvail().x * 0.5f - style.ItemSpacing.x, 0)))
					{
						ApplyChaperoneBounds();
						ImGui::CloseCurrentPopup();
					}
					ImGui::SameLine();
					if (ImGui::Button("Cancel", ImVec2(0, 0)))
						ImGui::CloseCurrentPopup();
					ImGui::EndPopup();
				}

				ImGui::Text("");
			}
			else if (CalCtx.state == CalibrationState::Editing)
			{
				float width = ImGui::GetContentRegionAvail().x / 3.0f - style.FramePadding.x;
				float widthF = width - style.FramePadding.x;

				textWithWidth("YawLabel", "Yaw", width);
				ImGui::SameLine();
				textWithWidth("PitchLabel", "Pitch", width);
				ImGui::SameLine();
				textWithWidth("RollLabel", "Roll", width);

				ImGui::PushItemWidth(widthF);
				ImGui::InputDouble("##Yaw", &CalCtx.calibratedRotation(1), 0.1, 1.0, "%.8f");
				ImGui::SameLine();
				ImGui::InputDouble("##Pitch", &CalCtx.calibratedRotation(2), 0.1, 1.0, "%.8f");
				ImGui::SameLine();
				ImGui::InputDouble("##Roll", &CalCtx.calibratedRotation(0), 0.1, 1.0, "%.8f");

				textWithWidth("XLabel", "X", width);
				ImGui::SameLine();
				textWithWidth("YLabel", "Y", width);
				ImGui::SameLine();
				textWithWidth("ZLabel", "Z", width);

				ImGui::InputDouble("##X", &CalCtx.calibratedTranslation(0), 1.0, 10.0, "%.8f");
				ImGui::SameLine();
				ImGui::InputDouble("##Y", &CalCtx.calibratedTranslation(1), 1.0, 10.0, "%.8f");
				ImGui::SameLine();
				ImGui::InputDouble("##Z", &CalCtx.calibratedTranslation(2), 1.0, 10.0, "%.8f");

				textWithWidth("ScaleLabel", "Scale", width);

				ImGui::InputDouble("##Scale", &CalCtx.calibratedScale, 0.0001, 0.01, "%.8f");
				ImGui::PopItemWidth();

				if (ImGui::Button("Save Profile", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetTextLineHeight() * 2)))
				{
					SaveProfile(CalCtx);
					CalCtx.state = CalibrationState::None;
				}
			}
			else
			{
				const char* progressLabel = CalCtx.state == CalibrationState::PartialSampling
					|| CalCtx.state == CalibrationState::Recovering
					? "Re-estimating mount offset..."
					: "Calibration in progress...";
				ImGui::Button(progressLabel, ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetTextLineHeight() * 2));
				if (CalCtx.liveCalibrationQualityValid)
				{
					const double thresholdMm = CalCtx.state == CalibrationState::PartialSampling
						? CalibrationContext::PartialMountRmsThresholdMeters * 1000.0
						: CalibrationContext::FullCalibrationRmsThresholdMeters * 1000.0;
					const ImVec4 color = CalCtx.liveCalibrationRmsMm <= thresholdMm
						? ImVec4(0.4f, 1.0f, 0.4f, 1.0f)
						: ImVec4(1.0f, 0.7f, 0.2f, 1.0f);
					ImGui::TextColored(color, "Live quality (RMS): %.1f mm (target <= %.0f mm)", CalCtx.liveCalibrationRmsMm, thresholdMm);
				}
			}

			float footerHeight = ImGui::GetTextLineHeightWithSpacing() * (runningInOverlay ? 2.0f : 1.0f);
			ImGui::SetCursorPos(ImVec2(10.0f, ImGui::GetWindowHeight() - footerHeight - style.WindowPadding.y));
			ImGui::BeginChild("##bottom_line", ImVec2(ImGui::GetWindowWidth() - 20.0f, footerHeight), false, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
			ImGui::Text("OpenVR-SpaceOverride v%s (community fork)", SPACECAL_VERSION_STRING);
			ImGui::TextDisabled("Based on Nyabsi/SpaceOverride; calibration math from OpenVR-SpaceCalibrator");
			if (runningInOverlay)
			{
				ImGui::Text("close VR overlay to use mouse");
			}
			ImGui::EndChild();

			ImGui::SetNextWindowPos(ImVec2(20.0f, 20.0f));
			ImGui::SetNextWindowSize(ImVec2(io.DisplaySize.x - 40.0f, io.DisplaySize.y - 40.0f));
			if (ImGui::BeginPopupModal("Calibration Progress", nullptr, modalWindowFlags))
			{
				ImGui::PushStyleColor(ImGuiCol_FrameBg, (ImVec4)ImColor(0, 0, 0));
				for (auto &message : CalCtx.messages)
				{
					switch (message.type)
					{
					case CalibrationContext::Message::String:
						ImGui::TextWrapped(message.str.c_str());
						break;
					case CalibrationContext::Message::Progress:
						float fraction = (float)message.progress / (float)message.target;
						ImGui::Text("");
						ImGui::ProgressBar(fraction, ImVec2(-1.0f, 0.0f), "");
						ImGui::SetCursorPosY(ImGui::GetCursorPosY() - ImGui::GetFontSize() - style.FramePadding.y * 2);
						ImGui::Text(" %d%%", (int)(fraction * 100));
						if (CalCtx.liveCalibrationQualityValid)
						{
							const double thresholdMm = CalCtx.state == CalibrationState::PartialSampling
								? CalibrationContext::PartialMountRmsThresholdMeters * 1000.0
								: CalibrationContext::FullCalibrationRmsThresholdMeters * 1000.0;
							ImGui::Text("Live quality (RMS): %.1f mm (target <= %.0f mm)", CalCtx.liveCalibrationRmsMm, thresholdMm);
						}
						break;
					}
				}
				ImGui::PopStyleColor();

				if (CalCtx.calibrationFailureOffer && CalCtx.state == CalibrationState::None)
				{
					ImGui::Text("");
					float failButtonWidth = (ImGui::GetContentRegionAvail().x - style.ItemSpacing.x) / 2.0f;
					if (ImGui::Button("Retry", ImVec2(failButtonWidth, ImGui::GetTextLineHeight() * 2)))
					{
						RetryCalibrationAfterFailure();
						ImGui::CloseCurrentPopup();
						ImGui::OpenPopup("Calibration Progress");
					}
					ImGui::SameLine();
					if (ImGui::Button("Restore Previous", ImVec2(failButtonWidth, ImGui::GetTextLineHeight() * 2)))
					{
						RestoreCalibrationAfterFailure();
						ImGui::CloseCurrentPopup();
					}
				}
				else if (CalCtx.state == CalibrationState::None)
				{
					ImGui::Text("");
					if (ImGui::Button("Close", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetTextLineHeight() * 2)))
						ImGui::CloseCurrentPopup();
				}

				ImGui::EndPopup();
			}

			ImGui::EndTabItem();
		}

		if (ImGui::BeginTabItem("Smoothing"))
		{
			ImGui::TextColored(ImVec4(0.4f, 1.0f, 0.4f, 1.0f), "NOTE: Changes here take effect instantly, no need to re-calibrate.");
			ImGui::Spacing();
			ImGui::TextWrapped(
				"These settings smooth out tracking so your view and devices look steady instead of shaky. "
				"If something looks shaky, add more smoothing. If it feels laggy or floaty when you move, "
				"ease off. Not sure? Hover over a slider for tips.");
			ImGui::Spacing();

			const double cutoffMin = 0.1, cutoffMax = 5.0;
			const double betaMin = 0.0, betaMax = 2.0;

			auto paramSliders = [&](const char* id, protocol::OneEuroParams& p) {
				bool c = false;
				ImGui::PushID(id);

				ImGui::Text("minCutoff");
				ImGui::SameLine(170);
				ImGui::SetNextItemWidth(-1);
				c |= ImGui::SliderScalar("##minCutoff", ImGuiDataType_Double, &p.minCutoff, &cutoffMin, &cutoffMax, "%.3f Hz");
				ImGui::SetItemTooltip(
					"How steady things look when you are not moving.\n"
					"Drag left to remove shaking for a calmer image; drag right if things start to feel laggy or floaty."
				);

				ImGui::Text("beta");
				ImGui::SameLine(170);
				ImGui::SetNextItemWidth(-1);
				c |= ImGui::SliderScalar("##beta", ImGuiDataType_Double, &p.beta, &betaMin, &betaMax, "%.3f Hz");
				ImGui::SetItemTooltip(
					"How quickly tracking keeps up when you move fast.\n"
					"Drag right if fast movements feel delayed or laggy; drag left if they look shaky."
				);

				ImGui::Text("dCutoff");
				ImGui::SameLine(170);
				ImGui::SetNextItemWidth(-1);
				c |= ImGui::SliderScalar("##dCutoff", ImGuiDataType_Double, &p.dCutoff, &cutoffMin, &cutoffMax, "%.3f Hz");
				ImGui::SetItemTooltip(
					"Most people can leave this alone.\n"
					"It fine-tunes how the smoothing reacts as your movement speed changes."
				);

				ImGui::PopID();
				return c;
			};

			bool changed = false;

			changed |= ImGui::Checkbox("Smooth headset tracker", &CalCtx.headFilterEnabled);
			ImGui::SetItemTooltip("Steadies what you see through the headset to reduce shaking. This can add a tiny bit of delay, so if your view feels laggy when you move quickly, adjust the sliders below.");

			ImGui::Spacing();
			ImGui::SeparatorText("Headset Tracker");
			ImGui::BeginDisabled(!CalCtx.headFilterEnabled);
			changed |= paramSliders("head", CalCtx.headFilterParams);
			ImGui::EndDisabled();

			ImGui::Spacing();
			ImGui::SeparatorText("Relative Calibration");
			ImGui::TextWrapped(
				"Keeps your controllers and other tracked devices lined up with your real space, and "
				"steadies your view if the headset briefly loses tracking. Add more smoothing if they "
				"look shaky; ease off if they are slow to line up.");
			ImGui::Spacing();
			changed |= paramSliders("drift", CalCtx.driftFilterParams);

			if (changed)
				SendOneEuroParams();

			ImGui::EndTabItem();
		}

		if (ImGui::BeginTabItem("Settings"))
		{
			ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.2f, 1.0f), "NOTE: Calibration Speed requires re-calibration. Other settings apply immediately.");
			ImGui::Spacing();
			ImGui::Text("Tip: hover over the settings to see additional information.");
			ImGui::Spacing();

			float halfWidth = (ImGui::GetContentRegionAvail().x - style.ItemSpacing.x) * 0.5f;
			bool runtimeSettingsChanged = false;
			const double settingsApplyTime = CalCtx.timeLastTick;

			ImGui::BeginChild("##left_panel", ImVec2(halfWidth, 0), true);

			runtimeSettingsChanged |= ImGui::Checkbox("Fallback to SLAM", &CalCtx.fallbackToSlam);
			ImGui::SetItemTooltip(
				"Temporarily uses HMD (SLAM) tracking if the headset tracker loses line of sight.");

			runtimeSettingsChanged |= ImGui::Checkbox("Enable Angular Velocity", &CalCtx.enableAngularVelocity);
			ImGui::SetItemTooltip(
				"Feeds angular velocity into SteamVR for smoother prediction.\n"
				"On by default; disable only if a device misbehaves with ang-vel enabled.");

			runtimeSettingsChanged |= ImGui::Checkbox("Auto partial recal on mount drift", &CalCtx.autoPartialRecalOnMountDrift);
			ImGui::SetItemTooltip(
				"When runtime mount residual stays above 30 mm for ~7.5 s, automatically runs a partial\n"
				"recalibration to refresh the tracker-to-HMD offset. Cooldown: 5 minutes.");

			runtimeSettingsChanged |= ImGui::Checkbox("Relative Calibration", &CalCtx.continuousSync);
			ImGui::SetItemTooltip(
				"Continuously re-aligns SLAM-tracked devices (controllers etc.) to the calibrated space\n"
				"in the background. The headset's raw SLAM pose is compared against the tracker-driven\n"
				"pose to measure SLAM drift, and the correction is applied gradually and automatically.");

			runtimeSettingsChanged |= ImGui::Checkbox("Sync HMD drift (advanced)", &CalCtx.syncHmdDrift);
			ImGui::SetItemTooltip(
				"Also applies SLAM drift correction to the HMD pose. Off by default because the HMD is\n"
				"normally driven by the lighthouse tracker.");

			const bool stageSpace = IsStageTrackingSpace();
			ImGui::BeginDisabled(stageSpace);
			runtimeSettingsChanged |= ImGui::Checkbox("Discard Calibrated Offset", &CalCtx.enableNative);
			ImGui::EndDisabled();
			if (stageSpace)
			{
				ImGui::SameLine();
				ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.2f, 1.0f), "(requires Local/seated space)");
			}
			ImGui::SetItemTooltip(
				"Discards all SLAM tracking (even as fallback) and feeds only raw tracker data with the offset applied.\n"
				"Downside: a yaw orientation mismatch may occur. This only works in Local (seated) tracking space, where\n"
				"re-centering adjusts the yaw - Stage (standing) tracking space never re-centers yaw, so the mismatch cannot\n"
				"be corrected there.\n"
				"Will not work on all devices - tested only on Pico.");

			ImGui::EndChild();

			ImGui::SameLine();

			ImGui::BeginChild("##right_panel", ImVec2(0, 0), true);

			ImGui::Text("Prediction Time");
			ImGui::SameLine();
			const float sliderMax = CalCtx.predictionAuto ? 4.0f : 10.0f;
			ImGui::BeginDisabled(CalCtx.predictionAuto);
			if (ImGui::SliderFloat("##prediction_time", &CalCtx.predictionTime, 0.0f, sliderMax, "%.1f frames"))
				runtimeSettingsChanged = true;
			if (ImGui::IsItemDeactivatedAfterEdit())
			{
				CalCtx.predictionAuto = false;
				runtimeSettingsChanged = true;
			}
			ImGui::EndDisabled();
			ImGui::SetItemTooltip(
				"How many frames of prediction SteamVR applies to the tracker.\n"
				"Wireless streaming (e.g. Virtual Desktop) often benefits from more prediction.\n"
				"Adjusting the slider disables Auto mode.");

			runtimeSettingsChanged |= ImGui::Checkbox("Auto", &CalCtx.predictionAuto);
			ImGui::SetItemTooltip(
				"Estimates wireless latency from tracker vs SLAM velocity over a 2 s window,\n"
				"adds a small wireless bias, and tunes prediction to 0–4 frames. Disable for manual control.");

			if (CalCtx.predictionAuto)
			{
				ImGui::Text("Applied: %.1f frames", CalCtx.autoPredictionFrames);
			}

			if (CalCtx.predictionTelemetryValid)
			{
				ImGui::Text("Estimated lag: %.1f ms (%.1f frames)",
					CalCtx.predictionLagMs,
					CalCtx.predictionLagFrames);
			}
			else if (CalCtx.predictionAuto)
			{
				ImGui::TextDisabled("Estimated lag: rotate head to measure");
			}

			ImGui::Spacing();

			ImGui::Text("Calibration Speed");

			auto speed = CalCtx.calibrationSpeed;

			if (ImGui::RadioButton("Fast", speed == CalibrationContext::FAST))
				CalCtx.calibrationSpeed = CalibrationContext::FAST;

			if (ImGui::RadioButton("Slow", speed == CalibrationContext::SLOW))
				CalCtx.calibrationSpeed = CalibrationContext::SLOW;

			if (ImGui::RadioButton("Very Slow", speed == CalibrationContext::VERY_SLOW))
				CalCtx.calibrationSpeed = CalibrationContext::VERY_SLOW;

			ImGui::SetItemTooltip("Controls how long calibration deltas are collected.");

			ImGui::EndChild();

			if (runtimeSettingsChanged && CalCtx.validProfile)
				ApplyRuntimeDriverSettings(settingsApplyTime);

			ImGui::Spacing();
			if (ImGui::CollapsingHeader("Diagnostics", ImGuiTreeNodeFlags_DefaultOpen))
			{
				if (!CalCtx.ipcHealthy)
					ImGui::TextColored(ImVec4(0.9f, 0.35f, 0.3f, 1.0f), "IPC: unhealthy (driver unreachable or rejected last request)");

				if (CalCtx.driverTelemetryValid)
				{
					const auto& t = CalCtx.driverTelemetry;
					ImGui::Text("Pose hooks: %s", t.poseHooksInstalled ? "installed" : "missing");
					if (t.overrideActive)
						ImGui::TextColored(ImVec4(0.2f, 0.8f, 0.3f, 1.0f), "Driver override: active");
					else
						ImGui::TextColored(ImVec4(0.9f, 0.35f, 0.3f, 1.0f),
							"Driver override: %s", DriverOverrideInactiveReasonText(t.overrideInactiveReason));

					ImGui::Text("Tracker pose: %s", t.trackerValid ? "valid" : "lost");
					if (!t.trackerValid && t.trackerLostSeconds > 0.0f)
						ImGui::Text("Tracker lost for: %.1f s", t.trackerLostSeconds);
					if (t.trackerBlendActive)
						ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.2f, 1.0f), "Tracker/SLAM blend active");

					if (t.driftValid)
					{
						ImGui::Text("Drift yaw: %.1f deg", t.driftYawDeg);
						ImGui::Text("Drift translation: %.1f mm", t.driftTranslationMm);
					}
					else
						ImGui::TextDisabled("Drift correction: inactive");

					ImGui::Text("Prediction: %.1f frames @ %.0f Hz",
						t.appliedPredictionFrames,
						t.displayHz);
				}
				else
				{
					ImGui::TextDisabled("Driver telemetry unavailable — is the Space Override driver enabled?");
				}

				if (CalCtx.validProfile)
				{
					const OverrideStatus status = EvaluateOverrideStatus(CalCtx);
					if (status.active)
						ImGui::TextColored(ImVec4(0.2f, 0.8f, 0.3f, 1.0f), "Override: active");
					else
						ImGui::TextColored(ImVec4(0.9f, 0.35f, 0.3f, 1.0f),
							"Override: inactive (%s)", OverrideInactiveReasonText(status.inactiveReason));

					if (CalCtx.lastCalibrationRmsMm > 0.0)
						ImGui::Text("Last calibration RMS: %.1f mm", CalCtx.lastCalibrationRmsMm);

					if (CalCtx.runtimeResidualValid)
					{
						const ImVec4 color = CalCtx.runtimeResidualMm <= CalibrationContext::RuntimeResidualWarnMm
							? ImVec4(0.2f, 0.8f, 0.3f, 1.0f)
							: ImVec4(0.9f, 0.35f, 0.3f, 1.0f);
						ImGui::TextColored(color, "Runtime mount residual: %.1f mm", CalCtx.runtimeResidualMm);
					}

					if (CalCtx.guardianShiftSuspect)
						ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.2f, 1.0f),
							"Guardian shift suspect (SLAM jump %.0f mm, tracker stable)", CalCtx.guardianShiftSlamJumpMm);

					if (CalCtx.mountRigidityWarning)
						ImGui::TextColored(ImVec4(0.9f, 0.35f, 0.3f, 1.0f),
							"Mount rigidity warning: partial RMS %.1f mm vs baseline %.1f mm",
							CalCtx.lastPartialMountRmsMm, CalCtx.baselineMountRmsMm);

					if (CalCtx.lastCalibrationTime > 0.0)
					{
						const std::time_t calTime = (std::time_t)CalCtx.lastCalibrationTime;
						char timeBuf[64] = {};
						if (std::strftime(timeBuf, sizeof timeBuf, "%Y-%m-%d %H:%M:%S", std::localtime(&calTime)))
							ImGui::Text("Last calibrated: %s", timeBuf);
					}
				}
				else
				{
					ImGui::TextDisabled("No calibration profile loaded");
				}

				ImGui::Separator();
				static char profileFilePath[512] = {};
				static bool profilePathInitialized = false;
				if (!profilePathInitialized)
				{
					const char* docs = std::getenv("USERPROFILE");
					if (docs)
						snprintf(profileFilePath, sizeof profileFilePath, "%s\\Documents\\OpenVR-SpaceOverride-profile.json", docs);
					profilePathInitialized = true;
				}
				ImGui::InputText("Profile file", profileFilePath, sizeof profileFilePath);
				if (ImGui::Button("Export profile") && CalCtx.validProfile)
				{
					if (ExportProfileToFile(CalCtx, profileFilePath))
						ImGui::OpenPopup("ProfileExportOk");
				}
				ImGui::SameLine();
				if (ImGui::Button("Import profile"))
				{
					if (ImportProfileFromFile(CalCtx, profileFilePath))
						ImGui::OpenPopup("ProfileImportOk");
					else
						ImGui::OpenPopup("ProfileImportFail");
				}
				if (ImGui::BeginPopup("ProfileExportOk"))
				{
					ImGui::Text("Profile exported.");
					ImGui::EndPopup();
				}
				if (ImGui::BeginPopup("ProfileImportOk"))
				{
					ImGui::Text("Profile imported and saved to registry.");
					ImGui::EndPopup();
				}
				if (ImGui::BeginPopup("ProfileImportFail"))
				{
					ImGui::TextColored(ImVec4(0.9f, 0.35f, 0.3f, 1.0f), "Import failed — check path and JSON.");
					ImGui::EndPopup();
				}
			}

			ImGui::EndTabItem();
		}

		ImGui::EndTabBar();
	}

	ImGui::PopStyleColor();
	ImGui::End();
}
