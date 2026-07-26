// SPDX-License-Identifier: AGPL-3.0-only

#include "UserInterface.h"
#include "Calibration.h"
#include "Configuration.h"
#include "Version.h"

#include <string>
#include <vector>
#include <algorithm>
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

			if (!CalCtx.validProfile)
				ImGui::TextColored(ImColor(0.5f, 0.5f, 0.5f), "No calibration. Press Calibrate, then move your head to identify the headset tracker.");
			else if (!tracker)
				ImGui::TextColored(ImColor(0.8f, 0.2f, 0.2f), "Headset tracker (%s) not connected, override disabled", CalCtx.trackerSerial.c_str());
			else if (!CalCtx.enabled)
				ImGui::TextColored(ImColor(0.8f, 0.2f, 0.2f), "Override disabled (HMD tracking system changed?)");
			else
				ImGui::TextColored(ImColor(0.2f, 0.7f, 0.2f), "Override active: HMD driven by %s (%s)", tracker->serial.c_str(), tracker->trackingSystem.c_str());

			ImGui::Text("");

			if (CalCtx.state == CalibrationState::None)
			{
				float buttonWidth = ImGui::GetContentRegionAvail().x;
				if (CalCtx.validProfile)
					buttonWidth = (buttonWidth - style.ItemSpacing.x * 2.0f) / 3.0f;

				if (ImGui::Button("Calibrate", ImVec2(buttonWidth, ImGui::GetTextLineHeight() * 2)))
				{
					ImGui::OpenPopup("Calibration Progress");
					StartCalibration();
				}

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
						RemoveProfile(CalCtx);
					}
				}

				/*
				float chapWidth = ImGui::GetContentRegionAvail().x;
				if (CalCtx.chaperone.valid)
					chapWidth = (chapWidth - style.ItemSpacing.x) / 2.0f;

				ImGui::Text("");
				if (ImGui::Button("Copy Chaperone Bounds to profile", ImVec2(chapWidth, ImGui::GetTextLineHeight() * 2)))
				{
					LoadChaperoneBounds();
					SaveProfile(CalCtx);
				}

				if (CalCtx.chaperone.valid)
				{
					ImGui::SameLine();
					if (ImGui::Button("Paste Chaperone Bounds", ImVec2(chapWidth, ImGui::GetTextLineHeight() * 2)))
					{
						ApplyChaperoneBounds();
					}

					if (ImGui::Checkbox(" Paste Chaperone Bounds automatically when geometry resets", &CalCtx.chaperone.autoApply))
					{
						SaveProfile(CalCtx);
					}
				}

				ImGui::Text("");
				*/
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
				ImGui::SameLine();
				textWithWidth("HmdScaleLabel", "HMD Scale", width);

				ImGui::InputDouble("##Scale", &CalCtx.calibratedScale, 0.0001, 0.01, "%.8f");
				ImGui::SameLine();
				ImGui::InputDouble("##HmdScale", &CalCtx.hmdScale, 0.0001, 0.01, "%.8f");
				ImGui::PopItemWidth();

				if (ImGui::Button("Save Profile", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetTextLineHeight() * 2)))
				{
					SaveProfile(CalCtx);
					CalCtx.state = CalibrationState::None;
				}
			}
			else
			{
				ImGui::Button("Calibration in progress...", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetTextLineHeight() * 2));
			}

			float footerHeight = ImGui::GetTextLineHeightWithSpacing() * (runningInOverlay ? 2.0f : 1.0f);
			ImGui::SetCursorPos(ImVec2(10.0f, ImGui::GetWindowHeight() - footerHeight - style.WindowPadding.y));
			ImGui::BeginChild("##bottom_line", ImVec2(ImGui::GetWindowWidth() - 20.0f, footerHeight), false, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
			ImGui::Text("OpenVR-SpaceOverride v" SPACECAL_VERSION_STRING " fusion fork - by helpgore (Discord: helpgore) | upstream by Nyabsi | Space Calibrator by tach/pushrax");
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
						break;
					}
				}
				ImGui::PopStyleColor();

				// Live motion coaching during sampling: what the solver still needs.
				if (CalCtx.state == CalibrationState::Sampling && !CalCtx.sampleHint.empty())
				{
					ImVec4 hintCol =
						CalCtx.sampleHintLevel >= 2 ? ImVec4(1.0f, 0.55f, 0.2f, 1.0f) :  // slow down
						CalCtx.sampleHintLevel == 1 ? ImVec4(1.0f, 0.85f, 0.3f, 1.0f) :  // add motion
						                              ImVec4(0.4f, 1.0f, 0.4f, 1.0f);    // on track
					ImGui::Text("");
					ImGui::PushStyleColor(ImGuiCol_Text, hintCol);
					ImGui::TextWrapped("%s", CalCtx.sampleHint.c_str());
					ImGui::PopStyleColor();
				}

				if (CalCtx.state == CalibrationState::None)
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
			// Live driver settings (steamvr.vrsettings). The driver polls these ~1 Hz,
			// so they apply without re-calibrating or restarting SteamVR.
			{
				static bool fusionMode = false, fusionDiag = false;
				static double lastRead = -1.0;
				const double nowTime = ImGui::GetTime();
				if (lastRead < 0.0 || (nowTime - lastRead) > 0.5)
				{
					vr::EVRSettingsError err = vr::VRSettingsError_None;
					bool v = vr::VRSettings()->GetBool("driver_spaceoverride", "fusionMode", &err);
					if (err == vr::VRSettingsError_None) fusionMode = v;
					err = vr::VRSettingsError_None;
					v = vr::VRSettings()->GetBool("driver_spaceoverride", "fusionDiag", &err);
					if (err == vr::VRSettingsError_None) fusionDiag = v;
					lastRead = nowTime;
				}

				ImGui::TextColored(ImVec4(0.4f, 1.0f, 0.4f, 1.0f),
					"NOTE: Tracking mode applies within ~1 second. No re-calibration needed.");
				ImGui::Spacing();

				if (ImGui::Checkbox("Fusion mode", &fusionMode))
				{
					vr::EVRSettingsError err = vr::VRSettingsError_None;
					vr::VRSettings()->SetBool("driver_spaceoverride", "fusionMode", fusionMode, &err);
				}
				ImGui::SetItemTooltip(
					"ON (Fusion): your view comes from the headset's own tracking, and the head\n"
					"tracker quietly keeps it anchored to your lighthouse space. Smoothest view;\n"
					"losing sight of the tracker is harmless.\n\n"
					"OFF (Override): your view is built directly from the head tracker. Locks head\n"
					"and body to the exact same space, but you feel the tracker's jitter and\n"
					"line-of-sight losses.\n\n"
					"Switching mid-session is rate-limited rather than stepped, so the change\n"
					"eases in over a moment instead of jumping.");

				if (ImGui::Checkbox("Diagnostic log (CSV)", &fusionDiag))
				{
					vr::EVRSettingsError err = vr::VRSettingsError_None;
					vr::VRSettings()->SetBool("driver_spaceoverride", "fusionDiag", fusionDiag, &err);
				}
				ImGui::SetItemTooltip(
					"Writes a per-frame tracking-quality log to\n"
					"%LOCALAPPDATA%\\OpenVR-SpaceOverride\\logs\\fusion_diag_*.csv\n"
					"Only needed when investigating tracking quality. Fusion mode only.");

				ImGui::Spacing();
				ImGui::Separator();
				ImGui::Spacing();
			}

			ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.2f, 1.0f), "NOTE: All settings below require re-calibration to be applied");
			ImGui::Spacing();
			ImGui::Text("Tip: hover over the settings to see additional information.");
			ImGui::Spacing();

			float halfWidth = (ImGui::GetContentRegionAvail().x - style.ItemSpacing.x) * 0.5f;

			ImGui::BeginChild("##left_panel", ImVec2(halfWidth, 0), true);

			ImGui::Checkbox("Fallback to SLAM", &CalCtx.fallbackToSlam);
			ImGui::SetItemTooltip(
				"Temporarily uses HMD (SLAM) tracking if the headset tracker loses line of sight.");

			ImGui::Checkbox("Enable Angular Velocity", &CalCtx.enableAngularVelocity);
			ImGui::SetItemTooltip(
				"Enables angular velocity reporting, as it may cause issues with some devices, it is disabled by default.");

			ImGui::Checkbox("Relative Calibration", &CalCtx.continuousSync);
			ImGui::SetItemTooltip(
				"Continuously re-aligns SLAM-tracked devices (controllers etc.) to the calibrated space\n"
				"in the background. The headset's raw SLAM pose is compared against the tracker-driven\n"
				"pose to measure SLAM drift, and the correction is applied gradually and automatically.");

			ImGui::Checkbox("Discard Calibrated Offset", &CalCtx.enableNative);
			ImGui::SetItemTooltip(
				"Discards all SLAM tracking (even as fallback) and feeds only raw tracker data with the offset applied.\n"
				"Downside: a yaw orientation mismatch may occur. This only works in Local tracking space, where\n"
				"re-centering adjusts the yaw - Stage tracking space never re-centers yaw, so the mismatch cannot\n"
				"be corrected there.\n"
				"Will not work on all devices - tested only on Pico.");

			ImGui::EndChild();

			ImGui::SameLine();

			ImGui::BeginChild("##right_panel", ImVec2(0, 0), true);

			ImGui::Text("Prediction Time");
			ImGui::SameLine();
			ImGui::SliderFloat("##prediction_time", &CalCtx.predictionTime, 0.0f, 10.0f, "%.1f");
			ImGui::SetItemTooltip(
				"How many frames of prediction SteamVR applies to the tracker.\n"
				"Some wireless solutions may need more prediction to feel smooth.");

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

			ImGui::EndTabItem();
		}

		ImGui::EndTabBar();
	}

	ImGui::PopStyleColor();
	ImGui::End();
}
