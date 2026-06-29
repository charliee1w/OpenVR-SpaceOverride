// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include <cstddef>
#include <vector>

namespace calibration
{
	struct AngVelSeries
	{
		std::vector<double> tracker;
		std::vector<double> slam;
		std::vector<double> trackerLin;
		std::vector<double> slamLin;
	};

	struct LagEstimate
	{
		bool valid = false;
		int lagFrames = 0;
		double correlation = 0.0;
	};

	// Cross-correlate tracker vs SLAM motion-speed magnitudes (angular + optional linear).
	// Positive lagFrames means SLAM leads tracker by that many samples.
	LagEstimate estimatePredictionLagFrames(
		const AngVelSeries& series,
		std::size_t minSamples = 45,
		int maxLagFrames = 3);

	double correlationAtLag(const AngVelSeries& series, int lagFrames);
}