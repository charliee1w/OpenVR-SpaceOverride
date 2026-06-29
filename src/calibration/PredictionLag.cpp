// SPDX-License-Identifier: AGPL-3.0-only

#include "calibration/PredictionLag.h"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace calibration
{
	static double pearsonCorrelation(const double* a, const double* b, std::size_t count)
	{
		if (count < 2)
			return 0.0;

		double meanA = std::accumulate(a, a + count, 0.0) / (double)count;
		double meanB = std::accumulate(b, b + count, 0.0) / (double)count;

		double cov = 0.0;
		double varA = 0.0;
		double varB = 0.0;

		for (std::size_t i = 0; i < count; ++i)
		{
			double da = a[i] - meanA;
			double db = b[i] - meanB;
			cov += da * db;
			varA += da * da;
			varB += db * db;
		}

		if (varA <= 1e-12 || varB <= 1e-12)
			return 0.0;

		return cov / std::sqrt(varA * varB);
	}

	static double correlationAtLagForSeries(
		const std::vector<double>& tracker,
		const std::vector<double>& slam,
		int lagFrames)
	{
		if (lagFrames < 0)
			return 0.0;

		const std::size_t n = tracker.size();
		if (slam.size() != n || n < 2)
			return 0.0;

		const std::size_t count = n - (std::size_t)lagFrames;
		if (count < 2)
			return 0.0;

		return pearsonCorrelation(tracker.data(), slam.data() + lagFrames, count);
	}

	double correlationAtLag(const AngVelSeries& series, int lagFrames)
	{
		return correlationAtLagForSeries(series.tracker, series.slam, lagFrames);
	}

	LagEstimate estimatePredictionLagFrames(
		const AngVelSeries& series,
		std::size_t minSamples,
		int maxLagFrames)
	{
		LagEstimate result;

		if (series.tracker.size() != series.slam.size() || series.tracker.size() < minSamples)
			return result;

		maxLagFrames = std::clamp(maxLagFrames, 0, 3);

		double bestCorr = -2.0;
		int bestLag = 0;

		const bool hasLinear = series.trackerLin.size() == series.tracker.size()
			&& series.slamLin.size() == series.tracker.size()
			&& !series.trackerLin.empty();

		for (int lag = 0; lag <= maxLagFrames; ++lag)
		{
			double corr = correlationAtLagForSeries(series.tracker, series.slam, lag);
			if (hasLinear)
			{
				const double linCorr = correlationAtLagForSeries(series.trackerLin, series.slamLin, lag);
				corr = std::max(corr, linCorr);
			}

			if (corr > bestCorr)
			{
				bestCorr = corr;
				bestLag = lag;
			}
		}

		if (bestCorr < 0.2)
			return result;

		result.valid = true;
		result.lagFrames = bestLag;
		result.correlation = bestCorr;
		return result;
	}
}