// SPDX-License-Identifier: AGPL-3.0-only

#include "calibration/PoseBlend.h"

#include <algorithm>

namespace calibration
{
	Eigen::Quaterniond slerpQuaternion(const Eigen::Quaterniond& from, const Eigen::Quaterniond& to, double t)
	{
		double clamped = std::clamp(t, 0.0, 1.0);
		Eigen::Quaterniond a = from.normalized();
		Eigen::Quaterniond b = to.normalized();
		return a.slerp(clamped, b);
	}

	Eigen::Vector3d lerpVector3(const Eigen::Vector3d& from, const Eigen::Vector3d& to, double t)
	{
		double clamped = std::clamp(t, 0.0, 1.0);
		return from + (to - from) * clamped;
	}
}