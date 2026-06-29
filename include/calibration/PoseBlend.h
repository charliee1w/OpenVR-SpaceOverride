// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include <Dense>

namespace calibration
{
	Eigen::Quaterniond slerpQuaternion(const Eigen::Quaterniond& from, const Eigen::Quaterniond& to, double t);
	Eigen::Vector3d lerpVector3(const Eigen::Vector3d& from, const Eigen::Vector3d& to, double t);
}