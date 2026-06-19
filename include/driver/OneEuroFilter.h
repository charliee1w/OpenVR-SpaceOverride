// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <openvr_driver.h>
#include <cmath>

namespace oneeuro {

	constexpr double PI = 3.14159265358979323846;

	struct Params {
		double minCutoff = 1.0;
		double beta = 0.0;
		double dCutoff = 1.0;
	};

	inline double alpha(double cutoff, double dt) {
		double tau = 1.0 / (2.0 * PI * cutoff);
		return 1.0 / (1.0 + tau / dt);
	}

	struct Vec3 {
		Params params;
		bool initialized = false;
		vr::HmdVector3d_t value = { 0, 0, 0 };
		vr::HmdVector3d_t deriv = { 0, 0, 0 };

		void reset() { initialized = false; }

		vr::HmdVector3d_t filter(const vr::HmdVector3d_t& x, double dt) {
			if (!initialized) {
				value = x;
				deriv = { 0, 0, 0 };
				initialized = true;
				return value;
			}

			double aDeriv = alpha(params.dCutoff, dt);
			for (int i = 0; i < 3; i++) {
				double rawDeriv = (x.v[i] - value.v[i]) / dt;
				deriv.v[i] = aDeriv * rawDeriv + (1.0 - aDeriv) * deriv.v[i];
			}

			double speed = std::sqrt(deriv.v[0] * deriv.v[0] + deriv.v[1] * deriv.v[1] + deriv.v[2] * deriv.v[2]);
			double aValue = alpha(params.minCutoff + params.beta * speed, dt);
			for (int i = 0; i < 3; i++)
				value.v[i] = aValue * x.v[i] + (1.0 - aValue) * value.v[i];

			return value;
		}
	};

	struct Quat {
		Params params;
		bool initialized = false;
		vr::HmdQuaternion_t value = { 1, 0, 0, 0 };
		double speed = 0.0;

		void reset() { initialized = false; }

		vr::HmdQuaternion_t filter(vr::HmdQuaternion_t x, double dt) {
			x = normalize(x);
			if (!initialized) {
				value = x;
				speed = 0.0;
				initialized = true;
				return value;
			}

			double dot = value.w * x.w + value.x * x.x + value.y * x.y + value.z * x.z;
			if (dot < 0.0) {
				dot = -dot;
				x.w = -x.w; x.x = -x.x; x.y = -x.y; x.z = -x.z;
			}
			if (dot > 1.0) dot = 1.0;

			double rawSpeed = (2.0 * std::acos(dot)) / dt;
			double aDeriv = alpha(params.dCutoff, dt);
			speed = aDeriv * rawSpeed + (1.0 - aDeriv) * speed;

			double aValue = alpha(params.minCutoff + params.beta * speed, dt);
			value = slerp(value, x, aValue);
			return value;
		}

	private:
		static vr::HmdQuaternion_t normalize(vr::HmdQuaternion_t q) {
			double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
			if (n > 0.0) { q.w /= n; q.x /= n; q.y /= n; q.z /= n; }
			return q;
		}

		static vr::HmdQuaternion_t slerp(const vr::HmdQuaternion_t& a, vr::HmdQuaternion_t b, double t) {
			double dot = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
			if (dot < 0.0) {
				dot = -dot;
				b.w = -b.w; b.x = -b.x; b.y = -b.y; b.z = -b.z;
			}

			double wa, wb;
			if (dot > 0.9995) {
				wa = 1.0 - t;
				wb = t;
			}
			else {
				double theta = std::acos(dot);
				double s = std::sin(theta);
				wa = std::sin((1.0 - t) * theta) / s;
				wb = std::sin(t * theta) / s;
			}

			return normalize({
				wa * a.w + wb * b.w,
				wa * a.x + wb * b.x,
				wa * a.y + wb * b.y,
				wa * a.z + wb * b.z
			});
		}
	};
}
