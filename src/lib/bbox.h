
#pragma once

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <iostream>
#include <ostream>
#include <vector>

#include "mat4.h"
#include "ray.h"
#include "vec2.h"
#include "vec3.h"

struct BBox {

	/// Default min is max float value, default max is negative max float value
	BBox() : min(FLT_MAX), max(-FLT_MAX) {
	}
	/// Set minimum and maximum extent
	explicit BBox(Vec3 min, Vec3 max) : min(min), max(max) {
	}

	BBox(const BBox&) = default;
	BBox& operator=(const BBox&) = default;
	~BBox() = default;

	/// Rest min to max float, max to negative max float
	void reset() {
		min = Vec3(FLT_MAX);
		max = Vec3(-FLT_MAX);
	}

	/// Expand bounding box to include point
	void enclose(Vec3 point) {
		min = hmin(min, point);
		max = hmax(max, point);
	}
	void enclose(BBox box) {
		min = hmin(min, box.min);
		max = hmax(max, box.max);
	}

	/// Get center point of box
	Vec3 center() const {
		return (min + max) * 0.5f;
	}

	// Check whether box has no volume
	bool empty() const {
		return min.x > max.x || min.y > max.y || min.z > max.z;
	}

	/// Get surface area of the box
	float surface_area() const {
		if (empty()) return 0.0f;
		Vec3 extent = max - min;
		return 2.0f * (extent.x * extent.z + extent.x * extent.y + extent.y * extent.z);
	}

	/// Transform box by a matrix
	BBox& transform(const Mat4& trans) {
		Vec3 amin = min, amax = max;
		min = max = trans[3].xyz();
		for (uint32_t i = 0; i < 3; i++) {
			for (uint32_t j = 0; j < 3; j++) {
				float a = trans[j][i] * amin[j];
				float b = trans[j][i] * amax[j];
				if (a < b) {
					min[i] += a;
					max[i] += b;
				} else {
					min[i] += b;
					max[i] += a;
				}
			}
		}
		return *this;
	}

	bool hit(const Ray& ray, Vec2& times) const {
		//A3T3 - bbox hit

		// Implement ray - bounding box intersection test
		// If the ray intersected the bounding box within the range given by
		// [times.x,times.y], update times with the new intersection times.
		float tmin, tmax, tymin, tymax, tzmin, tzmax;

		Vec3 invdir = 1.0f / ray.dir;
		
		/*std::cout << "ray: (" + std::to_string(ray.point.x) + ", " + std::to_string(ray.point.y) + ", " + std::to_string(ray.point.z) + ")\n";
		std::cout << "dir: (" + std::to_string(ray.dir.x) + ", " + std::to_string(ray.dir.y) + ", " + std::to_string(ray.dir.z) + ")\n";
		std::cout << "min: (" + std::to_string(BBox::min.x) + ", " + std::to_string(BBox::min.y) + ", " + std::to_string(BBox::min.z) + ")\n";
		std::cout << "max: (" + std::to_string(BBox::max.x) + ", " + std::to_string(BBox::max.y) + ", " + std::to_string(BBox::max.z) + ")\n";
		std::cout << "times: (" + std::to_string(times.x) + ", " + std::to_string(times.y) + ")\n";

		std::cout << "invdir: (" + std::to_string(invdir.x) + ", " + std::to_string(invdir.y) + ", " + std::to_string(invdir.z) + ")\n";
		*/

		if (invdir.x >= 0) {
			tmin = (min.x - ray.point.x) * invdir.x;
			tmax = (max.x - ray.point.x) * invdir.x;
		}
		else {
			tmin = (max.x - ray.point.x) * invdir.x;
			tmax = (min.x - ray.point.x) * invdir.x;
		}

		if (invdir.y >= 0) {
			tymin = (min.y - ray.point.y) * invdir.y;
			tymax = (max.y - ray.point.y) * invdir.y;
		}
		else {
			tymin = (max.y - ray.point.y) * invdir.y;
			tymax = (min.y - ray.point.y) * invdir.y;
		}
		/*
		tmin =  (BBox::min.x - ray.point.x) * invdir.x;
		// (-0.48 - 1.693115) * -3.754548 = 8.159064577
		tmax =  (BBox::max.x - ray.point.x) * invdir.x;
		// (0.48 - 1.693115) * -3.754548 = 4.554698497
		tymin = (BBox::min.y - ray.point.y) * invdir.y;
		// (-0.48 - 1.574) * -4.224963 = 8.678074002
		tymax = (BBox::max.y - ray.point.y) * invdir.y;
		// (0.48 - 1.574) * -4.224963 = 4.622109522*/

		if ((tmin > tymax) || (tymin > tmax)) {
			// std::cout << "fail 1 \n";
			return false;
		}
		if (tymin > tmin) {
			tmin = tymin;
		}
		if (tymax < tmax) {
			tmax = tymax;
		}

		/*tzmin = (BBox::min.z - ray.point.z) * invdir.z;
		tzmax = (BBox::max.z - ray.point.z) * invdir.z;*/
		if (invdir.z >= 0) {
			tzmin = (min.z - ray.point.z) * invdir.z;
			tzmax = (max.z - ray.point.z) * invdir.z;
		}
		else {
			tzmin = (max.z - ray.point.z) * invdir.z;
			tzmax = (min.z - ray.point.z) * invdir.z;
		}

		if ((tmin > tzmax) || (tzmin > tmax)) {
			// std::cout << "fail 2 \n";
			return false;
		}

		if (tzmin > tmin) {
			tmin = tzmin;
		}

		if (tzmax < tmax) {
			tmax = tzmax;
		}

		
		if (tmax < times.x || tmin > times.y) {
			// std::cout << "fail 3 \n";
			return false;
		}
		if (tmax > times.y) {
			tmax = times.y;
		}
		if (tmin < times.x) {
			tmin = times.x;
		}
		// std::cout << "pass with " + std::to_string(tmin) + " - " + std::to_string(tmax) + "\n";

		times = Vec2(tmin, tmax);
		// std::cout << "times new: (" + std::to_string(times.x) + ", " + std::to_string(times.y) + ")\n";

		return true;
	}

	/// Get the eight corner points of the bounding box
	std::vector<Vec3> corners() const {
		std::vector<Vec3> ret(8);
		ret[0] = Vec3(min.x, min.y, min.z);
		ret[1] = Vec3(max.x, min.y, min.z);
		ret[2] = Vec3(min.x, max.y, min.z);
		ret[3] = Vec3(min.x, min.y, max.z);
		ret[4] = Vec3(max.x, max.y, min.z);
		ret[5] = Vec3(min.x, max.y, max.z);
		ret[6] = Vec3(max.x, min.y, max.z);
		ret[7] = Vec3(max.x, max.y, max.z);
		return ret;
	}

	/// Given a screen transformation (projection), calculate screen-space ([-1,1]x[-1,1])
	/// bounds that will always contain the bounding box on screen
	void screen_rect(const Mat4& transform, Vec2& min_out, Vec2& max_out) const {

		min_out = Vec2(FLT_MAX);
		max_out = Vec2(-FLT_MAX);
		auto c = corners();
		bool partially_behind = false, all_behind = true;
		for (auto& v : c) {
			Vec3 p = transform * v;
			if (p.z < 0) {
				partially_behind = true;
			} else {
				all_behind = false;
			}
			min_out = hmin(min_out, Vec2(p.x, p.y));
			max_out = hmax(max_out, Vec2(p.x, p.y));
		}

		if (partially_behind && !all_behind) {
			min_out = Vec2(-1.0f, -1.0f);
			max_out = Vec2(1.0f, 1.0f);
		} else if (all_behind) {
			min_out = Vec2(0.0f, 0.0f);
			max_out = Vec2(0.0f, 0.0f);
		}
	}

	Vec3 min, max;
};

inline std::ostream& operator<<(std::ostream& out, BBox b) {
	out << "BBox{" << b.min << "," << b.max << "}";
	return out;
}
