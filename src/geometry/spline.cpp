
#include "../geometry/spline.h"

template<typename T> T Spline<T>::at(float time) const {

	// A4T1b: Evaluate a Catumull-Rom spline

	// Given a time, find the nearest positions & tangent values
	// defined by the control point map.

	// Transform them for use with cubic_unit_spline

	// Be wary of edge cases! What if time is before the first knot,
	// before the second knot, etc...
	float time_start = knots.begin()->first;
	float time_end = knots.rbegin()->first;

	if (knots.size() == 0) {
		return T();
	}
	if (knots.size() == 1) {
		return knots.begin()->second;
	}
	if (time <= time_start) {
		return knots.begin()->second;
	}
	if (time >= time_end) {
		return knots.rbegin()->second;
	}
	//Normalize time
	auto normalize_time = [](float utime, float start, float end) {
		float range = end - start;
		return (utime - start) / range;
	};

	auto k2 = knots.upper_bound(time);
	auto k1 = std::prev(k2);
	float t0, t1, t2, t3;
	T p0, p1, p2, p3;

	t1 = k1->first;
	t2 = k2->first;
	p1 = k1->second;
	p2 = k2->second;

	if (k1 == knots.begin()) {
		// k0 dne
		t0 = t1 - (t2 - t1); // mirror
		p0 = p1 - (p2 - p1);
	}
	else {
		t0 = std::prev(k1,1)->first;
		p0 = std::prev(k1,1)->second;
	}
	if (std::next(k2) == knots.end()) {
		// k3 dne
		t3 = t2 + (t2 - t1); // mirror
		p3 = p2 + (p2 - p1);
	}
	else {
		t3 = std::next(k2,1)->first;
		p3 = std::next(k2,1)->second;
	}

	// std::cout << "t0: " + std::to_string(t0) + " t1: " + std::to_string(t1) + " t2: " + std::to_string(t2) + " t3: " + std::to_string(t3) + "\n";
	/*t0 = normalize_time(t0);
	t1 = normalize_time(t1);
	t2 = normalize_time(t2);
	t3 = normalize_time(t3);*/
	T m0 = (p2 - p0) / (t2 - t0);
	T m1 = (p3 - p1) / (t3 - t1);
	// knots.upper_bound(time);
	/*if constexpr (std::is_same_v<T, Vec3>) {
		std::cout << "p1: (" + std::to_string(p1.x) + ", " + std::to_string(p1.y) + ", " + std::to_string(p1.z) + ")\n";
		std::cout << "p2: (" + std::to_string(p2.x) + ", " + std::to_string(p2.y) + ", " + std::to_string(p2.z) + ")\n";
		std::cout << "m0: (" + std::to_string(m0.x) + ", " + std::to_string(m0.y) + ", " + std::to_string(m0.z) + ")\n";
		std::cout << "m1: (" + std::to_string(m1.x) + ", " + std::to_string(m1.y) + ", " + std::to_string(m1.z) + ")\n";
	}*/

	float normal_time = normalize_time(time, t1, t2);
	return cubic_unit_spline(normal_time, p1, p2, m0, m1);
}

template<typename T>
T Spline<T>::cubic_unit_spline(float time, const T& position0, const T& position1,
                               const T& tangent0, const T& tangent1) {

	// A4T1a: Hermite Curve over the unit interval
	float t = time;
	float t2 = t * t;
	float t3 = t2 * t;

	// Given time in [0,1] compute the cubic spline coefficients and use them to compute
	// the interpolated value at time 'time' based on the positions & tangents

	float h00 = (2.0f * t3) - (3.0f * t2) + 1.0f;
	float h10 = t3 - (2.0f * t2) + t;
	float h01 = (-2.0f * t3) + (3.0f * t2);
	float h11 = t3 - t2;

	// Note that Spline is parameterized on type T, which allows us to create splines over
	// any type that supports the * and + operators.

	return T((h00 * position0) + (h10 * tangent0) + (h01 * position1) + (h11 * tangent1));
}

template class Spline<float>;
template class Spline<double>;
template class Spline<Vec4>;
template class Spline<Vec3>;
template class Spline<Vec2>;
template class Spline<Mat4>;
template class Spline<Spectrum>;
