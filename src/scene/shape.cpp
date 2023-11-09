

#include <iostream>
#include "shape.h"
#include "../geometry/util.h"

namespace Shapes {

Vec2 Sphere::uv(Vec3 dir) {
	float u = std::atan2(dir.z, dir.x) / (2.0f * PI_F);
	if (u < 0.0f) u += 1.0f;
	float v = std::acos(-1.0f * std::clamp(dir.y, -1.0f, 1.0f)) / PI_F;
	return Vec2{u, v};
}

BBox Sphere::bbox() const {
	BBox box;
	box.enclose(Vec3(-radius));
	box.enclose(Vec3(radius));
	return box;
}

PT::Trace Sphere::hit(Ray ray) const {
	//A3T2 - sphere hit

    // TODO (PathTracer): Task 2
    // Intersect this ray with a sphere of radius Sphere::radius centered at the origin.

    // If the ray intersects the sphere twice, ret should
    // represent the first intersection, but remember to respect
    // ray.dist_bounds! For example, if there are two intersections,
    // but only the _later_ one is within ray.dist_bounds, you should
    // return that one!
	float a = ray.dir.norm() * ray.dir.norm();
	float b = 2 * dot(ray.point, ray.dir);
	float c = (ray.point.norm() * ray.point.norm()) - (Sphere::radius * Sphere::radius);
	float disc = (b * b) - (4.0f * a * c);
	float t1 = 0.0f;
	float t2 = 0.0f;

	auto in_bounds = [&ray](float t) {
		return (t > ray.dist_bounds.x && t < ray.dist_bounds.y);
	};
	// std::cout << "discriminant = " + std::to_string(disc) + "\n";
    PT::Trace ret;
    ret.origin = ray.point;
	if (disc > 0.0f) {
		t1 = ((-1.0f * b) + sqrt(disc)) / (2.0f * a); // further
		t2 = ((-1.0f * b) - sqrt(disc)) / (2.0f * a); // closer
		ret.hit = true;
	}
	else {
		ret.hit = false;
	}
	if (in_bounds(t1) && !in_bounds(t2)) {
		ret.distance = t1;
	}
	else {
		ret.distance = t2;
	}   // at what distance did the intersection occur?

    ret.position = ray.point + (ret.distance * ray.dir); // where was the intersection?
    ret.normal = ret.position.unit();   // what was the surface normal at the intersection?
	ret.uv = Sphere::uv(ret.normal); 	   // what was the uv coordinates at the intersection? (you may find Sphere::uv to be useful)
    return ret;
}

Vec3 Sphere::sample(RNG &rng, Vec3 from) const {
	die("Sampling sphere area lights is not implemented yet.");
}

float Sphere::pdf(Ray ray, Mat4 pdf_T, Mat4 pdf_iT) const {
	die("Sampling sphere area lights is not implemented yet.");
}

Indexed_Mesh Sphere::to_mesh() const {
	return Util::closed_sphere_mesh(radius, 2);
}

} // namespace Shapes

bool operator!=(const Shapes::Sphere& a, const Shapes::Sphere& b) {
	return a.radius != b.radius;
}

bool operator!=(const Shape& a, const Shape& b) {
	if (a.shape.index() != b.shape.index()) return false;
	return std::visit(
		[&](const auto& shape) {
			return shape != std::get<std::decay_t<decltype(shape)>>(b.shape);
		},
		a.shape);
}
