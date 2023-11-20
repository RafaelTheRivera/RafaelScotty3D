

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

    PT::Trace ret;
    ret.origin = ray.point;
    ret.hit = false; 

    if (disc >= 0.0f) {
        float t1 = ((-b) + sqrt(disc)) / (2.0f * a);
        float t2 = ((-b) - sqrt(disc)) / (2.0f * a);

        auto in_bounds = [&ray](float t) {
            return (t > ray.dist_bounds.x && t < ray.dist_bounds.y);
        };

        bool inBoundsT1 = in_bounds(t1);
        bool inBoundsT2 = in_bounds(t2);

        if (inBoundsT1 && inBoundsT2) {
            ret.distance = std::min(t1, t2);
            ret.hit = true;
        } else if (inBoundsT1) {
            ret.distance = t1;
            ret.hit = true;
        } else if (inBoundsT2) {
            ret.distance = t2;
            ret.hit = true;
        }
    }

    if (ret.hit) {
        // Calculate the intersection point, normal, and uv coordinates
        ret.position = ray.point + (ret.distance * ray.dir); // Intersection point
        ret.normal = ret.position.unit();  // Surface normal at the intersection
        ret.uv = Sphere::uv(ret.normal);   // UV coordinates at the intersection
    }

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
