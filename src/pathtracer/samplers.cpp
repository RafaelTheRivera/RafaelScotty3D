
#include "samplers.h"
#include "../util/rand.h"

constexpr bool IMPORTANCE_SAMPLING = true;

namespace Samplers {

Vec2 Rect::sample(RNG &rng) const {
	//A3T1 - step 2 - supersampling

    // Return a point selected uniformly at random from the rectangle [0,size.x)x[0,size.y)
    // Useful function: rng.unit()
	float rand_x = rng.unit() * size.x;
	float rand_y = rng.unit() * size.y;

    return Vec2{rand_x,rand_y};
}

float Rect::pdf(Vec2 at) const {
	if (at.x < 0.0f || at.x > size.x || at.y < 0.0f || at.y > size.y) return 0.0f;
	return 1.0f / (size.x * size.y);
}

Vec3 Point::sample(RNG &rng) const {
	return point;
}

float Point::pdf(Vec3 at) const {
	return at == point ? 1.0f : 0.0f;
}

Vec3 Triangle::sample(RNG &rng) const {
	float u = std::sqrt(rng.unit());
	float v = rng.unit();
	float a = u * (1.0f - v);
	float b = u * v;
	return a * v0 + b * v1 + (1.0f - a - b) * v2;
}

float Triangle::pdf(Vec3 at) const {
	float a = 0.5f * cross(v1 - v0, v2 - v0).norm();
	float u = 0.5f * cross(at - v1, at - v2).norm() / a;
	float v = 0.5f * cross(at - v2, at - v0).norm() / a;
	float w = 1.0f - u - v;
	if (u < 0.0f || v < 0.0f || w < 0.0f) return 0.0f;
	if (u > 1.0f || v > 1.0f || w > 1.0f) return 0.0f;
	return 1.0f / a;
}

Vec3 Hemisphere::Uniform::sample(RNG &rng) const {

	float Xi1 = rng.unit();
	float Xi2 = rng.unit();

	float theta = std::acos(Xi1);
	float phi = 2.0f * PI_F * Xi2;

	float xs = std::sin(theta) * std::cos(phi);
	float ys = std::cos(theta);
	float zs = std::sin(theta) * std::sin(phi);

	return Vec3(xs, ys, zs);
}

float Hemisphere::Uniform::pdf(Vec3 dir) const {
	if (dir.y < 0.0f) return 0.0f;
	return 1.0f / (2.0f * PI_F);
}

Vec3 Hemisphere::Cosine::sample(RNG &rng) const {

	float phi = rng.unit() * 2.0f * PI_F;
	float cos_t = std::sqrt(rng.unit());

	float sin_t = std::sqrt(1 - cos_t * cos_t);
	float x = std::cos(phi) * sin_t;
	float z = std::sin(phi) * sin_t;
	float y = cos_t;

	return Vec3(x, y, z);
}

float Hemisphere::Cosine::pdf(Vec3 dir) const {
	if (dir.y < 0.0f) return 0.0f;
	return dir.y / PI_F;
}

Vec3 Sphere::Uniform::sample(RNG &rng) const {
	//A3T7 - sphere sampler

    // Generate a uniformly random point on the unit sphere.
    // Tip: start with Hemisphere::Uniform
	Hemisphere::Uniform sampler;
	Vec3 out = sampler.sample(rng);
	if (rng.coin_flip(0.5f)){
		out *= -1.0f;
	}
    return out;
}

float Sphere::Uniform::pdf(Vec3 dir) const {
	return 1.0f / (4.0f * PI_F);
}

Sphere::Image::Image(const HDR_Image& image) {
    //A3T7 - image sampler init

    // Set up importance sampling data structures for a spherical environment map image.
    // You may make use of the _pdf, _cdf, and total members, or create your own.

	
    const auto [_w, _h] = image.dimension();
    w = _w;
    h = _h;
	RNG rng;
	float totalPdf = 0.0f;
	_pdf.reserve(w*h);
	_cdf.reserve(w*h);
	//std::cout << "width (total): " + std::to_string(w) + "; height (total): " + std::to_string(h) + "\n";
	for (size_t height = 0; height < h; height++) {
		float angle = ((float)height / (float)h) * PI_F;
		float sintheta = sin(angle);
		for (size_t width = 0; width < w; width++){
			Spectrum pixel = image.at((uint32_t)width, (uint32_t)height);
			float luminance = pixel.luma();
			float pdf = luminance * sintheta;
			totalPdf += pdf;
			_pdf.emplace_back(pdf);
			/*if (rng.coin_flip(0.000001f)){
				std::cout << "luma = " + std::to_string(luminance) + "\n";
				std::cout << "sintheta = " + std::to_string(sintheta) + "\n";
				std::cout << "pdf = " + std::to_string(pdf) + "\n";
				std::cout << "reclaimed pdf = " + std::to_string(_pdf[height * w + width]) + "\n";
			}*/
		}
	}

	for (size_t height = 0; height < h; height++){
		for (size_t width = 0; width < w; width++) {
			_pdf[height*w + width] = _pdf[height*w + width] / totalPdf;
			float cdf = _pdf[height*w + width];
			if (height != 0 || width != 0) {
				cdf += _cdf[height*w + width - 1];
			}
			_cdf.emplace_back(cdf);
		}
	}
	//std::cout << "finished build\n";
}

Vec3 Sphere::Image::sample(RNG &rng) const {
	if(!IMPORTANCE_SAMPLING) {
		// Step 1: Uniform sampling
		// Declare a uniform sampler and return its sample
		Sphere::Uniform sampler;
		Vec3 out = sampler.sample(rng);
    	return out;
	} else {
		// Step 2: Importance sampling
		// Use your importance sampling data structure to generate a sample direction.
		// Tip: std::upper_bound
		float pixel = rng.unit(); 
		auto i = std::upper_bound(_cdf.begin(), _cdf.end(), pixel);
		size_t index = i - _cdf.begin();
		size_t height = index / w;
		size_t width = index % w;
		float theta = ((float)height / (float)h) * PI_F;
		theta = PI_F - theta; // easier to convert into theta from positive y for the coord calculation.
		float phi = ((float)width / (float)w) * 2.0f * PI_F;
		float x = (float)(sin(theta) * cos(phi));
		float y = (float)(cos(theta));
		float z = (float)(sin(theta) * sin(phi));
		/*if (x == -0.0f && y == -1.0f && z == -0.0f){
			std::cout << "lol";
		}*/
    	return Vec3{x,y,z};
	}
}

float Sphere::Image::pdf(Vec3 dir) const {
    if(!IMPORTANCE_SAMPLING) {
		// Step 1: Uniform sampling
		// Declare a uniform sampler and return its pdf
		Sphere::Uniform sampler;
		float out = sampler.pdf(dir);
    	return out;
	} else {
		// A3T7 - image sampler importance sampling pdf
		// What is the PDF of this distribution at a particular direction?
		
		//honestly couldn't figure out how to call this so just copied it locally
		auto uvsphere = [=](Vec3 dirin) {
			float u = std::atan2(dirin.z, dirin.x) / (2.0f * PI_F);
			if (u < 0.0f) u += 1.0f;
			float v = std::acos(-1.0f * std::clamp(dirin.y, -1.0f, 1.0f)) / PI_F;
			return Vec2{u, v};
		};
		Vec2 uv = uvsphere(dir);
		uint32_t width = (uint32_t)(uv.x * w);
		uint32_t height = (uint32_t)(uv.y * h);
		float theta = ((float)height / (float)h) * PI_F;
		
		float pdf = _pdf[height * w + width];
		float jacobian;
		if (theta == 0.0f || theta == PI_F) {
			jacobian = std::numeric_limits<float>::max();
		} else {
			jacobian = (w * h) / (2.0f * PI_F * PI_F * (float)sin(theta));
		}
		RNG rng;
		
		float out = pdf * jacobian;
		/*if (rng.coin_flip(0.00001f)){
			std::cout << "For input direction, x: " + std::to_string(dir.x) + ", y: " + std::to_string(dir.y) + ", z: " + std::to_string(dir.z) + "\n";
			// std::cout << "Output angles, theta: " + std::to_string(theta) + "; phi: " + std::to_string(phi) + "\n";
			std::cout << "Translating to width: " + std::to_string(width) + "; height: " + std::to_string(height) + " with index " + std::to_string(height * w + width) + "\n";
			std::cout << "With jacobian: " + std::to_string(jacobian) + ", pdf: " + std::to_string(pdf) + " and output " + std::to_string(out) + "\n";
			
		}*/
    	return out;
	}
}
/*float theta = (float)acos(dir.y);
		theta = PI_F - theta; // convert to low 0 high pi
		float phi;
		if (dir.x == 0.0f) {
			if (dir.z >= 0.0f){
				phi = PI_F / 2.0f;	
			} else {
				phi = PI_F / 2.f * 3.0f;
			}
		}
		else {
			phi = (float)atan(dir.z/dir.x);
		}
		if (phi < 0.0f){
			phi += PI_F;
		}
		if (dir.x < 0.0f) {
			phi += PI_F;
		}
		float widthf = (w * phi) / (2.0f * PI_F);
		float heightf = (theta * h) / PI_F;
		uint32_t width = (uint32_t)widthf;
		uint32_t height = (uint32_t)heightf;
		if (width >= w) {
			width = 0;
		}
		if (height >= h) {
			height = h-1;
		}
		if (width < 0 || width >= w || height < 0 || height >= h) {
			std::cout << "problem here, width: " + std::to_string(width) + "; height: " + std::to_string(height) + "\n";
			std::cout << "for input angles, theta: " + std::to_string(theta) + "; phi: " + std::to_string(phi) + "\n";
			std::cout << "on input direction, x: " + std::to_string(dir.x) + ", y: " + std::to_string(dir.y) + ", z: " + std::to_string(dir.z) + "\n";
		}*/
} // namespace Samplers
