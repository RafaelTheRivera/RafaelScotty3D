
#include "texture.h"

#include <iostream>

namespace Textures {


Spectrum sample_nearest(HDR_Image const &image, Vec2 uv) {
	//clamp texture coordinates, convert to [0,w]x[0,h] pixel space:
	float x = image.w * std::clamp(uv.x, 0.0f, 1.0f);
	float y = image.h * std::clamp(uv.y, 0.0f, 1.0f);

	//the pixel with the nearest center is the pixel that contains (x,y):
	int32_t ix = int32_t(std::floor(x));
	int32_t iy = int32_t(std::floor(y));

	//texture coordinates of (1,1) map to (w,h), and need to be reduced:
	ix = std::min(ix, int32_t(image.w) - 1);
	iy = std::min(iy, int32_t(image.h) - 1);

	return image.at(ix, iy);
}

Spectrum sample_bilinear(HDR_Image const& image, Vec2 uv) {
	// A1T6: sample_bilinear
	//TODO: implement bilinear sampling strategy on texture 'image'

	// std::cout << "UV: " + std::to_string(uv.x) + ", " + std::to_string(uv.y) + ")\n";

	float x = image.w * std::clamp(uv.x, 0.0f, 1.0f);
	float y = image.h * std::clamp(uv.y, 0.0f, 1.0f);
	//std::cout << "XY: " + std::to_string(x) + ", " + std::to_string(y) + ")\n";
	//std::cout << "WH: " + std::to_string(image.w) + ", " + std::to_string(image.h) + ")\n";

	float xp = std::round(x) - 0.5f;
	float yp = std::round(y) - 0.5f;
	float dx = x - xp;
	float dy = y - yp;
	//std::cout << "Dxy: " + std::to_string(dx) + ", " + std::to_string(dy) + ")\n";

	//std::cout << "Lookup over (" + std::to_string(xp) + ", " + std::to_string(yp) + ") \n";
	Spectrum t00, t01, t10, t11;
	//Spectrum t00 = image.at((int)xp, (int)yp);
	if (xp < 0.0f) {
		if (yp < 0.0f) {
			//std::cout << "LeftCorner\n";
			t11 = image.at((int)(xp + 1.0f), (int)(yp + 1.0f));
			return t11;
		}
		if (yp + 1 >= image.h) {
			t10 = image.at((int)(xp + 1.0f), (int)yp);
			return t10;
		}
		t10 = image.at((int)(xp + 1.0f), (int)yp);
		t11 = image.at((int)(xp + 1.0f), (int)(yp + 1.0f));
		return Spectrum((1 - dy) * t10[0] + dy * t11[0], (1 - dy) * t10[1] + dy * t11[1], (1 - dy) * t10[2] + dy * t11[2]);

	}
	if (xp + 1 >= image.w) {
		if (yp < 0.0f) {
			t01 = image.at((int)xp, (int)(yp + 1.0f));
			return t01;
		}
		if (yp + 1 >= image.h) {
			t00 = image.at((int)xp, (int)yp);
			return t00;
		}
		t00 = image.at((int)xp, (int)yp);
		t01 = image.at((int)xp, (int)(yp + 1.0f));
		return Spectrum((1 - dx) * t00[0] + dx * t01[0], (1 - dx) * t00[1] + dx * t01[1], (1 - dx) * t00[2] + dx * t01[2]);
	}
	if (yp < 0.0f) {
		t01 = image.at((int)xp, (int)(yp + 1.0f));
		t11 = image.at((int)(xp + 1.0f), (int)yp+1);
		return Spectrum((1 - dx) * t01[0] + dx * t11[0], (1 - dx) * t01[1] + dx * t11[1], (1 - dx) * t01[2] + dx * t11[2]);
	}
	if (yp + 1 >= image.h) {
		//std::cout << "LEdge\n";
		t00 = image.at((int)xp, (int)yp);
		t10 = image.at((int)(xp + 1.0f), (int)yp);
		return Spectrum((1 - dx) * t00[0] + dx * t10[0], (1 - dx) * t00[1] + dx * t10[1], (1 - dx) * t00[2] + dx * t10[2]); 
	}
	//std::cout << "Centered\n";
	t00 = image.at((int)xp, (int)yp);
	t10 = image.at((int)(xp + 1.0f), (int)yp);
	t01 = image.at((int)xp,	 (int)(yp + 1.0f));
	t11 = image.at((int)(xp + 1.0f), (int)(yp + 1.0f));

	//std::cout << "t00 (" + std::to_string(t00[0]) + ", " + std::to_string(t00[1]) + ", " + std::to_string(t00[2]) + ") \n";
	//std::cout << "t10 (" + std::to_string(t10[0]) + ", " + std::to_string(t10[1]) + ", " + std::to_string(t10[2]) + ") \n";
	//std::cout << "t01 (" + std::to_string(t01[0]) + ", " + std::to_string(t01[1]) + ", " + std::to_string(t01[2]) + ") \n";
	//std::cout << "t11 (" + std::to_string(t11[0]) + ", " + std::to_string(t11[1]) + ", " + std::to_string(t11[2]) + ") \n";


	Spectrum tx = Spectrum((1 - dx) * t00[0] + dx * t10[0], (1 - dx) * t00[1] + dx * t10[1], (1 - dx) * t00[2] + dx * t10[2]);
	Spectrum ty = Spectrum((1 - dx) * t01[0] + dx * t11[0], (1 - dx) * t01[1] + dx * t11[1], (1 - dx) * t01[2] + dx * t11[2]);
	Spectrum t =  Spectrum((1 - dy) * tx[0]  + dy * ty[0],  (1 - dy) * tx[1]  + dy * ty[1],  (1 - dy) * tx[2]  + dy * ty[2]);

	//std::cout << "Yielded (" + std::to_string(t[0]) + ", " + std::to_string(t[1]) + ", " + std::to_string(t[2]) + ") \n";
	return t;
	// return sample_nearest(image, uv); //placeholder so image doesn't look blank
}


Spectrum sample_trilinear(HDR_Image const &base, std::vector< HDR_Image > const &levels, Vec2 uv, float lod) {
	// A1T6: sample_trilinear
	//TODO: implement trilinear sampling strategy on using mip-map 'levels'
	// std::cout << "UV: " + std::to_string(uv.x) + ", " + std::to_string(uv.y) + ")\n";
	// lod = lod / 2;
	int dp = (int)floor(lod);
	float dd = lod - dp;

	/*if (uv.x < 0.0001f) {
		std::cout << "Raw lod: " + std::to_string(lod) + +" Processed lod: " + std::to_string(dp) + "\n";
		std::cout << "UV: " + std::to_string(uv.x) + ", " + std::to_string(uv.y) + ")\n";
		std::cout << "Current Level " + std::to_string(dp) + "; Height = " + std::to_string(levels[dp].h) + "; Width = " + std::to_string(levels[dp].h) + "\n";
	}*/
	Spectrum td, td1;
	if (dp == 0) {
		td = sample_bilinear(base, uv);
	}
	else {
		td = sample_bilinear(levels[dp-1], uv);
	}

	// std::cout << "Raw lod: " + std::to_string(lod) + + " Processed lod: " + std::to_string(dp) + "\n";
	// std::cout << "Testing Level " + std::to_string(dp) + "; Height = " + std::to_string(levels[dp].h) + "; Width = " + std::to_string(levels[dp].h) + "\n";
	
	if (levels[dp].h == 0 || levels[dp].w == 0) {
		return td;
	}
	// std::cout << "Testing Level " + std::to_string(dp+1) + "; Height = " + std::to_string(levels[dp + 1].h) + "; Width = " + std::to_string(levels[dp + 1].h) + "\n";
	td1 = sample_bilinear(levels[dp], uv);
	return Spectrum((1 - dd) * td[0] + dd * td1[0], (1 - dd) * td[1] + dd * td1[1], (1 - dd) * td[2] + dd * td1[2]);
	// return sample_nearest(base, uv); //placeholder so image doesn't look blank
}

/*
 * generate_mipmap- generate mipmap levels from a base image.
 *  base: the base image
 *  levels: pointer to vector of levels to fill (must not be null)
 *
 * generates a stack of levels [1,n] of sizes w_i, h_i, where:
 *   w_i = max(1, floor(w_{i-1})/2)
 *   h_i = max(1, floor(h_{i-1})/2)
 *  with:
 *   w_0 = base.w
 *   h_0 = base.h
 *  and n is the smalles n such that w_n = h_n = 1
 *
 * each level should be calculated by downsampling a blurred version
 * of the previous level to remove high-frequency detail.
 *
 */
void generate_mipmap(HDR_Image const &base, std::vector< HDR_Image > *levels_) {
	assert(levels_);
	auto &levels = *levels_;


	{ // allocate sublevels sufficient to scale base image all the way to 1x1:
		int32_t num_levels = static_cast<int32_t>(std::log2(std::max(base.w, base.h)));
		assert(num_levels >= 0);

		levels.clear();
		levels.reserve(num_levels);

		uint32_t width = base.w;
		uint32_t height = base.h;
		for (int32_t i = 0; i < num_levels; ++i) {
			assert(!(width == 1 && height == 1)); //would have stopped before this if num_levels was computed correctly

			width = std::max(1u, width / 2u);
			height = std::max(1u, height / 2u);

			levels.emplace_back(width, height);
		}
		assert(width == 1 && height == 1);
		assert(levels.size() == uint32_t(num_levels));
	}

	
	//now fill in the levels using a helper:
	//downsample:
	// fill in dst to represent the low-frequency component of src
	auto downsample = [](HDR_Image const &src, HDR_Image &dst) {
		//dst is half the size of src in each dimension:
		assert(std::max(1u, src.w / 2u) == dst.w);
		assert(std::max(1u, src.h / 2u) == dst.h);
		
		// A1T6: generate
		//TODO: Write code to fill the levels of the mipmap hierarchy by downsampling
		//std::cout << "Width: " + std::to_string(dst.w) + " Height: " + std::to_string(dst.h) + "\n";
			for (uint32_t ri = 0; ri < dst.h; ri++) {
				for (uint32_t ci = 0; ci < dst.w; ci++) {
					if (((2 * ci + 1) >= src.w) && ((2*ri + 1) >= src.h)) {
						Spectrum p1 = src.at(2 * ci, 2 * ri);
						dst.at(ci, ri) = p1;
					}
					else if (2 * ci + 1 >= src.w) {
						Spectrum p1 = src.at(2 * ci, 2 * ri);
						Spectrum p3 = src.at(2 * ci, 2 * ri + 1);
						dst.at(ci, ri) = Spectrum((p1.r + p3.r) / 2.0f, (p1.g + p3.g) / 2.0f, (p1.b + p3.b) / 2.0f);
					}
					else if (2 * ri + 1 >= src.h) {
						Spectrum p1 = src.at(2 * ci, 2 * ri);
						Spectrum p2 = src.at(2 * ci + 1, 2 * ri);
						dst.at(ci, ri) = Spectrum((p1.r + p2.r) / 2.0f, (p1.g + p2.g) / 2.0f, (p1.b + p2.b) / 2.0f);
					}
					else {
						Spectrum p1 = src.at(2 * ci, 2 * ri);
						Spectrum p2 = src.at(2 * ci + 1, 2 * ri);
						Spectrum p3 = src.at(2 * ci, 2 * ri + 1);
						Spectrum p4 = src.at(2 * ci + 1, 2 * ri + 1);
						dst.at(ci, ri) = Spectrum((p1.r + p2.r + p3.r + p4.r) / 4.0f, (p1.g + p2.g + p3.g + p4.g) / 4.0f, (p1.b + p2.b + p3.b + p4.b) / 4.0f);
					}
					Spectrum pout = dst.at(ci, ri);
					//std::cout << std::to_string(pout.r) + ", " + std::to_string(pout.g) + ", " + std::to_string(pout.b) + ", ";
				}
			}
			//std::cout << "\n END \n";
		//Be aware that the alignment of the samples in dst and src will be different depending on whether the image is even or odd.

	};

	std::cout << "Regenerating mipmap (" << levels.size() << " levels): [" << base.w << "x" << base.h << "]";
	std::cout.flush();
	for (uint32_t i = 0; i < levels.size(); ++i) {
		HDR_Image const &src = (i == 0 ? base : levels[i-1]);
		HDR_Image &dst = levels[i];
		std::cout << " -> [" << dst.w << "x" << dst.h << "]"; std::cout.flush();

		downsample(src, dst);
	}
	std::cout << std::endl;
	
}

Image::Image(Sampler sampler_, HDR_Image const &image_) {
	sampler = sampler_;
	image = image_.copy();
	update_mipmap();
}

Spectrum Image::evaluate(Vec2 uv, float lod) const {
	if (sampler == Sampler::nearest) {
		return sample_nearest(image, uv);
	} else if (sampler == Sampler::bilinear) {
		return sample_bilinear(image, uv);
	} else {
		return sample_trilinear(image, levels, uv, lod);
	}
}

void Image::update_mipmap() {
	if (sampler == Sampler::trilinear) {
		generate_mipmap(image, &levels);
	} else {
		levels.clear();
	}
}

GL::Tex2D Image::to_gl() const {
	return image.to_gl(1.0f);
}

void Image::make_valid() {
	update_mipmap();
}

Spectrum Constant::evaluate(Vec2 uv, float lod) const {
	return color * scale;
}

} // namespace Textures
bool operator!=(const Textures::Constant& a, const Textures::Constant& b) {
	return a.color != b.color || a.scale != b.scale;
}

bool operator!=(const Textures::Image& a, const Textures::Image& b) {
	return a.image != b.image;
}

bool operator!=(const Texture& a, const Texture& b) {
	if (a.texture.index() != b.texture.index()) return false;
	return std::visit(
		[&](const auto& data) { return data != std::get<std::decay_t<decltype(data)>>(b.texture); },
		a.texture);
}
