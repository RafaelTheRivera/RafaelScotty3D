// clang-format off
#include "pipeline.h"

#include <iostream>

#include "../lib/log.h"
#include "../lib/mathlib.h"
#include "framebuffer.h"
#include "sample_pattern.h"

template<PrimitiveType primitive_type, class Program, uint32_t flags>
void Pipeline<primitive_type, Program, flags>::run(std::vector<Vertex> const& vertices,
                                                   typename Program::Parameters const& parameters,
                                                   Framebuffer* framebuffer_) {
	// Framebuffer must be non-null:
	assert(framebuffer_);
	auto& framebuffer = *framebuffer_;
	

	// A1T7: sample loop
	// TODO: update this function to rasterize to *all* sample locations in the framebuffer.
	//  	 This will probably involve inserting a loop of the form:
	// 		 	std::vector< Vec3 > const &samples = framebuffer.sample_pattern.centers_and_weights;
	//      	for (uint32_t s = 0; s < samples.size(); ++s) { ... }
	//   	 around some subset of the code.
	// 		 You will also need to transform the input and output of the rasterize_* functions to
	// 	     account for the fact they deal with pixels centered at (0.5,0.5).

	std::vector<ShadedVertex> shaded_vertices;
	shaded_vertices.reserve(vertices.size());

	//--------------------------
	// shade vertices:
	for (auto const& v : vertices) {
		ShadedVertex sv;
		Program::shade_vertex(parameters, v.attributes, &sv.clip_position, &sv.attributes);
		shaded_vertices.emplace_back(sv);
	}

	//--------------------------
	// assemble + clip + homogeneous divide vertices:
	std::vector<ClippedVertex> clipped_vertices;

	// reserve some space to avoid reallocations later:
	if constexpr (primitive_type == PrimitiveType::Lines) {
		// clipping lines can never produce more than one vertex per input vertex:
		clipped_vertices.reserve(shaded_vertices.size());
	} else if constexpr (primitive_type == PrimitiveType::Triangles) {
		// clipping triangles can produce up to 8 vertices per input vertex:
		clipped_vertices.reserve(shaded_vertices.size() * 8);
	}
	// clang-format off

	//coefficients to map from clip coordinates to framebuffer (i.e., "viewport") coordinates:
	//x: [-1,1] -> [0,width]
	//y: [-1,1] -> [0,height]
	//z: [-1,1] -> [0,1] (OpenGL-style depth range)
	Vec3 const clip_to_fb_scale = Vec3{
		framebuffer.width / 2.0f,
		framebuffer.height / 2.0f,
		0.5f
	};
	Vec3 const clip_to_fb_offset = Vec3{
		0.5f * framebuffer.width,
		0.5f * framebuffer.height,
		0.5f
	};

	// helper used to put output of clipping functions into clipped_vertices:
	auto emit_vertex = [&](ShadedVertex const& sv) {
		ClippedVertex cv;
		float inv_w = 1.0f / sv.clip_position.w;
		cv.fb_position = clip_to_fb_scale * inv_w * sv.clip_position.xyz() + clip_to_fb_offset;
		cv.inv_w = inv_w;
		cv.attributes = sv.attributes;
		clipped_vertices.emplace_back(cv);
	};

	// actually do clipping:
	if constexpr (primitive_type == PrimitiveType::Lines) {
		for (uint32_t i = 0; i + 1 < shaded_vertices.size(); i += 2) {
			clip_line(shaded_vertices[i], shaded_vertices[i + 1], emit_vertex);
		}
	} else if constexpr (primitive_type == PrimitiveType::Triangles) {
		for (uint32_t i = 0; i + 2 < shaded_vertices.size(); i += 3) {
			clip_triangle(shaded_vertices[i], shaded_vertices[i + 1], shaded_vertices[i + 2], emit_vertex);
		}
	} else {
		static_assert(primitive_type == PrimitiveType::Lines, "Unsupported primitive type.");
	}

	//--------------------------
	// rasterize primitives:

	std::vector<Fragment> fragments;

	// helper used to put output of rasterization functions into fragments:
	auto emit_fragment = [&](Fragment const& f) { fragments.emplace_back(f); };

	// actually do rasterization:
	uint32_t out_of_range = 0; // check if rasterization produced fragments outside framebuffer 

	std::vector< Vec3 > const& samples = framebuffer.sample_pattern.centers_and_weights;
	for (uint32_t s = 0; s < samples.size(); ++s)
	{
		float dx = samples[s].x - 0.5f;
		float dy = samples[s].y - 0.5f;


		if constexpr (primitive_type == PrimitiveType::Lines) {
			for (uint32_t i = 0; i + 1 < clipped_vertices.size(); i += 2) {
				ClippedVertex coord1 = clipped_vertices[i];
				coord1.fb_position.x += dx;
				coord1.fb_position.y += dy;
				ClippedVertex coord2 = clipped_vertices[i + 1];
				coord2.fb_position.x += dx;
				coord2.fb_position.y += dy;
				rasterize_line(coord1, coord2, emit_fragment);
			}
		}
		else if constexpr (primitive_type == PrimitiveType::Triangles) {
			for (uint32_t i = 0; i + 2 < clipped_vertices.size(); i += 3) {
				ClippedVertex coord1 = clipped_vertices[i];
				coord1.fb_position.x += dx;
				coord1.fb_position.y += dy;
				ClippedVertex coord2 = clipped_vertices[i + 1];
				coord2.fb_position.x += dx;
				coord2.fb_position.y += dy;
				ClippedVertex coord3 = clipped_vertices[i + 2];
				coord3.fb_position.x += dx;
				coord3.fb_position.y += dy;
				rasterize_triangle(coord1, coord2, coord3, emit_fragment);
			}
		}
		else {
			static_assert(primitive_type == PrimitiveType::Lines, "Unsupported primitive type.");
		}

		//--------------------------
		// depth test + shade + blend fragments:
		// (indicates something is wrong with clipping)
		for (auto const& f : fragments) {

			// fragment location (in pixels):
			int32_t x = (int32_t)std::floor(f.fb_position.x);
			int32_t y = (int32_t)std::floor(f.fb_position.y);

			// if clipping is working properly, this condition shouldn't be needed;
			// however, it prevents crashes while you are working on your clipping functions,
			// so we suggest leaving it in place:
			if (x < 0 || (uint32_t)x >= framebuffer.width ||
				y < 0 || (uint32_t)y >= framebuffer.height) {
				++out_of_range;
				continue;
			}

			// local names that refer to destination sample in framebuffer:
			float& fb_depth = framebuffer.depth_at(x, y, s);
			Spectrum& fb_color = framebuffer.color_at(x, y, s);


			// depth test:
			if constexpr ((flags & PipelineMask_Depth) == Pipeline_Depth_Always) {
				// "Always" means the depth test always passes.
			}
			else if constexpr ((flags & PipelineMask_Depth) == Pipeline_Depth_Never) {
				// "Never" means the depth test never passes.
				continue; //discard this fragment
			}
			else if constexpr ((flags & PipelineMask_Depth) == Pipeline_Depth_Less) {
				// "Less" means the depth test passes when the new fragment has depth less than the stored depth.
				// A1T4: Depth_Less
				// TODO: implement depth test! We want to only emit fragments that have a depth less than the stored depth, hence "Depth_Less".
				if (f.fb_position.z >= fb_depth) {
					continue;
				}

			}
			else {
				static_assert((flags & PipelineMask_Depth) <= Pipeline_Depth_Always, "Unknown depth test flag.");
			}

			// if depth test passes, and depth writes aren't disabled, write depth to depth buffer:
			if constexpr (!(flags & Pipeline_DepthWriteDisableBit)) {
				fb_depth = f.fb_position.z;
			}

			// shade fragment:
			ShadedFragment sf;
			sf.fb_position = f.fb_position;
			Program::shade_fragment(parameters, f.attributes, f.derivatives, &sf.color, &sf.opacity);

			// write color to framebuffer if color writes aren't disabled:
			if constexpr (!(flags & Pipeline_ColorWriteDisableBit)) {
				// blend fragment:
				if constexpr ((flags & PipelineMask_Blend) == Pipeline_Blend_Replace) {
					fb_color = sf.color;
				}
				else if constexpr ((flags & PipelineMask_Blend) == Pipeline_Blend_Add) {
					// A1T4: Blend_Add
					// TODO: framebuffer color should have fragment color multiplied by fragment opacity added to it.
					fb_color += sf.color * sf.opacity;
				}
				else if constexpr ((flags & PipelineMask_Blend) == Pipeline_Blend_Over) {
					// A1T4: Blend_Over
					// TODO: set framebuffer color to the result of "over" blending (also called "alpha blending") the fragment color over the framebuffer color, using the fragment's opacity
					// 		 You may assume that the framebuffer color has its alpha premultiplied already, and you just want to compute the resulting composite color
					fb_color = sf.color + (1.0f - sf.opacity) * fb_color;
				}
				else {
					static_assert((flags & PipelineMask_Blend) <= Pipeline_Blend_Over, "Unknown blending flag.");
				}
			}
		}
	}

	if (out_of_range > 0) {
		if constexpr (primitive_type == PrimitiveType::Lines) {
			warn("Produced %d fragments outside framebuffer; this indicates something is likely "
			     "wrong with the clip_line function.",
			     out_of_range);
		} else if constexpr (primitive_type == PrimitiveType::Triangles) {
			warn("Produced %d fragments outside framebuffer; this indicates something is likely "
			     "wrong with the clip_triangle function.",
			     out_of_range);
		}
	}
}

// -------------------------------------------------------------------------
// clipping functions

// helper to interpolate between vertices:
template<PrimitiveType p, class P, uint32_t F>
auto Pipeline<p, P, F>::lerp(ShadedVertex const& a, ShadedVertex const& b, float t) -> ShadedVertex {
	ShadedVertex ret;
	ret.clip_position = (b.clip_position - a.clip_position) * t + a.clip_position;
	for (uint32_t i = 0; i < ret.attributes.size(); ++i) {
		ret.attributes[i] = (b.attributes[i] - a.attributes[i]) * t + a.attributes[i];
	}
	return ret;
}

/*
 * clip_line - clip line to portion with -w <= x,y,z <= w, emit vertices of clipped line (if non-empty)
 *  	va, vb: endpoints of line
 *  	emit_vertex: call to produce truncated line
 *
 * If clipping shortens the line, attributes of the shortened line should respect the pipeline's interpolation mode.
 * 
 * If no portion of the line remains after clipping, emit_vertex will not be called.
 *
 * The clipped line should have the same direction as the full line.
 */
template<PrimitiveType p, class P, uint32_t flags>
void Pipeline<p, P, flags>::clip_line(ShadedVertex const& va, ShadedVertex const& vb,
                                      std::function<void(ShadedVertex const&)> const& emit_vertex) {
	// Determine portion of line over which:
	// 		pt = (b-a) * t + a
	//  	-pt.w <= pt.x <= pt.w
	//  	-pt.w <= pt.y <= pt.w
	//  	-pt.w <= pt.z <= pt.w
	// ... as a range [min_t, max_t]:

	float min_t = 0.0f;
	float max_t = 1.0f;

	// want to set range of t for a bunch of equations like:
	//    a.x + t * ba.x <= a.w + t * ba.w
	// so here's a helper:
	auto clip_range = [&min_t, &max_t](float l, float dl, float r, float dr) {
		// restrict range such that:
		// l + t * dl <= r + t * dr
		// re-arranging:
		//  l - r <= t * (dr - dl)
		if (dr == dl) {
			// want: l - r <= 0
			if (l - r > 0.0f) {
				// works for none of range, so make range empty:
				min_t = 1.0f;
				max_t = 0.0f;
			}
		} else if (dr > dl) {
			// since dr - dl is positive:
			// want: (l - r) / (dr - dl) <= t
			min_t = std::max(min_t, (l - r) / (dr - dl));
		} else { // dr < dl
			// since dr - dl is negative:
			// want: (l - r) / (dr - dl) >= t
			max_t = std::min(max_t, (l - r) / (dr - dl));
		}
	};

	// local names for clip positions and their difference:
	Vec4 const& a = va.clip_position;
	Vec4 const& b = vb.clip_position;
	Vec4 const ba = b - a;

	// -a.w - t * ba.w <= a.x + t * ba.x <= a.w + t * ba.w
	clip_range(-a.w, -ba.w, a.x, ba.x);
	clip_range(a.x, ba.x, a.w, ba.w);
	// -a.w - t * ba.w <= a.y + t * ba.y <= a.w + t * ba.w
	clip_range(-a.w, -ba.w, a.y, ba.y);
	clip_range(a.y, ba.y, a.w, ba.w);
	// -a.w - t * ba.w <= a.z + t * ba.z <= a.w + t * ba.w
	clip_range(-a.w, -ba.w, a.z, ba.z);
	clip_range(a.z, ba.z, a.w, ba.w);

	if (min_t < max_t) {
		if (min_t == 0.0f) {
			emit_vertex(va);
		} else {
			ShadedVertex out = lerp(va, vb, min_t);
			// don't interpolate attributes if in flat shading mode:
			if constexpr ((flags & PipelineMask_Interp) == Pipeline_Interp_Flat) {
				out.attributes = va.attributes;
			}
			emit_vertex(out);
		}
		if (max_t == 1.0f) {
			emit_vertex(vb);
		} else {
			ShadedVertex out = lerp(va, vb, max_t);
			// don't interpolate attributes if in flat shading mode:
			if constexpr ((flags & PipelineMask_Interp) == Pipeline_Interp_Flat) {
				out.attributes = va.attributes;
			}
			emit_vertex(out);
		}
	}
}

/*
 * clip_triangle - clip triangle to portion with -w <= x,y,z <= w, emit resulting shape as triangles (if non-empty)
 *  	va, vb, vc: vertices of triangle
 *  	emit_vertex: call to produce clipped triangles (three calls per triangle)
 *
 * If clipping truncates the triangle, attributes of the new vertices should respect the pipeline's interpolation mode.
 * 
 * If no portion of the triangle remains after clipping, emit_vertex will not be called.
 *
 * The clipped triangle(s) should have the same winding order as the full triangle.
 */
template<PrimitiveType p, class P, uint32_t flags>
void Pipeline<p, P, flags>::clip_triangle(
	ShadedVertex const& va, ShadedVertex const& vb, ShadedVertex const& vc,
	std::function<void(ShadedVertex const&)> const& emit_vertex) {
	// A1EC: clip_triangle
	// TODO: correct code!
	emit_vertex(va);
	emit_vertex(vb);
	emit_vertex(vc);
}


// -------------------------------------------------------------------------
// rasterization functions

/*
 * rasterize_line:
 * calls emit_fragment( frag ) for every pixel "covered" by the line (va.fb_position.xy, vb.fb_position.xy).
 *
 *    a pixel (x,y) is "covered" by the line if it exits the inscribed diamond:
 * 
 *        (x+0.5,y+1)
 *        /        \
 *    (x,y+0.5)  (x+1,y+0.5)
 *        \        /
 *         (x+0.5,y)
 *
 *    to avoid ambiguity, we consider diamonds to contain their left and bottom points
 *    but not their top and right points. 
 * 
 * 	  since 45 degree lines breaks this rule, our rule in general is to rasterize the line as if its
 *    endpoints va and vb were at va + (e, e^2) and vb + (e, e^2) where no smaller nonzero e produces 
 *    a different rasterization result. 
 *    We will not explicitly check for 45 degree lines along the diamond edges (this will be extra credit),
 *    but you should be able to handle 45 degree lines in every other case (such as starting from pixel centers)
 *
 * for each such diamond, pass Fragment frag to emit_fragment, with:
 *  - frag.fb_position.xy set to the center (x+0.5,y+0.5)
 *  - frag.fb_position.z interpolated linearly between va.fb_position.z and vb.fb_position.z
 *  - frag.attributes set to va.attributes (line will only be used in Interp_Flat mode)
 *  - frag.derivatives set to all (0,0)
 *
 * when interpolating the depth (z) for the fragments, you may use any depth the line takes within the pixel
 * (i.e., you don't need to interpolate to, say, the closest point to the pixel center)
 *
 * If you wish to work in fixed point, check framebuffer.h for useful information about the framebuffer's dimensions.
 */
template<PrimitiveType p, class P, uint32_t flags>
void Pipeline<p, P, flags>::rasterize_line(
	ClippedVertex const& va, ClippedVertex const& vb,
	std::function<void(Fragment const&)> const& emit_fragment) {
	if constexpr ((flags & PipelineMask_Interp) != Pipeline_Interp_Flat) {
		assert(0 && "rasterize_line should only be invoked in flat interpolation mode.");
	}
	// A1T2: rasterize_line

	// TODO: Check out the block comment above this function for more information on how to fill in
	// this function!
	// The OpenGL specification section 3.5 may also come in handy.
	
	auto diamondOct = [=](float x, float y) {
		float isoX = x - floor(x);
		float isoY = y - floor(y);
		/*
		1 / | \ 4
		/ 2 | 3 \
		---------
		\ 6 | 7 /
		5 \ | / 8
		*/
		if (isoX <= 0.5f) {
			if (isoY < 0.5f) {
				if (isoX + isoY < 0.5f) {
					return 5;
				}
				else {
					return 6;
				}
			}
			else {
				if (isoY - isoX >= 0.5f) {
					return 1;
				}
				else {
					return 2;
				}
			}
		}
		else {
			if (isoY < 0.5f) {
				if (isoX - isoY < 0.5f) {
					return 7;
				}
				else {
					return 8;
				}
			}
			else {
				if (isoY + isoX < 1.5f) {
					return 3;
				}
				else {
					return 4;
				}
			}
		}
	};

	auto exitsDiamond = [=](float x, float y, float slope, bool start) {
		float isoX = x - floor(x);
		float isoY = y - floor(y);
		if (start) {
			float followY = (0.999f - isoX) * slope + isoY;
			float followX = 0.999f;
			if (std::abs(followY) > 1.0f) {
				followX = (0.999f - isoY) * (1.0f / slope) + isoX;
				if (slope > 0) { followY = 0.999f; }
				else { followY = 0.0f; };
			} // Attempts to calculate the point the line exits the pixel
			int octLeft = diamondOct(isoX, isoY);
			int octRight = diamondOct(followX, followY);
			return (octLeft != octRight); // never crossed anything (start case only possible if never exited)
		}
		else {
			float followY = isoX * (0.0f - slope) + isoY;
			float followX = 0.0f;
			if (std::abs(followY) > 1.0f) {
				followX = isoY * (0.0f - (1.0f / slope)) + isoX;
				if (slope > 0) { followY = 0.0f; }
				else { followY = 0.999f; }
			}// Attempts to calculate the point the line enters the pixel
			int octLeft = diamondOct(followX, followY);
			int octRight = diamondOct(isoX, isoY);
			return ((octRight == 1 || octRight == 4 || octRight == 5 || octRight == 8) && octRight != octLeft);
		}
	};

	float x1 = va.fb_position.x; 
	float y1 = va.fb_position.y;
	float x2 = vb.fb_position.x; 
	float y2 = vb.fb_position.y;
	float dx = x2 - x1; 
	float dy = y2 - y1;
	float slope;
	if (dx == 0) {
		slope = std::numeric_limits<float>::max();
	}
	else {
		slope = dy / dx;
	}
	if (dx > dy){
		// is x-major
		if (x1 > x2) {
			float x3 = x2; float y3 = y2;
			x2 = x1; y2 = y1;
			x1 = x3; y1 = y3;
		}
		int t1, t2;
		if (exitsDiamond(x1, y1, slope, true)) {
			t1 = (int)floor(x1);
		}
		else {
			t1 = (int)floor(x1) + 1;
		}
		if (exitsDiamond(x2, y2, slope, false)) {
			t2 = (int)floor(x2) + 1;
		}
		else {
			t2 = (int)floor(x2);
		}
		for (int u = t1; u < t2; u++) 
		{
			float w = ((float)u + 0.5f - x1) / (x2 - x1);
			float v = w * (y2 - y1) + y1;
			Fragment out;
			out.fb_position.x = (float)u + 0.5f;
			out.fb_position.y = (float)floor(v) + 0.5f;
			out.fb_position.z = ((vb.fb_position.z - va.fb_position.z) / (float)(t2 - t1)) * (float)(u - t1) + va.fb_position.z;
			out.attributes = va.attributes;
			out.derivatives.fill(Vec2(0.0f, 0.0f));
			emit_fragment(out);
		}
	}
	else {
		// is y-major
		if (y1 > y2) {
			float x3 = x2; 
			float y3 = y2;
			x2 = x1; y2 = y1;
			x1 = x3; y1 = y3;
		}
		int t1, t2;
		if (exitsDiamond(x1, y1, slope, true)) {
			t1 = (int)floor(y1);
		}
		else {
			t1 = (int)floor(y1) + 1;
		}
		if (exitsDiamond(x2, y2, slope, false)) {
			t2 = (int)floor(y2) + 1;
		}
		else {
			t2 = (int)floor(y2);
		}
		for (int u = t1; u < t2; u++) {
			float w = ((float)u + 0.5f - y1) / (y2 - y1);
			float v = w * (x2 - x1) + x1;
			Fragment out;
			out.fb_position.x = (float)floor(v) + 0.5f;
			out.fb_position.y = (float)(u) + 0.5f;
			out.fb_position.z = (va.fb_position.z + ((vb.fb_position.z - va.fb_position.z) * ((float)(u-t1)/(float)t2)));
			out.attributes = va.attributes;
			out.derivatives.fill(Vec2(0.0f, 0.0f));
			emit_fragment(out);
		}
	}

}

/*
 * rasterize_triangle(a,b,c,emit) calls 'emit(frag)' at every location
 *  	(x+0.5,y+0.5) (where x,y are integers) covered by triangle (a,b,c).
 *
 * The emitted fragment should have:
 * - frag.fb_position.xy = (x+0.5, y+0.5)
 * - frag.fb_position.z = linearly interpolated fb_position.z from a,b,c (NOTE: does not depend on Interp mode!)
 * - frag.attributes = depends on Interp_* flag in flags:
 *   - if Interp_Flat: copy from va.attributes
 *   - if Interp_Smooth: interpolate as if (a,b,c) is a 2D triangle flat on the screen
 *   - if Interp_Correct: use perspective-correct interpolation
 * - frag.derivatives = derivatives w.r.t. fb_position.x and fb_position.y of the first frag.derivatives.size() attributes.
 *
 * Notes on derivatives:
 * 	The derivatives are partial derivatives w.r.t. screen locations. That is:
 *    derivatives[i].x = d/d(fb_position.x) attributes[i]
 *    derivatives[i].y = d/d(fb_position.y) attributes[i]
 *  You may compute these derivatives analytically or numerically.
 *
 *  See section 8.12.1 "Derivative Functions" of the GLSL 4.20 specification for some inspiration. (*HOWEVER*, the spec is solving a harder problem, and also nothing in the spec is binding on your implementation)
 *
 *  One approach is to rasterize blocks of four fragments and use forward and backward differences to compute derivatives.
 *  To assist you in this approach, keep in mind that the framebuffer size is *guaranteed* to be even. (see framebuffer.h)
 *
 * Notes on coverage:
 *  If two triangles are on opposite sides of the same edge, and a
 *  fragment center lies on that edge, rasterize_triangle should
 *  make sure that exactly one of the triangles emits that fragment.
 *  (Otherwise, speckles or cracks can appear in the final render.)
 * 
 *  For degenerate (co-linear) triangles, you may consider them to not be on any side of an edge.
 * 	Thus, even if two degnerate triangles share an edge that contains a fragment center, you don't need to emit it.
 *  You will not lose points for doing something reasonable when handling this case
 *
 *  This is pretty tricky to get exactly right!
 *
 */
template<PrimitiveType p, class P, uint32_t flags>
void Pipeline<p, P, flags>::rasterize_triangle(
	ClippedVertex const& va, ClippedVertex const& vb, ClippedVertex const& vc,
	std::function<void(Fragment const&)> const& emit_fragment) {
	// NOTE: it is okay to restructure this function to allow these tasks to use the
	//  same code paths. Be aware, however, that all of them need to remain working!
	//  (e.g., if you break Flat while implementing Correct, you won't get points
	//   for Flat.)


	//vars I don't want to retype repeatedly
	float ax = va.fb_position.x;
	float ay = va.fb_position.y;
	float bx = vb.fb_position.x;
	float by = vb.fb_position.y;
	float cx = vc.fb_position.x;
	float cy = vc.fb_position.y;
	float az = va.fb_position.z;
	float bz = vb.fb_position.z;
	float cz = vc.fb_position.z;

	float xMin = std::min(ax, std::min(bx, cx));
	float xMax = std::max(ax, std::max(bx, cx));
	float yMin = std::min(ay, std::min(by, cy));
	float yMax = std::max(ay, std::max(by, cy));

	//lambda functions I don't want to retype repeatedly
	/*auto det = [=](float x1, float y1, float x2, float y2) {
		return ((x1 * y2) - (y1 * x2));
		};*/
	

	auto getBary = [=, &ax, &bx, &cx, &ay, &by, &cy](float qx, float qy, float aa, float ba, float ca) {
		//float a = std::abs((0.5f * (ax*(by - cy) + bx*(ay - cy) + cx*(ay - by))));
		float d = ((by - cy) * (ax - cx)) + ((cx - bx) * (ay - cy));

		float x = (((by - cy) * (qx - cx)) + ((cx - bx) * (qy - cy))) / d;
		float y = (((cy - ay) * (qx - cx)) + ((ax - cx) * (qy - cy))) / d;
		float z = 1.0f - x - y;
		return (aa * x + ba * y + ca * z);
		};

	auto pointInTriangle = [&](float qx, float qy) {
		if (qx > xMax || qx < xMin || qy > yMax || qy < yMin) {
			return false;
		}

		//std::cout << "Checking point (" + std::to_string(qx) + ", " + std::to_string(qy) + ") \n";
		float d = ((by - cy) * (ax - cx)) + ((cx - bx) * (ay - cy));
		// I want the x and y barymetrics as well.
		float baryx = (((by - cy) * (qx - cx)) + ((cx - bx) * (qy - cy))) / d;
		float baryy = (((cy - ay) * (qx - cx)) + ((ax - cx) * (qy - cy))) / d; 
		float baryz = 1.0f - baryx - baryy;
		//std::cout << "BaryX = " + std::to_string(baryx) + ", BaryY = " + std::to_string(baryy) + ", BaryZ = " + std::to_string(baryz) + ". \n";


		if (baryx < 0.0f || baryy < 0.0f || baryz < 0.0f) {
			return false;
		}
		if (baryx > 0.0f && baryy > 0.0f && baryz > 0.0f) {
			return true;
		}
		
		// baryx (representing a) == 0 means it lies on bc, baryy == 0 -> lies on ac, baryz == 0 -> lies on ab

		bool isCCW = (((bx - ax) * (cy - ay) - (cx - ax) * (by - ay)) > 0.0f); // rotation of a->b->c

		if (isCCW) {
			if (baryy == 0.0f && ((cy > ay) || (cy == ay && (cy > by)))) {
				//std::cout << "CCW Cy = " + std::to_string(cy) + "; Ay = " + std::to_string(ay) + "; By = " + std::to_string(by) + ". \n";
				return true;
			}
			if (baryx == 0.0f && ((by > cy) || (cy == by && (cy > ay)))) {
				//std::cout << "CCW By = " + std::to_string(by) + "; Cy = " + std::to_string(cy) + "; Ay = " + std::to_string(ay) +  ". \n";
				return true;
			}
			if (baryz == 0.0f && ((ay > by) || (by == ay && (ay > cy)))) {
				//std::cout << "CCW Ay = " + std::to_string(ay) + "; By = " + std::to_string(by) + "; Cy = " + std::to_string(cy) + ". \n";
				return true;
			}
		}
		else {
			if (baryy == 0.0f && ((cy < ay) || (cy == ay && (cy > by)))) {
				//std::cout << "CW Cy = " + std::to_string(cy) + "; Ay = " + std::to_string(ay) + "; By = " + std::to_string(by) + ". \n";
				return true;
			}
			if (baryx == 0.0f && ((by < cy) || (cy == by && (cy > ay)))) {
				//std::cout << "CW By = " + std::to_string(by) + "; Cy = " + std::to_string(cy) + "; Ay = " + std::to_string(ay) + ". \n";
				return true;
			}
			if (baryz == 0.0f && ((ay < by) || (by == ay && (ay > cy)))) {
				//std::cout << "CW Ay = " + std::to_string(ay) + "; By = " + std::to_string(by) + "; Cy = " + std::to_string(cy) + ". \n";
				return true;
			}
		}
		return false;
		// float dot1, dot2, dot3;
		/*if (isCW) {
			// CW Calc
			//std::cout << "CW\n";
			// dot1 = det(cx - ax, cy - ay, bx - ax, by - ay) * det(cx - ax, cy - ay, qx - ax, qy - ay); //ac
			// dot2 = det(bx - cx, by - cy, ax - cx, ay - cy) * det(bx - cx, by - cy, qx - cx, qy - cy); //cb
			// dot3 = det(ax - bx, ay - by, cx - bx, cy - by) * det(ax - bx, ay - by, qx - bx, qy - by); //ba
			if (dot1 > 0.0f && dot2 > 0.0f && dot3 > 0.0f) { return true; } //at exact edge, one of these will be 0.0f
			if (dot1 == 0.0f && ((cy > ay) || (cy == ay && (cy > by)))) {
				//std::cout << "CW Cy = " + std::to_string(cy) + "; Ay = " + std::to_string(ay) + "; By = " + std::to_string(by) + ". \n";
				return true;
			}
			if (dot2 == 0.0f && ((by > cy) || (cy == by && (cy > ay)))) {
				//std::cout << "CW By = " + std::to_string(by) + "; Cy = " + std::to_string(cy) + "; Ay = " + std::to_string(ay) +  ". \n";
				return true;
			}
			if (dot3 == 0.0f && ((ay > by) || (by == ay && (ay > cy)))) {
				//std::cout << "CW Ay = " + std::to_string(ay) + "; By = " + std::to_string(by) + "; Cy = " + std::to_string(cy) + ". \n";
				return true;
			}
			return false;
		}
		else {
			// CCW calc
			//std::cout << "CCW\n";
			dot1 = det(bx - ax, by - ay, cx - ax, cy - ay) * det(cx - ax, cy - ay, qx - ax, qy - ay); //ac
			dot2 = det(ax - cx, ay - cy, bx - cx, by - cy) * det(bx - cx, by - cy, qx - cx, qy - cy); //cb
			dot3 = det(cx - bx, cy - by, ax - bx, ay - by) * det(ax - bx, ay - by, qx - bx, qy - by); //ba
			if (dot1 < 0.0f && dot2 < 0.0f && dot3 < 0.0f) { return true; }
			if (dot1 == 0.0f && ((cy < ay) || (cy == ay && (cy > by)))) {
				//std::cout << "CCW Cy = " + std::to_string(cy) + "; Ay = " + std::to_string(ay) + "; By = " + std::to_string(by) + ". \n";
				return true;
			}
			if (dot2 == 0.0f && ((by < cy) || (cy == by && (cy > ay)))) {
				//std::cout << "CCW By = " + std::to_string(by) + "; Cy = " + std::to_string(cy) + "; Ay = " + std::to_string(ay) + ". \n";
				return true;
			}
			if (dot3 == 0.0f && ((ay < by) || (by == ay && (ay > cy)))) {
				//std::cout << "CCW Ay = " + std::to_string(ay) + "; By = " + std::to_string(by) + "; Cy = " + std::to_string(cy) + ". \n";
				return true;
			}
			return false;
		}*/
		};

	auto intersectsOver = [&](float q, float start, float end, bool isLR) {
		int bound = ((int)end) - ((int)start) + 1;
		if (isLR) {
			for (int e = 0; e < bound; e++) {
				if (pointInTriangle(q + 0.5f, start + (float)e + 0.5f)) {
					return true;
				}
			}
		}
		else {
			for (int e = 0; e < bound; e++) {
				if (pointInTriangle(start + (float)e + 0.5f, q + 0.5f)) {
					return true;
				}
			}
		}
		return false;
		};

	auto getZ = [=, &ax, &bx, &cx, &ay, &by, &cy, &az, &bz, &cz](float qx, float qy) {
		//float a = std::abs((0.5f * (ax*(by - cy) + bx*(ay - cy) + cx*(ay - by))));
		float d = ((by - cy) * (ax - cx)) + ((cx - bx) * (ay - cy));

		float x = (((by - cy) * (qx - cx)) + ((cx - bx) * (qy - cy))) / d;
		float y = (((cy - ay) * (qx - cx)) + ((ax - cx) * (qy - cy))) / d;
		float z = 1.0f - x - y;
		return (az * x + bz * y + cz * z);
		};
	auto getBaryInterp = [&](float qx, float qy, float aa, float ba, float ca, float aw, float bw, float cw) {
		float d = ((by - cy) * (ax - cx)) + ((cx - bx) * (ay - cy));
		float baryx = (((by - cy) * (qx - cx)) + ((cx - bx) * (qy - cy))) / d;
		float baryy = (((cy - ay) * (qx - cx)) + ((ax - cx) * (qy - cy))) / d;
		float baryz = 1.0f - baryx - baryy;
		float phiw = baryx * aa * aw + baryy * ba * bw + baryz * ca * cw;
		float intw = baryx * aw + baryy * bw + baryz * cw;
		if (intw == 0.0f) {
			if (phiw == 0.0f) {
				return 0.0f;
			}
			else {
				return std::numeric_limits<float>::max(); // divide by 0 case
			}
		}
		else {
			return phiw / intw;
		}
		};

	if constexpr ((flags & PipelineMask_Interp) == Pipeline_Interp_Flat) {
		// A1T3: flat triangles
		// TODO: rasterize triangle (see block comment above this function).


		//this gives bounds for raster
		

		
		//int clen = (int)(ceil(xMax) - floor(xMin));
		//int rlen = (int)(ceil(yMax) - floor(yMin));


		auto checkFrom = [&](float xbl, float ybl, float xtr, float ytr, auto&& checkFrom) {
			int width = (int)(floor(xtr) - floor(xbl)) + 1;
			int height = (int)(floor(ytr) - floor(ybl)) + 1;
			if (width == 1 && height == 1) 
			{
				if (pointInTriangle(xbl + 0.5f, ybl + 0.5f)) {
					Fragment out;
					out.fb_position.x = xbl + 0.5f;
					out.fb_position.y = ybl + 0.5f;
					out.fb_position.z = getZ(out.fb_position.x, out.fb_position.y);
					//std::cout << "(" + std::to_string(out.fb_position.x) + ", " + std::to_string(out.fb_position.y) + ") got through from individual check.\n";
					out.attributes = va.attributes;
					out.derivatives.fill(Vec2(0.0f, 0.0f));
					emit_fragment(out);
				}
				/*else
				{
					//std::cout << "(" + std::to_string(xbl + 0.5f) + ", " + std::to_string(ybl + 0.5f) + ") got rejected by individual check.\n";
				}*/
				return 1;
			} 
			//std::cout << "Checking (" + std::to_string(xbl) + ", " + std::to_string(ybl) + "), (" + std::to_string(xtr) + ", " + std::to_string(ytr) + ")";
			bool inbl = pointInTriangle(xbl + 0.5f, ybl + 0.5f);
			bool inbr = pointInTriangle(xtr + 0.5f, ybl + 0.5f);
			bool intl = pointInTriangle(xbl + 0.5f, ytr + 0.5f);
			bool intr = pointInTriangle(xtr + 0.5f, ytr + 0.5f);
			if (intl && intr && inbl && inbr) {
				//std::cout << "; Full\n";
				for (int ri = 0; ri < height; ri++) {
					for (int ci = 0; ci < width; ci++) {
						Fragment out;
						out.fb_position.x = (float)ci + xbl + 0.5f;
						out.fb_position.y = (float)ri + ybl + 0.5f;
						//std::cout << "(" + std::to_string(out.fb_position.x) + ", " + std::to_string(out.fb_position.y) + ") got through from early-in.\n";
						out.fb_position.z = getZ(out.fb_position.x, out.fb_position.y);
						out.attributes = va.attributes;
						out.derivatives.fill(Vec2(0.0f, 0.0f));
						emit_fragment(out);
					}
				}
				return 1;
			}
			if (intersectsOver(xbl, ybl, ytr, true) || intersectsOver(xtr, ybl, ytr, true) || intersectsOver(ybl, xbl, xtr, false) || intersectsOver(ytr, xbl, xtr, false)) {
				//std::cout << "; Mixed\n";
				if (width == 1) {
					for (int ci = 0; ci < height; ci++) {
						checkFrom(xbl, ybl + ci, xbl, ybl + ci, checkFrom);
					}
					return 1;
				}
				if (height == 1) {
					for (int ri = 0; ri < width; ri++) {
						checkFrom(xbl+ri, ybl, xbl+ ri, ybl, checkFrom);
					}
					return 1;
				}
				checkFrom(xbl, ybl, xbl + (float)(width / 2 - 1), ybl + (float)(height / 2 - 1), checkFrom); // bl
				checkFrom(xbl + (float)(width / 2), ybl, xtr, ybl + (float)(height / 2 - 1), checkFrom); // br
				checkFrom(xbl, ybl + (float)(height/2), xbl + (float)(width / 2 - 1), ytr, checkFrom); // tl
				checkFrom(xbl + (float)(width / 2), ybl + (float)(height / 2), xtr, ytr, checkFrom); // tr
				return 1;
			}
			//std::cout << "; Empty\n";
			return 0;
		};

		//Skip first check because it can be annoying
		auto startFun = [&](float xbl, float ybl, float xtr, float ytr, auto&& checkFrom) {
			int width = (int)(floor(xtr) - floor(xbl)) + 1;
			int height = (int)(floor(ytr) - floor(ybl)) + 1;
			checkFrom(xbl, ybl, xbl + (float)(width / 2 - 1), ybl + (float)(height / 2 - 1), checkFrom); // bl
			checkFrom(xbl + (float)(width / 2), ybl, xtr, ybl + (float)(height / 2 - 1), checkFrom); // br
			checkFrom(xbl, ybl + (float)(height / 2), xbl + (float)(width / 2 - 1), ytr, checkFrom); // tl
			checkFrom(xbl + (float)(width / 2), ybl + (float)(height / 2), xtr, ytr, checkFrom);
			};

		startFun(floor(xMin), floor(yMin), floor(xMax), floor(yMax), checkFrom);
		
		//std::cout << "Checking " + std::to_string(rlen) + " rows and " + std::to_string(clen) + " cols\n";
		/*for (int ri = 0; ri < rlen; ri++) {
			for (int ci = 0; ci < clen; ci++) {
				if (pointInTriangle((float)ci + floor(xMin) + 0.5f, ((float)ri + floor(yMin)) + 0.5f)) {
					Fragment out;
					out.fb_position.x = (float)ci + floor(xMin) + 0.5f;
					out.fb_position.y = (float)ri + floor(yMin) + 0.5f;
					out.fb_position.z = getZ(out.fb_position.x, out.fb_position.y);
					out.attributes = va.attributes;
					//out.derivatives[i].x = d / d(fb_position.x) attributes[i];
					//out.derivatives[i].y = d / d(fb_position.y) attributes[i];
					out.derivatives.fill(Vec2(0.0f, 0.0f)); // UPDATE THIS FOR 1.5
					emit_fragment(out);
				}
			}
		}*/
		/*
		Pipeline<PrimitiveType::Lines, P, flags>::rasterize_line(va, vb, emit_fragment);
		Pipeline<PrimitiveType::Lines, P, flags>::rasterize_line(vb, vc, emit_fragment);
		Pipeline<PrimitiveType::Lines, P, flags>::rasterize_line(vc, va, emit_fragment);*/
	}
	else if constexpr ((flags & PipelineMask_Interp) == Pipeline_Interp_Smooth) {
		// A1T5: screen-space smooth triangles
		// TODO: rasterize triangle (see block comment above this function).
		
		auto checkFrom = [&](float xbl, float ybl, float xtr, float ytr, auto&& checkFrom) {
			int width = (int)(floor(xtr) - floor(xbl)) + 1;
			int height = (int)(floor(ytr) - floor(ybl)) + 1;
			if (width == 1 && height == 1)
			{
				if (pointInTriangle(xbl + 0.5f, ybl + 0.5f)) {
					Fragment out;
					out.fb_position.x = xbl + 0.5f;
					out.fb_position.y = ybl + 0.5f;
					out.fb_position.z = getZ(out.fb_position.x, out.fb_position.y);
					for (uint32_t attr = 0; attr < va.attributes.size(); attr++) {
						out.attributes[attr] = getBary(out.fb_position.x, out.fb_position.y, va.attributes[attr], vb.attributes[attr], vc.attributes[attr]);
					}
					float ux = getBary(out.fb_position.x + 1.0f, out.fb_position.y, va.attributes[0], vb.attributes[0], vc.attributes[0]);
					float uy = getBary(out.fb_position.x, out.fb_position.y + 1.0f, va.attributes[0], vb.attributes[0], vc.attributes[0]);
					float vx = getBary(out.fb_position.x + 1.0f, out.fb_position.y, va.attributes[1], vb.attributes[1], vc.attributes[1]);
					float vy = getBary(out.fb_position.x, out.fb_position.y + 1.0f, va.attributes[1], vb.attributes[1], vc.attributes[1]);
					out.derivatives[0] = Vec2(ux - out.attributes[0], uy - out.attributes[0]);
					out.derivatives[1] = Vec2(vx - out.attributes[1], vy - out.attributes[1]);
					emit_fragment(out);
				}

				return 1;
			}
			//std::cout << "Checking (" + std::to_string(xbl) + ", " + std::to_string(ybl) + "), (" + std::to_string(xtr) + ", " + std::to_string(ytr) + ")";
			bool inbl = pointInTriangle(xbl + 0.5f, ybl + 0.5f);
			bool inbr = pointInTriangle(xtr + 0.5f, ybl + 0.5f);
			bool intl = pointInTriangle(xbl + 0.5f, ytr + 0.5f);
			bool intr = pointInTriangle(xtr + 0.5f, ytr + 0.5f);
			if (intl && intr && inbl && inbr) {
				//std::cout << "; Full\n";
				for (int ri = 0; ri < height; ri++) {
					for (int ci = 0; ci < width; ci++) {
						Fragment out;
						out.fb_position.x = (float)ci + xbl + 0.5f;
						out.fb_position.y = (float)ri + ybl + 0.5f;
						out.fb_position.z = getZ(out.fb_position.x, out.fb_position.y);
						for (uint32_t attr = 0; attr < va.attributes.size(); attr++) {
							out.attributes[attr] = getBary(out.fb_position.x, out.fb_position.y, va.attributes[attr], vb.attributes[attr], vc.attributes[attr]);
						}
						float ux = getBary(out.fb_position.x + 1.0f, out.fb_position.y, va.attributes[0], vb.attributes[0], vc.attributes[0]);
						float uy = getBary(out.fb_position.x, out.fb_position.y + 1.0f, va.attributes[0], vb.attributes[0], vc.attributes[0]);
						float vx = getBary(out.fb_position.x + 1.0f, out.fb_position.y, va.attributes[1], vb.attributes[1], vc.attributes[1]);
						float vy = getBary(out.fb_position.x, out.fb_position.y + 1.0f, va.attributes[1], vb.attributes[1], vc.attributes[1]);
						out.derivatives[0] = Vec2(ux - out.attributes[0], uy - out.attributes[0]);
						out.derivatives[1] = Vec2(vx - out.attributes[1], vy - out.attributes[1]);
						emit_fragment(out);
					}
				}
				return 1;
			}
			if (intersectsOver(xbl, ybl, ytr, true) || intersectsOver(xtr, ybl, ytr, true) || intersectsOver(ybl, xbl, xtr, false) || intersectsOver(ytr, xbl, xtr, false)) {
				//std::cout << "; Mixed\n";
				if (width == 1) {
					for (int ci = 0; ci < height; ci++) {
						checkFrom(xbl, ybl + ci, xbl, ybl + ci, checkFrom);
					}
					return 1;
				}
				if (height == 1) {
					for (int ri = 0; ri < width; ri++) {
						checkFrom(xbl + ri, ybl, xbl + ri, ybl, checkFrom);
					}
					return 1;
				}
				checkFrom(xbl, ybl, xbl + (float)(width / 2 - 1), ybl + (float)(height / 2 - 1), checkFrom); // bl
				checkFrom(xbl + (float)(width / 2), ybl, xtr, ybl + (float)(height / 2 - 1), checkFrom); // br
				checkFrom(xbl, ybl + (float)(height / 2), xbl + (float)(width / 2 - 1), ytr, checkFrom); // tl
				checkFrom(xbl + (float)(width / 2), ybl + (float)(height / 2), xtr, ytr, checkFrom); // tr
				return 1;
			}
			//std::cout << "; Empty\n";
			return 0;
			};

		//Skip first check because it can be annoying
		auto startTri = [&](float xbl, float ybl, float xtr, float ytr, auto&& checkFrom) {
			int width = (int)(floor(xtr) - floor(xbl)) + 1;
			int height = (int)(floor(ytr) - floor(ybl)) + 1;
			checkFrom(xbl, ybl, xbl + (float)(width / 2 - 1), ybl + (float)(height / 2 - 1), checkFrom); // bl
			checkFrom(xbl + (float)(width / 2), ybl, xtr, ybl + (float)(height / 2 - 1), checkFrom); // br
			checkFrom(xbl, ybl + (float)(height / 2), xbl + (float)(width / 2 - 1), ytr, checkFrom); // tl
			checkFrom(xbl + (float)(width / 2), ybl + (float)(height / 2), xtr, ytr, checkFrom);
			};

		startTri(floor(xMin), floor(yMin), floor(xMax), floor(yMax), checkFrom);
		// As a placeholder, here's code that calls the Flat interpolation version of the function:
		//(remove this and replace it with a real solution)
		// Pipeline<PrimitiveType::Lines, P, (flags & ~PipelineMask_Interp) | Pipeline_Interp_Flat>::rasterize_triangle(va, vb, vc, emit_fragment);
	} else if constexpr ((flags & PipelineMask_Interp) == Pipeline_Interp_Correct) {
		// A1T5: perspective correct triangles
		// TODO: rasterize triangle (block comment above this function).
		

		auto checkFrom = [&](float xbl, float ybl, float xtr, float ytr, auto&& checkFrom) {
			int width = (int)(floor(xtr) - floor(xbl)) + 1;
			int height = (int)(floor(ytr) - floor(ybl)) + 1;
			if (width == 1 && height == 1)
			{
				float d = ((by - cy) * (ax - cx)) + ((cx - bx) * (ay - cy));
				float baryx, baryy, baryz, phiw, intw;
			
				if (pointInTriangle(xbl + 0.5f, ybl + 0.5f)) {
					Fragment out;
					out.fb_position.x = xbl + 0.5f;
					out.fb_position.y = ybl + 0.5f;
					out.fb_position.z = getBary(out.fb_position.x, out.fb_position.y, az, bz, cz);
					for (uint32_t attr = 0; attr < va.attributes.size(); attr++) {
						baryx = (((by - cy) * (out.fb_position.x - cx)) + ((cx - bx) * (out.fb_position.y - cy))) / d;
						baryy = (((cy - ay) * (out.fb_position.x - cx)) + ((ax - cx) * (out.fb_position.y - cy))) / d;
						baryz = 1.0f - baryx - baryy;
						phiw = baryx * va.attributes[attr] * va.inv_w + baryy * vb.attributes[attr] * vb.inv_w + baryz * vc.attributes[attr] * vc.inv_w;
						intw = baryx * va.inv_w + baryy * vb.inv_w + baryz * vc.inv_w;
						if (intw == 0.0f) {
							out.attributes[attr] = std::numeric_limits<float>::max(); // divide by 0 case
						}
						else {
							out.attributes[attr] = phiw / intw;
						}
					}
					/*float ux = getBary(out.fb_position.x + 1.0f, out.fb_position.y, va.attributes[0], vb.attributes[0], vc.attributes[0]);
					float uy = getBary(out.fb_position.x, out.fb_position.y + 1.0f, va.attributes[0], vb.attributes[0], vc.attributes[0]);
					float vx = getBary(out.fb_position.x + 1.0f, out.fb_position.y, va.attributes[1], vb.attributes[1], vc.attributes[1]);
					float vy = getBary(out.fb_position.x, out.fb_position.y + 1.0f, va.attributes[1], vb.attributes[1], vc.attributes[1]);*/
					float ux = getBaryInterp(out.fb_position.x + 1.0f, out.fb_position.y, va.attributes[0], vb.attributes[0], vc.attributes[0], va.inv_w, vb.inv_w, vc.inv_w);
					float uy = getBaryInterp(out.fb_position.x, out.fb_position.y + 1.0f, va.attributes[0], vb.attributes[0], vc.attributes[0], va.inv_w, vb.inv_w, vc.inv_w);
					float vx = getBaryInterp(out.fb_position.x + 1.0f, out.fb_position.y, va.attributes[1], vb.attributes[1], vc.attributes[1], va.inv_w, vb.inv_w, vc.inv_w);
					float vy = getBaryInterp(out.fb_position.x, out.fb_position.y + 1.0f, va.attributes[1], vb.attributes[1], vc.attributes[1], va.inv_w, vb.inv_w, vc.inv_w);
					
					/*float uMax = std::max(va.attributes[0], std::max(vb.attributes[0], vc.attributes[0]));
					float uMin = std::min(va.attributes[0], std::min(vb.attributes[0], vc.attributes[0]));
					float vMax = std::max(va.attributes[1], std::max(vb.attributes[1], vc.attributes[1]));
					float vMin = std::min(va.attributes[1], std::min(vb.attributes[1], vc.attributes[1]));*/
					out.derivatives[0] = Vec2((ux - out.attributes[0]), 
											  (uy - out.attributes[0]));
					out.derivatives[1] = Vec2((vx - out.attributes[1]), 
											  (vy - out.attributes[1]));
					// std::cout << "O1 = " + std::to_string(out.attributes[0]) + "; O2 = " + std::to_string(out.attributes[1]) + "\n";
					// std::cout << "ux " + std::to_string(ux) + " uy " + std::to_string(uy) + " vx " + std::to_string(vx) + " vy " + std::to_string(vy) + "\n";
					// std::cout << "D1 = " + std::to_string(out.derivatives[0].x) + ", " + std::to_string(out.derivatives[0].y) + "; D2 = " + std::to_string(out.derivatives[1].x) + ", " + std::to_string(out.derivatives[1].y) + "\n";
					emit_fragment(out);
				}

				return 1;
			}
			//std::cout << "Checking (" + std::to_string(xbl) + ", " + std::to_string(ybl) + "), (" + std::to_string(xtr) + ", " + std::to_string(ytr) + ")";
			bool inbl = pointInTriangle(xbl + 0.5f, ybl + 0.5f);
			bool inbr = pointInTriangle(xtr + 0.5f, ybl + 0.5f);
			bool intl = pointInTriangle(xbl + 0.5f, ytr + 0.5f);
			bool intr = pointInTriangle(xtr + 0.5f, ytr + 0.5f);
			if (intl && intr && inbl && inbr) {
				//std::cout << "; Full\n";
				float d = ((by - cy) * (ax - cx)) + ((cx - bx) * (ay - cy)); 
				float baryx, baryy, baryz, phiw, intw; // Helper lambda assumes you only want interp
				for (int ri = 0; ri < height; ri++) {
					for (int ci = 0; ci < width; ci++) {
						Fragment out;
						out.fb_position.x = (float)ci + xbl + 0.5f;
						out.fb_position.y = (float)ri + ybl + 0.5f;
						out.fb_position.z = getZ(out.fb_position.x, out.fb_position.y);
						for (uint32_t attr = 0; attr < va.attributes.size(); attr++) {
							baryx = (((by - cy) * (out.fb_position.x - cx)) + ((cx - bx) * (out.fb_position.y - cy))) / d;
							baryy = (((cy - ay) * (out.fb_position.x - cx)) + ((ax - cx) * (out.fb_position.y - cy))) / d;
							baryz = 1.0f - baryx - baryy;
						    phiw = baryx * va.attributes[attr] * va.inv_w + baryy * vb.attributes[attr] * vb.inv_w + baryz * vc.attributes[attr] * vc.inv_w;
							intw = baryx * va.inv_w + baryy * vb.inv_w + baryz * vc.inv_w;
							if (intw == 0.0f) {
								if (phiw == 0.0f) {
									out.attributes[attr] = 0.0f;
								}
								else {
									out.attributes[attr] = std::numeric_limits<float>::max(); // divide by 0 case
								}
							}
							else {
								out.attributes[attr] = phiw/intw;
							}
						}
						/*float ux = getBary(out.fb_position.x + 1.0f, out.fb_position.y, va.attributes[0], vb.attributes[0], vc.attributes[0]);
						float uy = getBary(out.fb_position.x, out.fb_position.y + 1.0f, va.attributes[0], vb.attributes[0], vc.attributes[0]);
						float vx = getBary(out.fb_position.x + 1.0f, out.fb_position.y, va.attributes[1], vb.attributes[1], vc.attributes[1]);
						float vy = getBary(out.fb_position.x, out.fb_position.y + 1.0f, va.attributes[1], vb.attributes[1], vc.attributes[1]);*/
						float ux = getBaryInterp(out.fb_position.x + 1.0f, out.fb_position.y, va.attributes[0], vb.attributes[0], vc.attributes[0], va.inv_w, vb.inv_w, vc.inv_w);
						float uy = getBaryInterp(out.fb_position.x, out.fb_position.y + 1.0f, va.attributes[0], vb.attributes[0], vc.attributes[0], va.inv_w, vb.inv_w, vc.inv_w);
						float vx = getBaryInterp(out.fb_position.x + 1.0f, out.fb_position.y, va.attributes[1], vb.attributes[1], vc.attributes[1], va.inv_w, vb.inv_w, vc.inv_w);
						float vy = getBaryInterp(out.fb_position.x, out.fb_position.y + 1.0f, va.attributes[1], vb.attributes[1], vc.attributes[1], va.inv_w, vb.inv_w, vc.inv_w);
						out.derivatives[0] = Vec2(ux - out.attributes[0], uy - out.attributes[0]);
						out.derivatives[1] = Vec2(vx - out.attributes[1], vy - out.attributes[1]);
						// std::cout << "O1 = " + std::to_string(out.attributes[0]) + "; O2 = " + std::to_string(out.attributes[1]) + "\n";
						// std::cout << "ux " + std::to_string(ux) + " uy " + std::to_string(uy) + " vx " + std::to_string(vx) + " vy " + std::to_string(vy) + "\n";
						// std::cout << "D1 = " + std::to_string(out.derivatives[0].x) + ", " + std::to_string(out.derivatives[0].y) + "; D2 = " + std::to_string(out.derivatives[1].x) + ", " + std::to_string(out.derivatives[1].y) + "\n";
						emit_fragment(out);
					}
				}
				return 1;
			}
			if (intersectsOver(xbl, ybl, ytr, true) || intersectsOver(xtr, ybl, ytr, true) || intersectsOver(ybl, xbl, xtr, false) || intersectsOver(ytr, xbl, xtr, false)) {
				//std::cout << "; Mixed\n";
				if (width == 1) {
					for (int ci = 0; ci < height; ci++) {
						checkFrom(xbl, ybl + ci, xbl, ybl + ci, checkFrom);
					}
					return 1;
				}
				if (height == 1) {
					for (int ri = 0; ri < width; ri++) {
						checkFrom(xbl + ri, ybl, xbl + ri, ybl, checkFrom);
					}
					return 1;
				}
				checkFrom(xbl, ybl, xbl + (float)(width / 2 - 1), ybl + (float)(height / 2 - 1), checkFrom); // bl
				checkFrom(xbl + (float)(width / 2), ybl, xtr, ybl + (float)(height / 2 - 1), checkFrom); // br
				checkFrom(xbl, ybl + (float)(height / 2), xbl + (float)(width / 2 - 1), ytr, checkFrom); // tl
				checkFrom(xbl + (float)(width / 2), ybl + (float)(height / 2), xtr, ytr, checkFrom); // tr
				return 1;
			}
			//std::cout << "; Empty\n";
			return 0;
			};

		//Skip first check because it can be annoying
		auto startTri = [&](float xbl, float ybl, float xtr, float ytr, auto&& checkFrom) {
			int width = (int)(floor(xtr) - floor(xbl)) + 1;
			int height = (int)(floor(ytr) - floor(ybl)) + 1;
			checkFrom(xbl, ybl, xbl + (float)(width / 2 - 1), ybl + (float)(height / 2 - 1), checkFrom); // bl
			checkFrom(xbl + (float)(width / 2), ybl, xtr, ybl + (float)(height / 2 - 1), checkFrom); // br
			checkFrom(xbl, ybl + (float)(height / 2), xbl + (float)(width / 2 - 1), ytr, checkFrom); // tl
			checkFrom(xbl + (float)(width / 2), ybl + (float)(height / 2), xtr, ytr, checkFrom);
			};

		startTri(floor(xMin), floor(yMin), floor(xMax), floor(yMax), checkFrom);
		// As a placeholder, here's code that calls the Screen-space interpolation function:
		//(remove this and replace it with a real solution)
		// Pipeline<PrimitiveType::Lines, P, (flags & ~PipelineMask_Interp) | Pipeline_Interp_Smooth>::rasterize_triangle(va, vb, vc, emit_fragment);
	}
}

//-------------------------------------------------------------------------
// compile instantiations for all programs and blending and testing types:

#include "programs.h"

template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Always | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Always | Pipeline_Interp_Smooth>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Always | Pipeline_Interp_Correct>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Never | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Never | Pipeline_Interp_Smooth>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Never | Pipeline_Interp_Correct>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Less | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Less | Pipeline_Interp_Smooth>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Less | Pipeline_Interp_Correct>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Always | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Always | Pipeline_Interp_Smooth>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Always | Pipeline_Interp_Correct>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Never | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Never | Pipeline_Interp_Smooth>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Never | Pipeline_Interp_Correct>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Less | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Less | Pipeline_Interp_Smooth>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Less | Pipeline_Interp_Correct>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Always | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Always | Pipeline_Interp_Smooth>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Always | Pipeline_Interp_Correct>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Never | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Never | Pipeline_Interp_Smooth>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Never | Pipeline_Interp_Correct>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Less | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Less | Pipeline_Interp_Smooth>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Less | Pipeline_Interp_Correct>;
template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Always | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Never | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Less | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Always | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Never | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Less | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Always | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Never | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Less | Pipeline_Interp_Flat>;