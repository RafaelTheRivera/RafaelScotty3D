
#include "bvh.h"
#include "aggregate.h"
#include "instance.h"
#include "tri_mesh.h"

#include <stack>

namespace PT {

struct BVHBuildData {
	BVHBuildData(size_t start, size_t range, size_t dst) : start(start), range(range), node(dst) {
	}
	size_t start; ///< start index into the primitive array
	size_t range; ///< range of index into the primitive array
	size_t node;  ///< address to update
};

struct SAHBucketData {
	BBox bb;          ///< bbox of all primitives
	size_t num_prims; ///< number of primitives in the bucket
};
template<typename Primitive>
void BVH<Primitive>::build(std::vector<Primitive>&& prims, size_t max_leaf_size) {
	//A3T3 - build a bvh



	// Keep these
	nodes.clear();
	primitives = std::move(prims);
	/*for (int primnow = 0; primnow < primitives.size(); primnow++) {
		BBox bad = primitives[primnow].bbox();
		std::cout << "Primitive " + std::to_string(primnow) + " ";
		std::cout << "Min: (" + std::to_string(bad.min.x) + ", " + std::to_string(bad.min.y) + ", " + std::to_string(bad.min.z) + "); ";
		std::cout << "Max: (" + std::to_string(bad.max.x) + ", " + std::to_string(bad.max.y) + ", " + std::to_string(bad.max.z) + ") \n";
		
	}*/
	// std::cout << "max leaf " + std::to_string(max_leaf_size) + "\n";
	// Construct a BVH from the given vector of primitives and maximum leaf
	// size configuration.
	auto recurse = [&](size_t start, size_t end, auto&& recurse) {
		// this case never happens and it's completly redundant and useless but for some reason when I try to delete it the whole function breaks. Would love to know why
		//std::cout << "enter recurse\n";
		/*if (end - start < max_leaf_size) {
			std::cout << "special out";
			return (size_t)0;
		}*/
		//std::cout << "pass weird case\n";
		float best_cost = std::numeric_limits<float>::max();
		// int best_partition = 0;
		int best_axis = -1; // 0 = x, 1 = y, 2 = z;
		BBox besta, bestb, c;
		size_t best_acount = 0;
		// size_t best_bcount = 0;

		/*auto distance = [](Vec3 a, Vec3 b) {
			float xdist = b.x - a.x;
			float ydist = b.y - a.y;
			float zdist = b.z - a.z;
			return sqrt(xdist * xdist + ydist * ydist + zdist * zdist);
			};*/

		//calculate c early because it's the same for all
		//std::cout << "size: " + std::to_string(end-start);
		for (size_t prim = start; prim < end; prim++) {
			BBox ccurrent = primitives[prim].bbox();
			//Vec3 ccenter = ccurrent;
			c.enclose(ccurrent);/*
			std::cout << "Center: (" + std::to_string(ccenter[0]) + ", " + std::to_string(ccenter[1]) + ", " + std::to_string(ccenter[2]) + ") \n";
			std::cout << "Min: (" + std::to_string(c.min.x) + ", " + std::to_string(c.min.y) + ", " + std::to_string(c.min.z) + ") \n";
			std::cout << "Max: (" + std::to_string(c.max.x) + ", " + std::to_string(c.max.y) + ", " + std::to_string(c.max.z) + ") \n";

			std::cout << "Current size : " + std::to_string(c.surface_area()) + "\n";
		*/
			/*if (distance(ccurrent.min, ccurrent.max) > 10000) {
				BBox bad = ccurrent;
				std::cout << "something went wrong with primitive " + std::to_string(prim) + " earlier than expected. \n";
				std::cout << "Min: (" + std::to_string(bad.min.x) + ", " + std::to_string(bad.min.y) + ", " + std::to_string(bad.min.z) + ") \n";
				std::cout << "Max: (" + std::to_string(bad.max.x) + ", " + std::to_string(bad.max.y) + ", " + std::to_string(bad.max.z) + ") \n";
			}*/
		}
		if (end - start <= max_leaf_size) {
			// std::cout << "early out\n";
			size_t m = new_node(c, start, end-start, 0, 0);
			return m;
		}

		//std::cout << "in here\n";
		for (int axis = 0; axis < 3; axis++) {

			auto compare_prims = [&axis](const auto& a, const auto& b) {
				BBox abox = a.bbox();
				BBox bsbox = b.bbox();
				Vec3 centera = abox.center();
				Vec3 centerb = bsbox.center();
				return centera[axis] < centerb[axis];
				};


			std::sort(primitives.begin() + start, primitives.begin() + end, compare_prims);


			// size_t n = end - start;
			// make approximately equal bins as primitives per bin
			size_t bin_n =  8; // (size_t)ceil(sqrt((float)n));
			// size_t remainder = n % bin_n;
			// size_t bin_size = (size_t)ceil((float)n / (float)bin_n);

			// n = 17 -> bin_n = 5 -> bin_size = 4
			
			/*if (n == 2) {
				bin_n = 2;
				remainder = 0;
				bin_size = 1;
			}*/
			// std::cout << "total = " + std::to_string(n) + " bin_n " + std::to_string(bin_n) + " remainder " + std::to_string(remainder) + " bin size " + std::to_string(bin_size) + "\n";
			//std::cout << "first loop\n";
			std::vector<SAHBucketData> buckets;
			
			for (size_t bucket_it = 0; bucket_it < bin_n; bucket_it++) {
				SAHBucketData bin;
				bin.num_prims = 0;
				buckets.emplace_back(bin);
			}
			float bucket_length = (c.max[axis] - c.min[axis]) / ((float)bin_n);
			for (size_t i = start; i < end; i++) {
				BBox currentPrim = primitives[i].bbox();
				Vec3 currentPrimCenter = currentPrim.center();
				
				float axisloc = currentPrimCenter[axis];
				size_t actual_index = (size_t)floor((axisloc - c.min[axis]) / bucket_length);
				if (actual_index == bin_n) {
					actual_index = bin_n - 1;
				}
				buckets[actual_index].bb.enclose(currentPrim);
				buckets[actual_index].num_prims += 1;
			}
			/* for (size_t i = 0; i < bin_n; i++) {
				if (distance(primitives[i * bin_size + j + start].bbox().min, primitives[i * bin_size + j + start].bbox().max) > 10000) {
					BBox bad = primitives[i * bin_size + j + start].bbox();
					std::cout << "something went wrong with primitive " + std::to_string(i * bin_size + j + start) + "\n";
					std::cout << "Min: (" + std::to_string(bad.min.x) + ", " + std::to_string(bad.min.y) + ", " + std::to_string(bad.min.z) + ") \n";
					std::cout << "Max: (" + std::to_string(bad.max.x) + ", " + std::to_string(bad.max.y) + ", " + std::to_string(bad.max.z) + ") \n";
				}

				bin.bb.enclose(primitives[i * bin_size + j + start].bbox());
				bin.num_prims += 1;
				// SAHBucketData bin;
				// bin.num_prims = 0;
				// sorted by axis so we can just do this
				for (size_t j = 0; j < bin_size; j++) {
					if (i * bin_size + j < n) {
						if (distance(primitives[i * bin_size + j + start].bbox().min, primitives[i * bin_size + j + start].bbox().max) > 10000) {
							BBox bad = primitives[i * bin_size + j + start].bbox();
							std::cout << "something went wrong with primitive " + std::to_string(i * bin_size + j + start) + "\n";
							std::cout << "Min: (" + std::to_string(bad.min.x) + ", " + std::to_string(bad.min.y) + ", " + std::to_string(bad.min.z) + ") \n";
							std::cout << "Max: (" + std::to_string(bad.max.x) + ", " + std::to_string(bad.max.y) + ", " + std::to_string(bad.max.z) + ") \n";
						}

						bin.bb.enclose(primitives[i * bin_size + j + start].bbox());
						bin.num_prims += 1; 

						// std::cout << "adding prim with center (" + std::to_string(primitives[i * bin_size + j + start].bbox().center().x) + ", " + std::to_string(primitives[i * bin_size + j + start].bbox().center().y) + ", " + std::to_string(primitives[i * bin_size + j + start].bbox().center().z) + ") to bucket " + std::to_string(i) + "\n";
						// std::cout << "num prims for " + std::to_string(i) + std::to_string(j) + " is " + std::to_string(bin.num_prims) + "\n";
						// std::cout << std::to_string(i * bin_size + j);
						// std::cout << std::to_string(primitives[i * bin_size + j].bbox().center()[axis]);
					}
				}
				// buckets.emplace_back(bin);
			} */
			// std::cout << "second loop\n";
			

			for (size_t j = 0; j < (bin_n - 1); j++) {
				BBox a, b;
				size_t a_count = 0;
				size_t b_count = 0;
				for (size_t k = 0; k < bin_n; k++) {
					/*auto compare = [&j, &k]() {
						return (k <= j);
						};*/
					/*if (distance(buckets[k].bb.min, buckets[k].bb.max) > 10000) {
						std::cout << "something went wrong with bucket" + std::to_string(k) + " of size " + std::to_string(buckets[k].num_prims) + "\n";
					}*/
					if (k <= j) {
						a.enclose(buckets[k].bb);
						a_count += buckets[k].num_prims;
						// std::cout << "adding bucket with center (" + std::to_string(buckets[k].bb.center().x) + ", " + std::to_string(buckets[k].bb.center().y) + ", " + std::to_string(buckets[k].bb.center().z) + ") to A\n";
						// std::cout << "a count " + std::to_string(a_count) + " on round " + std::to_string(k) + "\n";
					}
					else {
						b.enclose(buckets[k].bb);
						b_count += buckets[k].num_prims;
						// std::cout << "adding bucket with center (" + std::to_string(buckets[k].bb.center().x) + ", " + std::to_string(buckets[k].bb.center().y) + ", " + std::to_string(buckets[k].bb.center().z) + ") to B\n";
						// std::cout << "b count " + std::to_string(a_count) + " on round " + std::to_string(k) + "\n";
					}
				}
				/*if (distance(a.min, a.max) > 10000 || distance(b.min, b.max) > 10000) {
					std::cout << "something went wrong\n";
				}*/
				// std::cout << "a " + std::to_string(a.surface_area()) + " b " + std::to_string(b.surface_area()) + " c " + std::to_string(c.surface_area()) + "\n";

				float asa = a.surface_area();
				float bsa = b.surface_area();
				float csa = c.surface_area();
				

				/*if (asa == 0.0f) {
					asa = distance(a.min, a.max) * 0.0001f;
				}
				if (bsa == 0.0f) {
					bsa = distance(b.min, b.max) * 0.0001f;
				}
				if (csa == 0.0f) {
					csa = distance(c.min, c.max) * 0.0001f;
				}*/ // ???
				/*std::cout << "a " + std::to_string(asa) + " b " + std::to_string(bsa) + " c " + std::to_string(csa) + "\n";
				std::cout << "AMin: (" + std::to_string(a.min.x) + ", " + std::to_string(a.min.y) + ", " + std::to_string(a.min.z) + ") \n";
				std::cout << "AMax: (" + std::to_string(a.max.x) + ", " + std::to_string(a.max.y) + ", " + std::to_string(a.max.z) + ") \n";
				std::cout << "BMin: (" + std::to_string(b.min.x) + ", " + std::to_string(b.min.y) + ", " + std::to_string(b.min.z) + ") \n";
				std::cout << "BMax: (" + std::to_string(b.max.x) + ", " + std::to_string(b.max.y) + ", " + std::to_string(b.max.z) + ") \n";
				std::cout << "CMin: (" + std::to_string(c.min.x) + ", " + std::to_string(c.min.y) + ", " + std::to_string(c.min.z) + ") \n";
				std::cout << "CMax: (" + std::to_string(c.max.x) + ", " + std::to_string(c.max.y) + ", " + std::to_string(c.max.z) + ") \n";*/
				float cost = ((asa / csa) * a_count) + ((bsa / csa) * b_count);
				if (cost < best_cost) {
					best_cost = cost;
					// best_partition = j;
					best_axis = axis;
					besta = a;
					bestb = b;
					best_acount = a_count;
					// best_bcount = b_count;

				}
				//std::cout << "checked with cost: " + std::to_string(cost) + " from partition " + std::to_string(j) + " on axis " + std::to_string(axis) + "\n";

			}

			//decide where buckets are...

			//std::cout << "optimized once \n";
		}
		//std::cout << "left loop\n";
		// std::cout << "done with best: " + std::to_string(best_cost) + " from partition " + std::to_string(best_partition) + " on axis " + std::to_string(best_axis) + "\n";
		
		if (!(best_axis == -1)) {
			// std::cout << "enter case 1\n";

			size_t l = 0;
			size_t r = 0;
			size_t m = 0;
			// std::cout << "total: " + std::to_string(end - start) + "; ";
			// std::cout << "best a: " + std::to_string(best_acount) + "; best b: " + std::to_string(best_bcount) + "\n";
			//if (best_acount > max_leaf_size) {
				// std::cout << "enter recurse 1\n";
			// std::cout << "start: " + std::to_string(start) + "; best_acount: " + std::to_string(best_acount) + "\n";
			auto compare_prims_end = [&best_axis](const auto& a, const auto& b) {
				BBox abox = a.bbox();
				BBox bsbox = b.bbox();
				Vec3 centera = abox.center();
				Vec3 centerb = bsbox.center();
				return centera[best_axis] < centerb[best_axis];
				};

			std::sort(primitives.begin() + start, primitives.begin() + end, compare_prims_end);

			l = recurse(start, start + best_acount, recurse);
			//}
			//else {
				// std::cout << "Adding Leaf Node from (" + std::to_string(besta.min.x) + ", " + std::to_string(besta.min.y) + ", " + std::to_string(besta.min.z) + ") to (" + std::to_string(besta.max.x) + ", " + std::to_string(besta.max.y) + ", " + std::to_string(besta.max.z) + ")\n";

				// std::cout << "enter leaf 1\n";
			    // l = new_node(besta, start, best_acount, 0, 0);
			//}
			//if (best_bcount > max_leaf_size) {
				// std::cout << "enter recurse 2\n";
				// std::cout << "start: " + std::to_string(start + best_acount) + "; end: " + std::to_string(end) + "\n";

			r = recurse(start + best_acount, end, recurse);
			//}
			//else {
				// std::cout << "Adding Leaf Node from (" + std::to_string(bestb.min.x) + ", " + std::to_string(bestb.min.y) + ", " + std::to_string(bestb.min.z) + ") to (" + std::to_string(bestb.max.x) + ", " + std::to_string(bestb.max.y) + ", " + std::to_string(bestb.max.z) + ")\n";

				// std::cout << "enter leaf 2\n";
				// r = new_node(bestb, best_acount, best_bcount, 0, 0);
			//}
			//std::cout << std::to_string(l) + "\n";
			//std::cout << std::to_string(r) + "\n";
			m = new_node(c, start, end - start, l, r);
			//std::cout << std::to_string(m) + "\n";
			return m;
		}
		else {
			// std::cout << "enter case 2\n";
			size_t m = 0;
			m = new_node(c, start, end - start, 0, 0);
			return m;
		}
		};
	//TODO

	size_t m = recurse(0, primitives.size(), recurse);
	BVH::root_idx = m;

	/*for (auto nodeit = nodes.begin(); nodeit != nodes.end(); ++nodeit) {
		Node currentNode = *nodeit;
		std::cout << "Node " + std::to_string(std::distance(nodes.begin(), nodeit)) +  " from (" + std::to_string(currentNode.bbox.min.x) + ", " + std::to_string(currentNode.bbox.min.y) + ", " + std::to_string(currentNode.bbox.min.z) + ") to (" + std::to_string(currentNode.bbox.max.x) + ", " + std::to_string(currentNode.bbox.max.y) + ", " + std::to_string(currentNode.bbox.max.z) + ")\n";
		std::cout << "Start: " + std::to_string(currentNode.start) + "; Size: " + std::to_string(currentNode.size) + "; Children: " + std::to_string(currentNode.l) + ", " + std::to_string(currentNode.r) + "\n";
	} */
	/*auto compare_prims_end = [&best_axis](const auto& a, const auto& b) {
		BBox abox = a.bbox();
		BBox bsbox = b.bbox();
		Vec3 centera = abox.center();
		Vec3 centerb = bsbox.center();
		return centera[best_axis] < centerb[best_axis];
		};

	std::sort(primitives.begin(), primitives.end(), compare_prims_end);
	std::cout << std::to_string(max_leaf_size) + "\n";
	std::cout << std::to_string(best_acount) + "\n";
	std::cout << std::to_string(best_bcount) + "\n";*/
	/*if (best_acount > max_leaf_size) {
		//std::vector<Primitive>::const_iterator aend = primitives.begin() + (best_acount-1);
		//std::vector<auto&> subvec(primitives.begin(), aend);
		recurseOnce(subvec, max_leaf_size);
		//(void)subvec;
		std::cout << "d";
	}
	else {
		new_node(besta, 0, best_acount, 0, 0);
	}
	if (best_bcount > max_leaf_size) {
		//std::vector<Primitive>::const_iterator bstart = primitives.begin() + best_acount;
		//std::vector<auto&> subvec(bstart, primitives.end());
		//build((primitives.begin() + best_acount, primitives.end()), max_leaf_size);
		//(void)subvec;
		std::cout << "b";
	}
	else {
		new_node(bestb, best_acount, best_bcount, 0, 0);
	}*/
	// std::cout << "ran once\n";
}

template<typename Primitive> Trace BVH<Primitive>::hit(const Ray& ray) const {
	//A3T3 - traverse your BVH

    // Implement ray - BVH intersection test. A ray intersects
    // with a BVH aggregate if and only if it intersects a primitive in
    // the BVH that is not an aggregate.

    // The starter code simply iterates through all the primitives.
    // Again, remember you can use hit() on any Primitive value.

	//TODO: replace this code with a more efficient traversal:
    Trace best;
	//Trace best_brute;
	//int best_prim_num = -1;

	auto recurseHit = [&](size_t node, Trace bestInScope, auto&& recurseHit){
		//std::cout << "in: " + std::to_string(node) + "\n";
		if (nodes.size() <= node) {
			return bestInScope;
		}
		Trace closest = bestInScope;
		Node currentNode = nodes[node];
		Vec2 bound = Vec2(-1.0f * std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
		if (currentNode.bbox.hit(ray, bound)) {
			if (currentNode.is_leaf()) {
				//std::cout << "leaf";
				for (size_t i = currentNode.start; i < currentNode.start + currentNode.size; i++) {
					Trace hit = primitives[i].hit(ray);
					/*if (hit.hit) {
						std::cout << "checked primitive with hit distance: " + std::to_string(hit.distance) + "\n";
					}*/
					/*if (i == best_prim_num) {
						std::cout << "checked best primitive within recursion\n";
					}*/
					closest = Trace::min(closest, hit);
					/*if (best.distance == hit.distance && hit.distance != 0.0f) {
						std::cout << "new best at: " + std::to_string(hit.distance) + "\n";
					}*/
				}
			}
			else {
				//std::cout << "node\n";
				/*Vec2 boundl = Vec2(std::numeric_limits<float>::min(), std::numeric_limits<float>::max());
				Vec2 boundr = Vec2(std::numeric_limits<float>::min(), std::numeric_limits<float>::max());
				bool lhit = nodes[currentNode.l].bbox.hit(ray, boundl);
				bool rhit = nodes[currentNode.r].bbox.hit(ray, boundr);
				size_t first, second;
				float hitsecond;
				if (lhit && rhit) {
					if (boundl.x <= boundr.x) {
						first = currentNode.l;
						second = currentNode.r;
						hitsecond = boundr.x;

					}
					else {
						first = currentNode.r;
						second = currentNode.l;
						hitsecond = boundl.x;
					}
					closest = Trace::min(closest, recurseHit(first, closest, recurseHit));
					if (!closest.hit || hitsecond < closest.distance) {
						// std::cout << "recurse second\n";
						closest = Trace::min(closest, recurseHit(second, closest, recurseHit));
					}
				}
				else if (lhit) {
					closest = Trace::min(closest, recurseHit(currentNode.l, closest, recurseHit));
				}
				else if (rhit) {
					closest = Trace::min(closest, recurseHit(currentNode.r, closest, recurseHit));
				}*/
				closest = Trace::min(closest, recurseHit(currentNode.l, closest, recurseHit));
				closest = Trace::min(closest, recurseHit(currentNode.r, closest, recurseHit));
				
			}
		}
		else {
			// std::cout << "Missed\n";
			/*if (best_prim_num >= currentNode.start && best_prim_num < currentNode.start + currentNode.size) {
				std::cout << "just removed best option in large if/else (node " + std::to_string(node) + ")\n";
				// std::cout << "sanity check " + std::to_string(std::numeric_limits<float>::min()) + "\n";
				std::cout << "bounds set to (" + std::to_string(bound.x) + ", " + std::to_string(bound.y) + ")\n";
			}*/
			return bestInScope;
		}
		return closest;
		};
	// const Primitive& besthit;
	/*for (int current_prim = 0; current_prim < primitives.size(); current_prim++) {
		const Primitive& prim = primitives[current_prim];
        Trace hit = prim.hit(ray);
        best_brute = Trace::min(best_brute, hit);
		if (best_brute.distance == hit.distance && hit.distance != 0.0f) {
			// std::cout << "new brute best at: " + std::to_string(hit.distance) + "\n";
			best_prim_num = current_prim;
		}
    }
	if (best_prim_num != -1) {
		std::cout << "Best Prim Num = " + std::to_string(best_prim_num) + "\n";
	}*/
	// std::cout << "Entering from root " + std::to_string(root_idx) + "\n";
	best = recurseHit(BVH::root_idx, best, recurseHit);
	// std::cout << "best hit is " ;
	/*if (best_brute.distance != best.distance) {
		std::cout << "problem with ray: correct = " + std::to_string (best_brute.distance) + "; calculated: " + std::to_string(best.distance) + "\n";
	}*/

    return best;
}

template<typename Primitive>
BVH<Primitive>::BVH(std::vector<Primitive>&& prims, size_t max_leaf_size) {
	build(std::move(prims), max_leaf_size);
}

template<typename Primitive> std::vector<Primitive> BVH<Primitive>::destructure() {
	nodes.clear();
	return std::move(primitives);
}

template<typename Primitive>
template<typename P>
typename std::enable_if<std::is_copy_assignable_v<P>, BVH<P>>::type BVH<Primitive>::copy() const {
	BVH<Primitive> ret;
	ret.nodes = nodes;
	ret.primitives = primitives;
	ret.root_idx = root_idx;
	return ret;
}

template<typename Primitive> Vec3 BVH<Primitive>::sample(RNG &rng, Vec3 from) const {
	if (primitives.empty()) return {};
	int32_t n = rng.integer(0, static_cast<int32_t>(primitives.size()));
	return primitives[n].sample(rng, from);
}

template<typename Primitive>
float BVH<Primitive>::pdf(Ray ray, const Mat4& T, const Mat4& iT) const {
	if (primitives.empty()) return 0.0f;
	float ret = 0.0f;
	for (auto& prim : primitives) ret += prim.pdf(ray, T, iT);
	return ret / primitives.size();
}

template<typename Primitive> void BVH<Primitive>::clear() {
	nodes.clear();
	primitives.clear();
}

template<typename Primitive> bool BVH<Primitive>::Node::is_leaf() const {
	// A node is a leaf if l == r, since all interior nodes must have distinct children
	return l == r;
}

template<typename Primitive>
size_t BVH<Primitive>::new_node(BBox box, size_t start, size_t size, size_t l, size_t r) {
	Node n;
	n.bbox = box;
	n.start = start;
	n.size = size;
	n.l = l;
	n.r = r;
	nodes.push_back(n);
	return nodes.size() - 1;
}
 
template<typename Primitive> BBox BVH<Primitive>::bbox() const {
	if(nodes.empty()) return BBox{Vec3{0.0f}, Vec3{0.0f}};
	return nodes[root_idx].bbox;
}

template<typename Primitive> size_t BVH<Primitive>::n_primitives() const {
	return primitives.size();
}

template<typename Primitive>
uint32_t BVH<Primitive>::visualize(GL::Lines& lines, GL::Lines& active, uint32_t level,
                                   const Mat4& trans) const {

	std::stack<std::pair<size_t, uint32_t>> tstack;
	tstack.push({root_idx, 0u});
	uint32_t max_level = 0u;

	if (nodes.empty()) return max_level;

	while (!tstack.empty()) {

		auto [idx, lvl] = tstack.top();
		max_level = std::max(max_level, lvl);
		const Node& node = nodes[idx];
		tstack.pop();

		Spectrum color = lvl == level ? Spectrum(1.0f, 0.0f, 0.0f) : Spectrum(1.0f);
		GL::Lines& add = lvl == level ? active : lines;

		BBox box = node.bbox;
		box.transform(trans);
		Vec3 min = box.min, max = box.max;

		auto edge = [&](Vec3 a, Vec3 b) { add.add(a, b, color); };

		edge(min, Vec3{max.x, min.y, min.z});
		edge(min, Vec3{min.x, max.y, min.z});
		edge(min, Vec3{min.x, min.y, max.z});
		edge(max, Vec3{min.x, max.y, max.z});
		edge(max, Vec3{max.x, min.y, max.z});
		edge(max, Vec3{max.x, max.y, min.z});
		edge(Vec3{min.x, max.y, min.z}, Vec3{max.x, max.y, min.z});
		edge(Vec3{min.x, max.y, min.z}, Vec3{min.x, max.y, max.z});
		edge(Vec3{min.x, min.y, max.z}, Vec3{max.x, min.y, max.z});
		edge(Vec3{min.x, min.y, max.z}, Vec3{min.x, max.y, max.z});
		edge(Vec3{max.x, min.y, min.z}, Vec3{max.x, max.y, min.z});
		edge(Vec3{max.x, min.y, min.z}, Vec3{max.x, min.y, max.z});

		if (!node.is_leaf()) {
			tstack.push({node.l, lvl + 1});
			tstack.push({node.r, lvl + 1});
		} else {
			for (size_t i = node.start; i < node.start + node.size; i++) {
				uint32_t c = primitives[i].visualize(lines, active, level - lvl, trans);
				max_level = std::max(c + lvl, max_level);
			}
		}
	}
	return max_level;
}

template class BVH<Triangle>;
template class BVH<Instance>;
template class BVH<Aggregate>;
template BVH<Triangle> BVH<Triangle>::copy<Triangle>() const;

} // namespace PT
