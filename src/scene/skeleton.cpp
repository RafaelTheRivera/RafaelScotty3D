#include <unordered_set>
#include "skeleton.h"
#include "test.h"
#include <iostream>


void Skeleton::Bone::compute_rotation_axes(Vec3 *x_, Vec3 *y_, Vec3 *z_) const {
	assert(x_ && y_ && z_);
	auto &x = *x_;
	auto &y = *y_;
	auto &z = *z_;

	//y axis points in the direction of extent:
	y = extent.unit();
	//if extent is too short to normalize nicely, point along the skeleton's 'y' axis:
	if (!y.valid()) {
		y = Vec3{0.0f, 1.0f, 0.0f};
	}

	//x gets skeleton's 'x' axis projected to be orthogonal to 'y':
	x = Vec3{1.0f, 0.0f, 0.0f};
	x = (x - dot(x,y) * y).unit();
	if (!x.valid()) {
		//if y perfectly aligns with skeleton's 'x' axis, x, gets skeleton's z axis:
		x = Vec3{0.0f, 0.0f, 1.0f};
		x = (x - dot(x,y) * y).unit(); //(this should do nothing)
	}

	//z computed from x,y:
	z = cross(x,y);

	//x,z rotated by roll:
	float cr = std::cos(roll / 180.0f * PI_F);
	float sr = std::sin(roll / 180.0f * PI_F);
	// x = cr * x + sr * -z;
	// z = cross(x,y);
	std::tie(x, z) = std::make_pair(cr * x + sr * -z, cr * z + sr * x);
}

std::vector< Mat4 > Skeleton::bind_pose() const {
	//A4T2a: bone-to-skeleton transformations in the bind pose
	//(the bind pose does not rotate by Bone::pose)

	std::vector< Mat4 > bind;
	bind.reserve(bones.size());

	//NOTE: bones is guaranteed to be ordered such that parents appear before child bones.

	for (auto const &bone : bones) {
		// (void)bone; //avoid complaints about unused bone
		Mat4 tcurrent;
		if (bone.parent == -1U) {
			// base bone
			tcurrent = Mat4::translate(base);
		}
		else {
			tcurrent = bind[bone.parent] * Mat4::translate(bones[bone.parent].extent);
			/*std::cout << "blah\n";
			std::cout << bind[bone.parent];
			std::cout << "\n";
			std::cout << Mat4::translate(bone.extent);
			std::cout << "\n";*/
		}
		//placeholder -- your code should actually compute the correct transform:
		bind.emplace_back(tcurrent);
	}

	assert(bind.size() == bones.size()); //should have a transform for every bone.
	return bind;
}

std::vector< Mat4 > Skeleton::current_pose() const {
    //A4T2a: bone-to-skeleton transformations in the current pose

	//Similar to bind_pose(), but takes rotation from Bone::pose into account.
	// (and translation from Skeleton::base_offset!)

	//You'll probably want to write a loop similar to bind_pose().

	//Useful functions:
	//Bone::compute_rotation_axes() will tell you what axes (in local bone space) Bone::pose should rotate around.
	//Mat4::angle_axis(angle, axis) will produce a matrix that rotates angle (in degrees) around a given axis.
	std::vector< Mat4 > pose;
	pose.reserve(bones.size());

	//NOTE: bones is guaranteed to be ordered such that parents appear before child bones.
	for (auto const& bone : bones) {
		// (void)bone; //avoid complaints about unused bone
		Mat4 tcurrent;
		if (bone.parent == -1U) {
			// base bone
			// Vec3 poseNow = bone.pose;
			Vec3 xAxis, yAxis, zAxis;
			bone.compute_rotation_axes(&xAxis, &yAxis, &zAxis);
			tcurrent = (Mat4::translate(base + base_offset)) * Mat4::angle_axis(bone.pose.z, zAxis) * Mat4::angle_axis(bone.pose.y, yAxis) * Mat4::angle_axis(bone.pose.x, xAxis);
		}
		else {
			// Vec3 poseNow = bone.pose;
			Vec3 xAxis, yAxis, zAxis;
			bone.compute_rotation_axes(&xAxis, &yAxis, &zAxis);
			Vec3 parentExtent = bones[bone.parent].extent;
			tcurrent = pose[bone.parent] * (Mat4::translate(parentExtent) * Mat4::angle_axis(bone.pose.z, zAxis) * Mat4::angle_axis(bone.pose.y, yAxis) * Mat4::angle_axis(bone.pose.x, xAxis));
		}
		//placeholder -- your code should actually compute the correct transform:
		pose.emplace_back(tcurrent);
	}

	assert(pose.size() == pose.size()); //should have a transform for every bone.
	return pose;

}

std::vector< Vec3 > Skeleton::gradient_in_current_pose() const {
    //A4T2b: IK gradient

    // Computes the gradient (partial derivative) of IK energy relative to each bone's Bone::pose, in the current pose.

	//The IK energy is the sum over all *enabled* handles of the squared distance from the tip of Handle::bone to Handle::target
	std::vector< Vec3 > gradient(bones.size(), Vec3{0.0f, 0.0f, 0.0f});
	
	// We can use this to move into skeleton space (by undoing the move into world space)
	std::vector< Mat4 > pose = current_pose();

	//TODO: loop over handles and over bones in the chain leading to the handle, accumulating gradient contributions.
	//remember bone.compute_rotation_axes() -- should be useful here, too!
	for (auto const& handle : handles) {
		if (!handle.enabled) {
			continue;
		}
		BoneIndex boneNow = handle.bone;
		Bone thisBone = bones[handle.bone];
		std::vector<BoneIndex> currentHierarchy;
		// Generate list of current relevant bones
		while (boneNow != -1U) {
			currentHierarchy.emplace(currentHierarchy.begin(),boneNow);
			boneNow = bones[boneNow].parent;
		}
		// Vector of bones in chain
		
		Mat4 s2l;
		Mat4 worldToSkeleton = Mat4::inverse(pose[0]);
		/*std::cout << "pose[0]: ";
		std::cout << pose[0];
		std::cout << "\n";
		std::cout << "w2s: ";
		std::cout << worldToSkeleton;
		std::cout << "\n";
		std::cout << "pose[bone]: ";
		std::cout << pose[handle.bone];
		std::cout << "\n";*/
		Vec3 xAxis, yAxis, zAxis;
		thisBone.compute_rotation_axes(&xAxis, &yAxis, &zAxis);
		
		s2l = pose[handle.bone];
		/*s2l = worldToSkeleton * pose[handle.bone];
		xAxis = worldToSkeleton * xAxis;
		yAxis = worldToSkeleton * yAxis;
		zAxis = worldToSkeleton * zAxis;*/
		
		/*if ((bones[handle.bone]).parent == -1U) {
			s2l = worldToSkeleton;
		}
		else {
			//s2l, x, y, z were here before?
		}*/
		//Vec3 p = s2l * Mat4::angle_axis() * handle.target;
		Vec3 h = handle.target;
		Vec3 p = s2l * thisBone.extent;
		/*std::cout << "s2l: ";
		std::cout << s2l;
		std::cout << "\n";
		std::cout << "target: " + to_string(handle.target) + "\n";
		//Vec4 target = Vec4(handle.target,1.0f);
		// Vec3 p = s2l * target;
		//float fourth = dot(s2l[3], target);
		// Unsure why * kept breaking here, decided to do the same operation 
		// by hand and it comes up with something different.
		//Vec3 p = Vec3{ dot(s2l[0], target)/fourth, dot(s2l[1], target) / fourth, dot(s2l[2], target) / fourth };
		Vec3 p = s2l * thisBone.extent;// *Mat4::angle_axis(thisBone.pose.z, zAxis)* Mat4::angle_axis(thisBone.pose.y, yAxis)* Mat4::angle_axis(thisBone.pose.x, xAxis);

		std::cout << "h: " + to_string(h) + "\n";
		std::cout << "p: " + to_string(p) + "\n";*/

		//Vec3 p = s2lNow * Vec3();
		//std::vector<Mat4> transformations(currentHierarchy.size(), Mat4::I);
		Mat4 xRot = Mat4::I;
		Mat4 yRot = Mat4::I;
		Mat4 zRot = Mat4::I;

		for (size_t i = 0; i < currentHierarchy.size(); i++) {
			BoneIndex index = currentHierarchy[i];
			Bone current = bones[index];
			Mat4 xf;
			Vec3 cxAxis, cyAxis, czAxis; 
			current.compute_rotation_axes(&cxAxis, &cyAxis, &czAxis);

			if (current.parent == -1U) {
				// xf = Mat4::I;
				// transformations[i] = Mat4::angle_axis(current.pose.z, czAxis) * Mat4::angle_axis(current.pose.y, cyAxis) * Mat4::angle_axis(current.pose.x, cxAxis);
				//xf = transformations[i];
				// xRot = Mat4::angle_axis(current.pose.x, cxAxis);
				// yRot = Mat4::angle_axis(current.pose.y, cyAxis);
				// zRot = Mat4::angle_axis(current.pose.z, czAxis);
				xf = pose[index];
			}
			else {
				// Vec3 parentExtent = bones[current.parent].extent;
				// transformations[i] = transformations[i - 1] * Mat4::angle_axis(current.pose.z, czAxis) * Mat4::angle_axis(current.pose.y, cyAxis) * Mat4::angle_axis(current.pose.x, cxAxis);
				//Mat4 worldToSkeleton = Mat4::inverse(pose[0]);
				// xf = transformations[i] * worldToSkeleton * pose[index];
				// xf = transformations[i];
				xf = pose[index];
				/*cxAxis = transformations[i] * cxAxis;
				cyAxis = transformations[i] * cyAxis;
				czAxis = transformations[i] * czAxis;*/
			}
			/*std::cout << "xf: ";
			std::cout << xf;
			std::cout << "\n";*/
			Vec3 r = xf * Vec3(0.0f, 0.0f, 0.0f);
			// Mat4 xf2 = Mat4::angle_axis(current.pose.z, czAxis) * Mat4::angle_axis(current.pose.y, cyAxis) * Mat4::angle_axis(current.pose.x, cxAxis);
			/*Vec4 tcxAxis = Mat4::angle_axis(current.pose.x, cxAxis) * Vec4(cxAxis, 0.0f);
			Vec4 tcyAxis = Mat4::angle_axis(current.pose.y, cyAxis) * Vec4(cyAxis,0.0f);
			Vec4 tczAxis = Mat4::angle_axis(current.pose.z, czAxis) * Vec4(czAxis, 0.0f);
			cxAxis = Vec3(tcxAxis.x, tcxAxis.y, tcxAxis.z);
			cyAxis = Vec3(tcyAxis.x, tcyAxis.y, tcyAxis.z);
			czAxis = Vec3(tczAxis.x, tczAxis.y, tczAxis.z);*/
			cxAxis = xf.rotate(cxAxis);
			cyAxis = xf.rotate(cyAxis);
			czAxis = xf.rotate(czAxis);
			/*std::cout << "r: " + to_string(r) + "\n";
			std::cout << "x axis: " + to_string(cxAxis) + "\n";
			std::cout << "y axis: " + to_string(cyAxis) + "\n";
			std::cout << "z axis: " + to_string(czAxis) + "\n";*/
			// std::cout << "x axis take 2: " + to_string(xf2 * cxAxis) + "\n";
			// std::cout << "y axis take 2: " + to_string(xf2 * cyAxis) + "\n";
			// std::cout << "z axis take 2: " + to_string(xf2 * czAxis) + "\n";
			Vec3 testpr = p - r;
			/*std::cout << "p - r: " + to_string(testpr) + "\n";
			std::cout << "p - h: " + to_string(p-h) + "\n";
			std::cout << "cx cross: " + to_string(cross(cxAxis, (p - r))) + "\n";
			std::cout << "cy cross: " + to_string(cross(cyAxis, (p - r))) + "\n";
			std::cout << "cz cross: " + to_string(cross(czAxis, (p - r))) + "\n";*/
			gradient[index].x += dot(cross(cxAxis, (p - r)), p - h);
			gradient[index].y += dot(cross(cyAxis, (p - r)), p - h);
			gradient[index].z += dot(cross(czAxis, (p - r)), p - h);
			//std::cout << "updated gradient at index " + std::to_string(index) + " to: (" + std::to_string(gradient[index].x) + ", " + std::to_string(gradient[index].y) + ", " + std::to_string(gradient[index].z) + ")\n";
		}


	}
	assert(gradient.size() == bones.size());
	return gradient;
}

bool Skeleton::solve_ik(uint32_t steps) {
	//A4T2b - gradient descent
	//run `steps` iterations

	//call gradient_in_current_pose() to compute d loss / d pose
	//add ...
	// if (true) { return false; };
	float timestep = 1.0f;
	float last_cost = 0.0f;
	for (size_t i = 0; i < steps; i++) {
		std::vector< Vec3 > grad = gradient_in_current_pose();
		std::vector< Mat4 > pose = current_pose();
		
		float cost = 0.0f;
		for (Handle const& handle : handles) {
			Vec3 h = handle.target;
			Vec3 p = pose[handle.bone] * bones[handle.bone].extent;
			auto dist = [](Vec3 in) {
				return sqrt(in.x * in.x + in.y * in.y + in.z * in.z);
			};
			float temp = dist(p - h);
			cost += 0.5f * temp * temp;
		}
		if (cost < 0.0001f || cost - last_cost < 0.0001f) {
			//extra clause for if it can't reach. 
			return true;
		}
		last_cost = cost;
		for (size_t j = 0; j < bones.size(); j++) {
			bones[j].pose -= timestep * grad[j];
		}
	}
	//if at a local minimum (e.g., gradient is near-zero), return 'true'.
	//if run through all steps, return `false`.
	return false;
}

Vec3 Skeleton::closest_point_on_line_segment(Vec3 const &a, Vec3 const &b, Vec3 const &p) {
	//A4T3: bone weight computation (closest point helper)

    // Return the closest point to 'p' on the line segment from a to b

	//Efficiency note: you can do this without any sqrt's! (no .unit() or .norm() is needed!)

    return Vec3{};
}

void Skeleton::assign_bone_weights(Halfedge_Mesh *mesh_) const {
	assert(mesh_);
	auto &mesh = *mesh_;
	(void)mesh; //avoid complaints about unused mesh

	//A4T3: bone weight computation

	//visit every vertex and **set new values** in Vertex::bone_weights (don't append to old values)

	//be sure to use bone positions in the bind pose (not the current pose!)

	//you should fill in the helper closest_point_on_line_segment() before working on this function

}

Indexed_Mesh Skeleton::skin(Halfedge_Mesh const &mesh, std::vector< Mat4 > const &bind, std::vector< Mat4 > const &current) {
	assert(bind.size() == current.size());


	//A4T3: linear blend skinning

	//one approach you might take is to first compute the skinned positions (at every vertex) and normals (at every corner)
	// then generate faces in the style of Indexed_Mesh::from_halfedge_mesh

	//---- step 1: figure out skinned positions ---

	std::unordered_map< Halfedge_Mesh::VertexCRef, Vec3 > skinned_positions;
	std::unordered_map< Halfedge_Mesh::HalfedgeCRef, Vec3 > skinned_normals;
	//reserve hash table space to (one hopes) avoid re-hashing:
	skinned_positions.reserve(mesh.vertices.size());
	skinned_normals.reserve(mesh.halfedges.size());

	//(you will probably want to precompute some bind-to-current transformation matrices here)

	for (auto vi = mesh.vertices.begin(); vi != mesh.vertices.end(); ++vi) {
		skinned_positions.emplace(vi, vi->position); //PLACEHOLDER! Replace with code that computes the position of the vertex according to vi->position and vi->bone_weights.
		//NOTE: vertices with empty bone_weights should remain in place.

		//circulate corners at this vertex:
		auto h = vi->halfedge;
		do {
			//NOTE: could skip if h->face->boundary, since such corners don't get emitted

			skinned_normals.emplace(h, h->corner_normal); //PLACEHOLDER! Replace with code that properly transforms the normal vector! Make sure that you normalize correctly.

			h = h->twin->next;
		} while (h != vi->halfedge);
	}

	//---- step 2: transform into an indexed mesh ---

	//Hint: you should be able to use the code from Indexed_Mesh::from_halfedge_mesh (SplitEdges version) pretty much verbatim, you'll just need to fill in the positions and normals.

	Indexed_Mesh result = Indexed_Mesh::from_halfedge_mesh(mesh, Indexed_Mesh::SplitEdges); //PLACEHOLDER! you'll probably want to copy the SplitEdges case from this function o'er here and modify it to use skinned_positions and skinned_normals.

	return result;
}

void Skeleton::for_bones(const std::function<void(Bone&)>& f) {
	for (auto& bone : bones) {
		f(bone);
	}
}


void Skeleton::erase_bone(BoneIndex bone) {
	assert(bone < bones.size());
	//update indices in bones:
	for (uint32_t b = 0; b < bones.size(); ++b) {
		if (bones[b].parent == -1U) continue;
		if (bones[b].parent == bone) {
			assert(b > bone); //topological sort!
			//keep bone tips in the same place when deleting parent bone:
			bones[b].extent += bones[bone].extent;
			bones[b].parent = bones[bone].parent;
		} else if (bones[b].parent > bone) {
			assert(b > bones[b].parent); //topological sort!
			bones[b].parent -= 1;
		}
	}
	// erase the bone
	bones.erase(bones.begin() + bone);
	//update indices in handles (and erase any handles on this bone):
	for (uint32_t h = 0; h < handles.size(); /* later */) {
		if (handles[h].bone == bone) {
			erase_handle(h);
		} else if (handles[h].bone > bone) {
			handles[h].bone -= 1;
			++h;
		} else {
			++h;
		}
	}
}

void Skeleton::erase_handle(HandleIndex handle) {
	assert(handle < handles.size());

	//nothing internally refers to handles by index so can just delete:
	handles.erase(handles.begin() + handle);
}


Skeleton::BoneIndex Skeleton::add_bone(BoneIndex parent, Vec3 extent) {
	assert(parent == -1U || parent < bones.size());
	Bone bone;
	bone.extent = extent;
	bone.parent = parent;
	//all other parameters left as default.

	//slightly unfortunate hack:
	//(to ensure increasing IDs within an editing session, but reset on load)
	std::unordered_set< uint32_t > used;
	for (auto const &b : bones) {
		used.emplace(b.channel_id);
	}
	while (used.count(next_bone_channel_id)) ++next_bone_channel_id;
	bone.channel_id = next_bone_channel_id++;

	//all other parameters left as default.

	BoneIndex index = BoneIndex(bones.size());
	bones.emplace_back(bone);

	return index;
}

Skeleton::HandleIndex Skeleton::add_handle(BoneIndex bone, Vec3 target) {
	assert(bone < bones.size());
	Handle handle;
	handle.bone = bone;
	handle.target = target;
	//all other parameters left as default.

	//slightly unfortunate hack:
	//(to ensure increasing IDs within an editing session, but reset on load)
	std::unordered_set< uint32_t > used;
	for (auto const &h : handles) {
		used.emplace(h.channel_id);
	}
	while (used.count(next_handle_channel_id)) ++next_handle_channel_id;
	handle.channel_id = next_handle_channel_id++;

	HandleIndex index = HandleIndex(handles.size());
	handles.emplace_back(handle);

	return index;
}


Skeleton Skeleton::copy() {
	//turns out that there aren't any fancy pointer data structures to fix up here.
	return *this;
}

void Skeleton::make_valid() {
	for (uint32_t b = 0; b < bones.size(); ++b) {
		if (!(bones[b].parent == -1U || bones[b].parent < b)) {
			warn("bones[%u].parent is %u, which is not < %u; setting to -1.", b, bones[b].parent, b);
			bones[b].parent = -1U;
		}
	}
	if (bones.empty() && !handles.empty()) {
		warn("Have %u handles but no bones. Deleting handles.", uint32_t(handles.size()));
		handles.clear();
	}
	for (uint32_t h = 0; h < handles.size(); ++h) {
		if (handles[h].bone >= HandleIndex(bones.size())) {
			warn("handles[%u].bone is %u, which is not < bones.size(); setting to 0.", h, handles[h].bone);
			handles[h].bone = 0;
		}
	}
}

//-------------------------------------------------

Indexed_Mesh Skinned_Mesh::bind_mesh() const {
	return Indexed_Mesh::from_halfedge_mesh(mesh, Indexed_Mesh::SplitEdges);
}

Indexed_Mesh Skinned_Mesh::posed_mesh() const {
	return Skeleton::skin(mesh, skeleton.bind_pose(), skeleton.current_pose());
}

Skinned_Mesh Skinned_Mesh::copy() {
	return Skinned_Mesh{mesh.copy(), skeleton.copy()};
}
