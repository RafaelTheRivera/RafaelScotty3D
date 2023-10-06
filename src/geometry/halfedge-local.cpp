
#include "halfedge.h"

#include <unordered_map>
#include <unordered_set>
#include <functional>
#include <iostream>

/******************************************************************
*********************** Local Operations **************************
******************************************************************/

/* Note on local operation return types:

    The local operations all return a std::optional<T> type. This is used so that your
    implementation can signify that it cannot perform an operation (i.e., because
    the resulting mesh does not have a valid representation).

    An optional can have two values: std::nullopt, or a value of the type it is
    parameterized on. In this way, it's similar to a pointer, but has two advantages:
    the value it holds need not be allocated elsewhere, and it provides an API that
    forces the user to check if it is null before using the value.

    In your implementation, if you have successfully performed the operation, you can
    simply return the required reference:

            ... collapse the edge ...
            return collapsed_vertex_ref;

    And if you wish to deny the operation, you can return the null optional:

            return std::nullopt;

    Note that the stubs below all reject their duties by returning the null optional.
*/


/*
 * add_face: add a standalone face to the mesh
 *  sides: number of sides
 *  radius: distance from vertices to origin
 *
 * We provide this method as an example of how to make new halfedge mesh geometry.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::add_face(uint32_t sides, float radius) {
	//faces with fewer than three sides are invalid, so abort the operation:
	if (sides < 3) return std::nullopt;


	std::vector< VertexRef > face_vertices;
	//In order to make the first edge point in the +x direction, first vertex should
	// be at -90.0f - 0.5f * 360.0f / float(sides) degrees, so:
	float const start_angle = (-0.25f - 0.5f / float(sides)) * 2.0f * PI_F;
	for (uint32_t s = 0; s < sides; ++s) {
		float angle = float(s) / float(sides) * 2.0f * PI_F + start_angle;
		VertexRef v = emplace_vertex();
		v->position = radius * Vec3(std::cos(angle), std::sin(angle), 0.0f);
		face_vertices.emplace_back(v);
	}

	assert(face_vertices.size() == sides);

	//assemble the rest of the mesh parts:
	FaceRef face = emplace_face(false); //the face to return
	FaceRef boundary = emplace_face(true); //the boundary loop around the face

	std::vector< HalfedgeRef > face_halfedges; //will use later to set ->next pointers

	for (uint32_t s = 0; s < sides; ++s) {
		//will create elements for edge from a->b:
		VertexRef a = face_vertices[s];
		VertexRef b = face_vertices[(s+1)%sides];

		//h is the edge on face:
		HalfedgeRef h = emplace_halfedge();
		//t is the twin, lies on boundary:
		HalfedgeRef t = emplace_halfedge();
		//e is the edge corresponding to h,t:
		EdgeRef e = emplace_edge(false); //false: non-sharp

		//set element data to something reasonable:
		//(most ops will do this with interpolate_data(), but no data to interpolate here)
		h->corner_uv = a->position.xy() / (2.0f * radius) + 0.5f;
		h->corner_normal = Vec3(0.0f, 0.0f, 1.0f);
		t->corner_uv = b->position.xy() / (2.0f * radius) + 0.5f;
		t->corner_normal = Vec3(0.0f, 0.0f,-1.0f);

		//thing -> halfedge pointers:
		e->halfedge = h;
		a->halfedge = h;
		if (s == 0) face->halfedge = h;
		if (s + 1 == sides) boundary->halfedge = t;

		//halfedge -> thing pointers (except 'next' -- will set that later)
		h->twin = t;
		h->vertex = a;
		h->edge = e;
		h->face = face;

		t->twin = h;
		t->vertex = b;
		t->edge = e;
		t->face = boundary;

		face_halfedges.emplace_back(h);
	}

	assert(face_halfedges.size() == sides);

	for (uint32_t s = 0; s < sides; ++s) {
		face_halfedges[s]->next = face_halfedges[(s+1)%sides];
		face_halfedges[(s+1)%sides]->twin->next = face_halfedges[s]->twin;
	}

	return face;
}


/*
 * bisect_edge: split an edge without splitting the adjacent faces
 *  e: edge to split
 *
 * returns: added vertex
 *
 * We provide this as an example for how to implement local operations.
 * (and as a useful subroutine!)
 */
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::bisect_edge(EdgeRef e) {
	// Phase 0: draw a picture
	//
	// before:
	//    ----h--->
	// v1 ----e--- v2
	//   <----t---
	//
	// after:
	//    --h->    --h2->
	// v1 --e-- vm --e2-- v2
	//    <-t2-    <--t--
	//

	// Phase 1: collect existing elements
	HalfedgeRef h = e->halfedge;
	HalfedgeRef t = h->twin;
	VertexRef v1 = h->vertex;
	VertexRef v2 = t->vertex;

	// Phase 2: Allocate new elements, set data
	VertexRef vm = emplace_vertex();
	vm->position = (v1->position + v2->position) / 2.0f;
	interpolate_data({v1, v2}, vm); //set bone_weights

	EdgeRef e2 = emplace_edge();
	e2->sharp = e->sharp; //copy sharpness flag

	HalfedgeRef h2 = emplace_halfedge();
	interpolate_data({h, h->next}, h2); //set corner_uv, corner_normal

	HalfedgeRef t2 = emplace_halfedge();
	interpolate_data({t, t->next}, t2); //set corner_uv, corner_normal

	// The following elements aren't necessary for the bisect_edge, but they are here to demonstrate phase 4
    FaceRef f_not_used = emplace_face();
    HalfedgeRef h_not_used = emplace_halfedge();

	// Phase 3: Reassign connectivity (careful about ordering so you don't overwrite values you may need later!)

	vm->halfedge = h2;

	e2->halfedge = h2;

	assert(e->halfedge == h); //unchanged

	//n.b. h remains on the same face so even if h->face->halfedge == h, no fixup needed (t, similarly)

	h2->twin = t;
	h2->next = h->next;
	h2->vertex = vm;
	h2->edge = e2;
	h2->face = h->face;

	t2->twin = h;
	t2->next = t->next;
	t2->vertex = vm;
	t2->edge = e;
	t2->face = t->face;
	
	h->twin = t2;
	h->next = h2;
	assert(h->vertex == v1); // unchanged
	assert(h->edge == e); // unchanged
	//h->face unchanged

	t->twin = h2;
	t->next = t2;
	assert(t->vertex == v2); // unchanged
	t->edge = e2;
	//t->face unchanged


	// Phase 4: Delete unused elements
    erase_face(f_not_used);
    erase_halfedge(h_not_used);

	// Phase 5: Return the correct iterator
	return vm;
}


/*
 * split_edge: split an edge and adjacent (non-boundary) faces
 *  e: edge to split
 *
 * returns: added vertex. vertex->halfedge should lie along e
 *
 * Note that when splitting the adjacent faces, the new edge
 * should connect to the vertex ccw from the ccw-most end of e
 * within the face.
 *
 * Do not split adjacent boundary faces.
 */
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::split_edge(EdgeRef e) {
	// A2L2 (REQUIRED): split_edge

	// These are important for conditionals
	HalfedgeRef h = e->halfedge;
	HalfedgeRef t = h->twin;

	
	//handle boundary cases as if h is boundary
	if (h->face->boundary || t->face->boundary) {

		//this cheekily avoids doubling this code
		if (t->face->boundary) {
			t = e->halfedge;
			h = t->twin;
		}
		
		// create
		FaceRef f3 = emplace_face();
		FaceRef f1 = t->face;
		EdgeRef e1 = emplace_edge();
		EdgeRef e2 = emplace_edge();
		HalfedgeRef h1 = emplace_halfedge();
		HalfedgeRef t1 = emplace_halfedge();
		HalfedgeRef h2 = emplace_halfedge();
		HalfedgeRef t2 = emplace_halfedge();
		VertexRef c = emplace_vertex();
		c->position = e->center();
		//h twins t1, t twins h1;

		/*std::cout << "added f3 " + std::to_string(f3->id) + "\n";
		std::cout << "found f1 " + std::to_string(f1->id) + "\n";
		std::cout << "added e1 " + std::to_string(e1->id) + "\n";
		std::cout << "added e2 " + std::to_string(e2->id) + "\n";
		std::cout << "added h1 " + std::to_string(h1->id) + "\n";
		std::cout << "added t1 " + std::to_string(t1->id) + "\n";
		std::cout << "added h2 " + std::to_string(h2->id) + "\n";
		std::cout << "added t2 " + std::to_string(t2->id) + "\n";
		std::cout << "added c " + std::to_string(c->id) + "\n";*/
		
		//assign vertices
		t1->vertex = c;
		h1->vertex = c;
		h2->vertex = c;
		t2->vertex = t->next->next->vertex;
		c->halfedge = t1;

		//assign halfedges
		h1->next = h->next;
		h->next = h1;
		t1->next = t->next;
		t->next = h2;
		h->twin = t1;
		t->twin = h1;
		h1->twin = t;
		t1->twin = h;
		h2->next = t1->next->next;
		t1->next->next = t2;
		t2->next = t1;
		t2->twin = h2;
		h2->twin = t2;

		//assign edges
		t1->edge = e;
		t->edge = e1;
		h1->edge = e1;
		e->halfedge = h;
		e1->halfedge = t;
		t2->edge = e2;
		h2->edge = e2;
		e2->halfedge = h2;

		//assign faces
		f3->halfedge = t2;
		f1->halfedge = t;
		t1->face = f3;
		t1->next->face = f3;
		t2->face = f3;
		h2->face = f1;
		h1->face = h->face;

		return c;
	}
	
	FaceRef f1 = h->face;
	FaceRef f2 = t->face;
	FaceRef f3 = emplace_face();
	FaceRef f4 = emplace_face();
	EdgeRef e1 = emplace_edge();
	EdgeRef e2 = emplace_edge();
	EdgeRef e3 = emplace_edge();
	HalfedgeRef h1 = emplace_halfedge();
	HalfedgeRef t1 = emplace_halfedge();
	HalfedgeRef h2 = emplace_halfedge();
	HalfedgeRef t2 = emplace_halfedge();
	HalfedgeRef h3 = emplace_halfedge();
	HalfedgeRef t3 = emplace_halfedge();

	//again, h pairs t1, t pairs h1

	VertexRef c = emplace_vertex();
	c->position = e->center();
	/*std::cout << "found f1 " + std::to_string(f1->id) + "\n";
	std::cout << "found f2 " + std::to_string(f2->id) + "\n";
	std::cout << "added f3 " + std::to_string(f3->id) + "\n";
	std::cout << "added f3 " + std::to_string(f4->id) + "\n";
	std::cout << "found e " + std::to_string(e->id) + "\n";
	std::cout << "added e1 " + std::to_string(e1->id) + "\n";
	std::cout << "added e2 " + std::to_string(e2->id) + "\n";
	std::cout << "added e3 " + std::to_string(e3->id) + "\n";
	std::cout << "found h " + std::to_string(h->id) + "\n";
	std::cout << "found t " + std::to_string(t->id) + "\n";
	std::cout << "added h1 " + std::to_string(h1->id) + "\n";
	std::cout << "added t1 " + std::to_string(t1->id) + "\n";
	std::cout << "added h2 " + std::to_string(h2->id) + "\n";
	std::cout << "added t2 " + std::to_string(t2->id) + "\n";
	std::cout << "added h3 " + std::to_string(h3->id) + "\n";
	std::cout << "added t3 " + std::to_string(t3->id) + "\n";
	std::cout << "added c " + std::to_string(c->id) + "\n";*/

	//assign vertices
	h1->vertex = c;
	h2->vertex = c;
	t1->vertex = c;
	h3->vertex = c;
	t2->vertex = h->next->next->vertex;
	t3->vertex = t->next->next->vertex;
	c->halfedge = h1;

	//assign halfedges
	h->twin = t1;
	t1->twin = h;
	t->twin = h1;
	h1->twin = t;
	h2->twin = t2;
	t2->twin = h2;
	h3->twin = t3;
	t3->twin = h3;

	h1->next = h->next;
	t1->next = t->next;
	h2->next = h->next->next;
	h3->next = t->next->next;
	h->next->next = t2;
	t->next->next = t3;
	h->next = h2;
	t->next = h3;
	t2->next = h1;
	t3->next = t1;

	//assign edges
	e->halfedge = h;
	e1->halfedge = t;
	e2->halfedge = h2;
	e3->halfedge = h3;
	t1->edge = e;
	t->edge = e1;
	h1->edge = e1;
	h2->edge = e2;
	t2->edge = e2;
	h3->edge = e3;
	t3->edge = e3;

	//assign faces
	f1->halfedge = h;
	f2->halfedge = t;
	f3->halfedge = h1;
	f4->halfedge = t1;
	h2->face = f1;
	t2->face = f3;
	h1->face = f3;
	t1->face = f4;
	h3->face = f2;
	t3->face = f4;
	t1->next->face = f4;
	h1->next->face = f3;



	/*VertexRef v1 = h->vertex;
	VertexRef v2 = t->vertex;
	VertexRef v3 = h->next->vertex;
	VertexRef v4 = t->next->vertex;
	FaceRef f1 = h->face;
	FaceRef f2 = t->face;*/


	//(void)e; //this line avoids 'unused parameter' warnings. You can delete it as you fill in the function.
    return c;
}



/*
 * inset_vertex: divide a face into triangles by placing a vertex at f->center()
 *  f: the face to add the vertex to
 *
 * returns:
 *  std::nullopt if insetting a vertex would make mesh invalid
 *  the inset vertex otherwise
 */
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::inset_vertex(FaceRef f) {
	// A2Lx4 (OPTIONAL): inset vertex
	
	(void)f;
    return std::nullopt;
}


/* [BEVEL NOTE] Note on the beveling process:

	Each of the bevel_vertex, bevel_edge, and extrude_face functions do not represent
	a full bevel/extrude operation. Instead, they should update the _connectivity_ of
	the mesh, _not_ the positions of newly created vertices. In fact, you should set
	the positions of new vertices to be exactly the same as wherever they "started from."

	When you click on a mesh element while in bevel mode, one of those three functions
	is called. But, because you may then adjust the distance/offset of the newly
	beveled face, we need another method of updating the positions of the new vertices.

	This is where bevel_positions and extrude_positions come in: these functions are
	called repeatedly as you move your mouse, the position of which determines the
	amount / shrink parameters. These functions are also passed an array of the original
	vertex positions, stored just after the bevel/extrude call, in order starting at
	face->halfedge->vertex, and the original element normal, computed just *before* the
	bevel/extrude call.

	Finally, note that the amount, extrude, and/or shrink parameters are not relative
	values -- you should compute a particular new position from them, not a delta to
	apply.
*/

/*
 * bevel_vertex: creates a face in place of a vertex
 *  v: the vertex to bevel
 *
 * returns: reference to the new face
 *
 * see also [BEVEL NOTE] above.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_vertex(VertexRef v) {
	//A2Lx5 (OPTIONAL): Bevel Vertex
	// Reminder: This function does not update the vertex positions.
	// Remember to also fill in bevel_vertex_helper (A2Lx5h)

	(void)v;
    return std::nullopt;
}

/*
 * bevel_edge: creates a face in place of an edge
 *  e: the edge to bevel
 *
 * returns: reference to the new face
 *
 * see also [BEVEL NOTE] above.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_edge(EdgeRef e) {
	//A2Lx6 (OPTIONAL): Bevel Edge
	// Reminder: This function does not update the vertex positions.
	// remember to also fill in bevel_edge_helper (A2Lx6h)

	(void)e;
    return std::nullopt;
}

/*
 * extrude_face: creates a face inset into a face
 *  f: the face to inset
 *
 * returns: reference to the inner face
 *
 * see also [BEVEL NOTE] above.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::extrude_face(FaceRef f) {
	//A2L4: Extrude Face
	// Reminder: This function does not update the vertex positions.
	// Remember to also fill in extrude_helper (A2L4h)

	HalfedgeRef h = f->halfedge;
	//std::cout << "h " + std::to_string(h->id) + "\n";
	// create new vertex, set position to h->vertex
	VertexRef vStart = emplace_vertex();
	vStart->position = h->vertex->position;

	// create edge and halfedges connecting vertex
	EdgeRef eStart = emplace_edge();
	HalfedgeRef hStart = emplace_halfedge();
	HalfedgeRef tStart = emplace_halfedge();
	//std::cout << "hstart " + std::to_string(hStart->id) + "\n";
	//std::cout << "tstart " + std::to_string(tStart->id) + "\n";
	hStart->twin = tStart;
	tStart->twin = hStart;
	hStart->vertex = h->vertex;
	h->vertex->halfedge = h;
	tStart->vertex = vStart;
	vStart->halfedge = tStart;
	hStart->edge = eStart;
	tStart->edge = eStart;
	eStart->halfedge = hStart;
	tStart->next = h;
	//face DNE yet

	HalfedgeRef hprev = h;
	//HalfedgeRef hsprev = hStart;
	HalfedgeRef tprev = tStart; //Note that this t is not the twin of hprev
	HalfedgeRef tfprev = tStart; //this will be overrided later.
	HalfedgeRef tffirst;
	//FaceRef fprev = f;
	HalfedgeRef hnow = h->next;

	// loop over halfedges until we get back to h
	while (hnow != h) {
		VertexRef vnow = hnow->vertex;
		VertexRef vnew = emplace_vertex();
		vnew->position = vnow->position;
		//std::cout << "hnow " + std::to_string(hnow->id) + "\n";

		EdgeRef enew = emplace_edge();
		HalfedgeRef hnew = emplace_halfedge();
		HalfedgeRef tnew = emplace_halfedge();
		//std::cout << "hnew " + std::to_string(hnew->id) + "\n";
		//std::cout << "tnew " + std::to_string(tnew->id) + "\n";
		hnew->twin = tnew;
		tnew->twin = hnew;
		hnew->vertex = vnow;
		vnow->halfedge = hnew;
		tnew->vertex = vnew;
		vnew->halfedge = tnew;
		hnew->edge = enew;
		tnew->edge = enew;
		enew->halfedge = hnew;
		
		FaceRef fnew = emplace_face(); //old face is becoming new center face
		//std::cout << "fnew " + std::to_string(fnew->id) + "\n";
		EdgeRef ef = emplace_edge();
		HalfedgeRef hf = emplace_halfedge();
		HalfedgeRef tf = emplace_halfedge();
		//std::cout << "hf " + std::to_string(hf->id) + "\n";
		//std::cout << "tf " + std::to_string(tf->id) + "\n";
		hf->twin = tf;
		tf->twin = hf;
		hf->vertex = vnew;
		tf->vertex = tprev->vertex;
		hf->vertex->halfedge = hf;
		tf->vertex->halfedge = tf;
		hf->edge = ef;
		tf->edge = ef;
		ef->halfedge = hf;
		hf->next = tprev;
		tnew->next = hnow;
		hprev->next = hnew;
		//std::cout << "hprev " + std::to_string(hprev->id) + " points to " + std::to_string(hnew->id) + "\n";
		hnew->next = hf;
		
		if (tfprev != tStart) { // has to be done here because these references disappear in next loop
			tfprev->next = tf;
			//std::cout << "tf prev " + std::to_string(tfprev->id) + " points to " + std::to_string(tf->id) + "\n";
		}
		else {
			tffirst = tf;
		}
		tprev->face = fnew;
		hprev->face = fnew;
		//std::cout << "hprev " + std::to_string(hprev->id) + " face points to " + std::to_string(fnew->id) + "\n";
		hf->face = fnew;
		hnew->face = fnew;
		fnew->halfedge = hprev;

		tf->face = f;
		f->halfedge = tf;

		//update values before next round
		hprev = hnow;
		tprev = tnew;
		tfprev = tf;
		//fprev = fnew;
		//std::cout << "f " + std::to_string(fnew->id) + " halfedge points to " + std::to_string(fnew->halfedge->id) + "\n";
		//std::cout << "Looping...";
		hnow = hnow->next;
		// hnow->next= 
	}

	//std::cout << "Out.";
	// fill in last set of inner edges and faces.
	HalfedgeRef hflast = emplace_halfedge();
	HalfedgeRef tflast = emplace_halfedge();
	//std::cout << "tf! " + std::to_string(tflast->id) + "\n";
	EdgeRef elast = emplace_edge();
	FaceRef flast = emplace_face();
	//std::cout << "flast " + std::to_string(flast->id) + "\n";

	hflast->twin = tflast;
	tflast->twin = hflast;
	hflast->edge = elast;
	tflast->edge = elast;
	elast->halfedge = hflast;
	hflast->vertex = vStart;
	hflast->next = tprev;
	tflast->vertex = tprev->vertex;
	tfprev->next = tflast;
	hStart->next = hflast;
	//std::cout << "tf prev " + std::to_string(tfprev->id) + " points to " + std::to_string(tflast->id) + "\n";
	tflast->next = tffirst;
	hprev->next = hStart;
	//std::cout << "tf prev " + std::to_string(tflast->id) + " points to " + std::to_string(tffirst->id) + "\n";
	//h->face = flast;
	hprev->face = flast;
	//hStart->face = flast;
	hflast->face = flast;
	tprev->face = flast;
	hStart->face = flast;
	flast->halfedge = hprev;
	
	tflast->face = f;
	f->halfedge = tflast;

	//std::cout << "Extrude";
	//extrude_positions(f, Vec3(0.0f), 0.5f);

	//(void)f;
    return f;
}

/*
 * flip_edge: rotate non-boundary edge ccw inside its containing faces
 *  e: edge to flip
 *
 * if e is a boundary edge, does nothing and returns std::nullopt
 * if flipping e would create an invalid mesh, does nothing and returns std::nullopt
 *
 * otherwise returns the edge, post-rotation
 *
 * does not create or destroy mesh elements.
 */
std::optional<Halfedge_Mesh::EdgeRef> Halfedge_Mesh::flip_edge(EdgeRef e) {
	//A2L1: Flip Edge
	
	HalfedgeRef h = e->halfedge;
	HalfedgeRef t = h->twin;

	VertexRef v1 = h->next->vertex;
	VertexRef v2 = t->next->vertex;
	VertexRef v3 = h->next->next->vertex;
	VertexRef v4 = t->next->next->vertex;
	FaceRef f1 = h->face;
	FaceRef f2 = t->face;
	HalfedgeRef prevh = h->next;
	HalfedgeRef prevt = t->next;

	//finding prevh and prevt
	while (prevh->next != h) {
		prevh = prevh->next;
	}
	while (prevt->next != t) {
		prevt = prevt->next;
	}
	//std::cout << "h = " + std::to_string(h->id) + "; prevh = " + std::to_string(prevh->id) + "; h->next = " + std::to_string(h->next->id) + "\n";
	//std::cout << "t = " + std::to_string(t->id) + "; prevt = " + std::to_string(prevt->id) + "; t->next = " + std::to_string(t->next->id) + "\n";

	//case: boundary edge
	if (f1->boundary || f2->boundary) {
		return std::nullopt;
	}


	v1->halfedge = h->next;
	v2->halfedge = t->next;
	f1->halfedge = h;
	f2->halfedge = t;

	t->next->face = f1;
	prevh->next = t->next;
	h->next->face = f2;
	prevt->next = h->next;
	t->vertex = v3;
	h->vertex = v4;
	t->next = t->next->next;
	h->next = h->next->next;
	v1->halfedge->next = t;
	v2->halfedge->next = h;
	//std::cout << "h from " + std::to_string(h->vertex->id) + " to " + std::to_string(h->next->vertex->id) + "\n";
	//std::cout << "t from " + std::to_string(t->vertex->id) + " to " + std::to_string(t->next->vertex->id) + "\n";

	//f1->halfedge = h;
	//f2->halfedge = t;
	//h->set_tnvef(t, h->next->next, v4, e, f1);
	//t->set_tnvet(h, t->next->next, v3, e, f2);


    return e;
}


/*
 * make_boundary: add non-boundary face to boundary
 *  face: the face to make part of the boundary
 *
 * if face ends up adjacent to other boundary faces, merge them into face
 *
 * if resulting mesh would be invalid, does nothing and returns std::nullopt
 * otherwise returns face
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::make_boundary(FaceRef face) {
	//A2Lx7: (OPTIONAL) make_boundary

	return std::nullopt; //TODO: actually write this code!
}

/*
 * dissolve_vertex: merge non-boundary faces adjacent to vertex, removing vertex
 *  v: vertex to merge around
 *
 * if merging would result in an invalid mesh, does nothing and returns std::nullopt
 * otherwise returns the merged face
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::dissolve_vertex(VertexRef v) {
	// A2Lx1 (OPTIONAL): Dissolve Vertex

    return std::nullopt;
}

/*
 * dissolve_edge: merge the two faces on either side of an edge
 *  e: the edge to dissolve
 *
 * merging a boundary and non-boundary face produces a boundary face.
 *
 * if the result of the merge would be an invalid mesh, does nothing and returns std::nullopt
 * otherwise returns the merged face.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::dissolve_edge(EdgeRef e) {
	// A2Lx2 (OPTIONAL): dissolve_edge

	//Reminder: use interpolate_data() to merge corner_uv / corner_normal data
	
    return std::nullopt;
}

/* collapse_edge: collapse edge to a vertex at its middle
 *  e: the edge to collapse
 *
 * if collapsing the edge would result in an invalid mesh, does nothing and returns std::nullopt
 * otherwise returns the newly collapsed vertex
 */
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_edge(EdgeRef e) {
	//A2L3: Collapse Edge

	//Reminder: use interpolate_data() to merge corner_uv / corner_normal data on halfedges
	// (also works for bone_weights data on vertices!)

	auto applyToAllHalfedgesAroundVertex = [=](VertexRef v, auto&& f) {
		HalfedgeRef h = v->halfedge;
		HalfedgeRef t = h->twin;

		auto first = f(h) + f(t);
		HalfedgeRef ith = t->next;
		HalfedgeRef itt = ith->twin;
		while (ith != h) {
			//std::cout << std::to_string(first) + "\n";
			first = first + f(ith) + f(itt);
			ith = itt->next;
			itt = ith->twin;
		}
		return first;
		};

	HalfedgeRef h = e->halfedge;
	HalfedgeRef t = h->twin;
	// pretty sure this shouldn't happen
	if (h->face->boundary && t->face->boundary) {
		return std::nullopt;
	}


	// boundary case
	if (h->face->boundary || t->face->boundary) {
		if (t->face->boundary) {
			t = e->halfedge;
			h = t->twin;
		}
		//check for triangle
		if (t->next->next->next == t) {
			// solo triangle - don't collapse
			if ((t->next->twin->face->boundary) && (t->next->next->twin->face->boundary)) {

				return std::nullopt;
			}
			interpolate_data({ h->vertex, t->vertex }, h->vertex);
			h->vertex->position = e->center();
			h->vertex->halfedge = h;
			//shift h over one because easier
			HalfedgeRef toDelete = h->next;
			h->next->twin->twin = h;
			h->twin = h->next->twin;
			h->next->edge->halfedge = h;
			h->edge = h->next->edge;
			h->next = h->next->next;
			HalfedgeRef hkeep = t->next->twin;
			HalfedgeRef tkeep = t->next->next->twin;

			erase_face(t->face);
			//if tkeep is boundary, h->next is tkeep, deleting tkeep will ruin h.

			tkeep->twin = hkeep;
			hkeep->twin = tkeep;
			hkeep->edge->halfedge = hkeep;
			hkeep->vertex->halfedge = hkeep;
			tkeep->vertex = h->vertex;
			tkeep->vertex->halfedge = tkeep;
			tkeep->edge->halfedge = tkeep;
			erase_edge(tkeep->edge);
			tkeep->edge = hkeep->edge;
			erase_vertex(t->vertex);
			erase_edge(t->edge);

			if (h->next == tkeep) {
				h->twin = hkeep;
				hkeep->twin = h;
			}
			toDelete->vertex->halfedge = h;
			toDelete->face->halfedge = h;
			erase_halfedge(toDelete);
			erase_halfedge(t->next->next);
			erase_halfedge(t->next);
			//std::cout << "erasing halfedge " + std::to_string(t->id) + "\n";
			erase_halfedge(t);
			return h->vertex;
		}
		//non-tri on bound case
		interpolate_data({ h->vertex, t->vertex }, h->vertex);
		h->vertex->position = e->center();
		h->vertex->halfedge = h;

		EdgeRef oldEdge = h->edge;
		//VertexRef oldVert = h->next->vertex;
		HalfedgeRef deleteTNext = t->next;
		t->next->vertex->halfedge = t;
		t->next->edge->halfedge = t;
		t->next->face->halfedge = t;
		t->next->twin->twin = t;
		t->twin = t->next->twin;
		t->next = t->next->next;
		t->edge = t->twin->edge;
		erase_halfedge(deleteTNext);

		HalfedgeRef deleteHNext = h->next;
		h->next->vertex->halfedge = h;
		h->next->edge->halfedge = h;
		h->next->face->halfedge = h;
		h->next->twin->twin = h;
		h->twin = h->next->twin;
		h->next = h->next->next;
		h->edge = h->twin->edge;
		erase_halfedge(deleteHNext);

		erase_vertex(t->vertex);
		t->vertex = h->vertex;
		erase_edge(oldEdge);
		return h->vertex;

	}
	//no boundary on either side

	/*HalfedgeRef prevh = h->next;
	HalfedgeRef prevt = t->next;

	//finding prevh and prevt
	while (prevh->next != h) {
		prevh = prevh->next;
	}
	while (prevt->next != t) {
		prevt = prevt->next;
	}*/

	//hourglass edge case (also covers two floating triangle case)
	auto isBoundary = [=](HalfedgeRef in) {
		if (in->face->boundary) {
			return 1;
		}
		return 0;
		};
	int hbound = applyToAllHalfedgesAroundVertex(h->vertex, isBoundary);
	int tbound = applyToAllHalfedgesAroundVertex(t->vertex, isBoundary);
	if ((hbound > 0) && (tbound > 0)) {
		return std::nullopt;
	}

	// no more real edge cases, edge guaranteed to collapse
	interpolate_data({ h->vertex, t->vertex }, h->vertex);
	VertexRef vout = h->vertex;
	VertexRef tVert = t->vertex;
	h->vertex->position = e->center();
	h->vertex->halfedge = h;

	auto moveToh = [=, &vout, &tVert](HalfedgeRef in) {
		if (in->vertex == tVert) {
			in->vertex = vout;
			vout->halfedge = in;
			std::cout << "Routing " + std::to_string(in->id) + " to " + std::to_string(vout->id) + "\n";
		}
		return 0;
		};

	applyToAllHalfedgesAroundVertex(t->vertex, moveToh);

	// h is a triangle side
	if (h->next->next->next == h) {
		//std::cout << "h = " + std::to_string(h->id) + "; h->next = " + std::to_string(h->next->id) + "; h->next->next = " + std::to_string(h->next->next->id) + "\n";
		//std::cout << "h halfedge = " + std::to_string(vout->halfedge->id) + "\n";
		if (h->next->twin->face->boundary) {
			h->next->twin->twin = h->next->next->twin;
			h->next->next->twin->twin = h->next->twin;
			h->next->twin->edge = h->next->next->edge;
			//h->next->twin->face = h->next->next->twin->face;
			h->next->twin->vertex->halfedge = h->next->twin;
			h->next->twin->edge->halfedge = h->next->twin;
			h->next->twin->face->halfedge = h->next->twin;
			h->vertex->halfedge = h->next->twin;
			h->next->next->vertex->halfedge = h->next->next->twin->next;
			//std::cout << "h halfedge = " + std::to_string(vout->halfedge->id) + "\n";

			erase_face(h->face);
			erase_edge(h->next->edge);
			erase_halfedge(h->next->next);
			erase_halfedge(h->next);
			erase_halfedge(h);
			// skip erasing anything t related as this will be handled later
		}
		else {
			//if (h->next->next->twin->face->boundary) {
				h->next->twin->twin = h->next->next->twin;
				h->next->next->twin->twin = h->next->twin;

				h->next->next->twin->edge = h->next->edge;
				//h->next->next->twin->face = h->next->twin->face;
				h->next->next->twin->vertex->halfedge = h->next->next->twin;
				h->next->next->twin->edge->halfedge = h->next->next->twin;
				h->next->next->twin->face->halfedge = h->next->next->twin;
				h->vertex->halfedge = h->next->next->twin;
				h->next->next->vertex->halfedge = h->next->next->twin->next;

				erase_face(h->face);
				erase_edge(h->next->next->edge);
				erase_halfedge(h->next->next);
				erase_halfedge(h->next);
				erase_halfedge(h);
			/* }
			else {

			}*/
		}
	}
	// h has 3+ sides
	else {
		HalfedgeRef hn = h->next;
		//std::cout << "1t halfedge = " + std::to_string(h->next->vertex->halfedge->id) + "\n";
		h->next->vertex->halfedge = h;
		//std::cout << "2t halfedge = " + std::to_string(h->next->vertex->halfedge->id) + "\n";
		h->next->edge->halfedge = h;
		h->next->face->halfedge = h;
		h->vertex = h->next->vertex;
		h->edge = h->next->edge;
		h->twin = h->next->twin;
		h->next->twin->twin = h;
		h->next = h->next->next;
		h->next->next->vertex->halfedge = h->next->next->twin->next;

		erase_halfedge(hn);
	}


	// t is a triangle side
	if (t->next->next->next == t) {
		if (t->next->twin->face->boundary) {
			t->next->twin->twin = t->next->next->twin;
			t->next->next->twin->twin = t->next->twin;
			t->next->twin->edge = t->next->next->edge;
			//t->next->twin->face = t->next->next->twin->face;
			t->next->twin->vertex->halfedge = t->next->twin;
			t->next->twin->edge->halfedge = t->next->twin;
			t->next->twin->face->halfedge = t->next->twin;
			//t->vertex->halfedge = t->next->twin;
			t->next->next->vertex->halfedge = t->next->next->twin->next;

			erase_face(t->face);
			erase_edge(t->next->edge);
			erase_halfedge(t->next->next);
			erase_halfedge(t->next);
			erase_vertex(tVert);
			erase_edge(t->edge);
			erase_halfedge(t);
		}
		else {
			//if (h->next->next->twin->face->boundary) {
			t->next->twin->twin = t->next->next->twin;
			t->next->next->twin->twin = t->next->twin;

			t->next->next->twin->edge = t->next->edge;
			//t->next->next->twin->face = t->next->twin->face;
			t->next->next->twin->vertex->halfedge = t->next->next->twin;
			t->next->next->twin->edge->halfedge = t->next->next->twin;
			t->next->next->twin->face->halfedge = t->next->next->twin;
			//t->vertex->halfedge = t->next->next->twin;
			t->next->next->vertex->halfedge = t->next->next->twin->next;

			erase_face(t->face);
			erase_edge(t->next->next->edge);
			erase_halfedge(t->next->next);
			erase_halfedge(t->next);
			erase_vertex(tVert);
			erase_edge(t->edge);
			erase_halfedge(t);
			/* }
			else {

			}*/
		}
	}
	// t has 3+ sides
	else {
		HalfedgeRef tn = t->next;
		erase_edge(t->edge);
		erase_vertex(tVert);
		t->next->vertex->halfedge = t;
		t->next->edge->halfedge = t;
		t->next->face->halfedge = t;
		t->vertex = t->next->vertex;
		t->edge = t->next->edge;
		t->twin = t->next->twin;
		t->next->twin->twin = t;
		t->next = t->next->next;
		t->next->next->vertex->halfedge = t->next->next->twin->next;

		erase_halfedge(tn);
	}

    return vout;
}

/*
 * collapse_face: collapse a face to a single vertex at its center
 *  f: the face to collapse
 *
 * if collapsing the face would result in an invalid mesh, does nothing and returns std::nullopt
 * otherwise returns the newly collapsed vertex
 */
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_face(FaceRef f) {
	//A2Lx3 (OPTIONAL): Collapse Face

	//Reminder: use interpolate_data() to merge corner_uv / corner_normal data on halfedges
	// (also works for bone_weights data on vertices!)

    return std::nullopt;
}

/*
 * weld_edges: glue two boundary edges together to make one non-boundary edge
 *  e, e2: the edges to weld
 *
 * if welding the edges would result in an invalid mesh, does nothing and returns std::nullopt
 * otherwise returns e, updated to represent the newly-welded edge
 */
std::optional<Halfedge_Mesh::EdgeRef> Halfedge_Mesh::weld_edges(EdgeRef e, EdgeRef e2) {
	//A2Lx8: Weld Edges

	//Reminder: use interpolate_data() to merge bone_weights data on vertices!

    return std::nullopt;
}



/*
 * bevel_positions: compute new positions for the vertices of a beveled vertex/edge
 *  face: the face that was created by the bevel operation
 *  start_positions: the starting positions of the vertices
 *     start_positions[i] is the starting position of face->halfedge(->next)^i
 *  direction: direction to bevel in (unit vector)
 *  distance: how far to bevel
 *
 * push each vertex from its starting position along its outgoing edge until it has
 *  moved distance `distance` in direction `direction`. If it runs out of edge to
 *  move along, you may choose to extrapolate, clamp the distance, or do something
 *  else reasonable.
 *
 * only changes vertex positions (no connectivity changes!)
 *
 * This is called repeatedly as the user interacts, just after bevel_vertex or bevel_edge.
 * (So you can assume the local topology is set up however your bevel_* functions do it.)
 *
 * see also [BEVEL NOTE] above.
 */
void Halfedge_Mesh::bevel_positions(FaceRef face, std::vector<Vec3> const &start_positions, Vec3 direction, float distance) {
	//A2Lx5h / A2Lx6h (OPTIONAL): Bevel Positions Helper
	
	// The basic strategy here is to loop over the list of outgoing halfedges,
	// and use the preceding and next vertex position from the original mesh
	// (in the start_positions array) to compute an new vertex position.
	
}

/*
 * extrude_positions: compute new positions for the vertices of an extruded face
 *  face: the face that was created by the extrude operation
 *  move: how much to translate the face
 *  shrink: amount to linearly interpolate vertices in the face toward the face's centroid
 *    shrink of zero leaves the face where it is
 *    positive shrink makes the face smaller (at shrink of 1, face is a point)
 *    negative shrink makes the face larger
 *
 * only changes vertex positions (no connectivity changes!)
 *
 * This is called repeatedly as the user interacts, just after extrude_face.
 * (So you can assume the local topology is set up however your extrude_face function does it.)
 *
 * Using extrude face in the GUI will assume a shrink of 0 to only extrude the selected face
 * Using bevel face in the GUI will allow you to shrink and increase the size of the selected face
 * 
 * see also [BEVEL NOTE] above.
 */
void Halfedge_Mesh::extrude_positions(FaceRef face, Vec3 move, float shrink) {
	//A2L4h: Extrude Positions Helper

	//General strategy:
	// use mesh navigation to get starting positions from the surrounding faces,
	// compute the centroid from these positions + use to shrink,
	// offset by move
	//std::cout << "In";
	HalfedgeRef hstart = face->halfedge;
	HalfedgeRef hnow = hstart->next;
	Vec3 c = hstart->vertex->position;
	//std::cout << "(" + std::to_string(c.x) + ", " + std::to_string(c.y) + ", " + std::to_string(c.z) + ")\n";
	int sides = 1;
	while (hnow != hstart) {
		//std::cout << "hnow " + std::to_string(hnow->id);
		//std::cout << "Adding: (" + std::to_string(hnow->vertex->position.x) + ", " + std::to_string(hnow->vertex->position.y) + ", " + std::to_string(hnow->vertex->position.z) + ")\n";
		c = hnow->vertex->position + c;
		sides += 1;
		hnow = hnow->next;
		//std::cout << "(" + std::to_string(c.x) + ", " + std::to_string(c.y) + ", " + std::to_string(c.z) + ")\n";
	}
	//std::cout << "Sides";
	c = c / ((float)sides);
	Vec3 diff = c - hnow->vertex->position;
	diff *= shrink;
	diff += move;
	hnow->vertex->position = hnow->vertex->position + diff;
	hnow = hnow->next;
	while (hnow != hstart) {
		//std::cout << "Reassigning.";
		diff = c - hnow->vertex->position;
		diff *= shrink;
		diff += move;
		hnow->vertex->position = hnow->vertex->position + diff;
		hnow = hnow->next;
	}
	//return face;

}

