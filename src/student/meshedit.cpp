
#include <iostream>
#include <queue>
#include <set>
#include <unordered_map>

#include "../geometry/halfedge.h"
#include "debug.h"

/* Note on local operation return types:

    The local operations all return a std::optional<T> type. This is used so that your
    implementation can signify that it does not want to perform the operation for
    whatever reason (e.g. you don't want to allow the user to erase the last vertex).

    An optional can have two values: std::nullopt, or a value of the type it is
    parameterized on. In this way, it's similar to a pointer, but has two advantages:
    the value it holds need not be allocated elsewhere, and it provides an API that
    forces the user to check if it is null before using the value.

    In your implementaiton, if you have successfully performed the operation, you can
    simply return the required reference:

            ... collapse the edge ...
            return collapsed_vertex_ref;

    And if you wish to deny the operation, you can return the null optional:

            return std::nullopt;

    Note that the stubs below all reject their duties by returning the null optional.
*/

std::list<Halfedge_Mesh::HalfedgeRef> Halfedge_Mesh::Vertex::get_halfedges() {
    auto v_he = std::list<HalfedgeRef>();
    auto iter = this->halfedge();
    do {
        v_he.push_back(iter);
        iter = iter->twin()->next();
    } while(iter->id() != this->halfedge()->id());
    return v_he;
}

std::list<Halfedge_Mesh::FaceRef> Halfedge_Mesh::Vertex::get_faces()
{
    const auto v_he = this->get_halfedges();
    auto v_f = std::list<FaceRef>();
    for (const auto halfedge : v_he)
    {
        v_f.push_back(halfedge->face());
    }
    return v_f;
}

std::list<Halfedge_Mesh::EdgeRef> Halfedge_Mesh::Vertex::get_edges() {
    const auto v_he = this->get_halfedges();
    auto v_e = std::list<EdgeRef>();
    for(const auto halfedge : v_he) {
        v_e.push_back(halfedge->edge());
    }
    return v_e;
}
            

Halfedge_Mesh::VertexRef Halfedge_Mesh::Halfedge::endVertex()
{
    return this->twin()->vertex();
}


Vec3 Halfedge_Mesh::Halfedge::vector()
{
    return this->endVertex()->pos - this->vertex()->pos;
}

Vec3 Halfedge_Mesh::Halfedge::dir()
{
    return this->vector().normalize();
}


Halfedge_Mesh::HalfedgeRef Halfedge_Mesh::Halfedge::prev() {
    auto iter = _twin;
    while(iter->next()->id() != this->id()) {
        iter = iter->next()->twin();
    }
    return iter;
}

Halfedge_Mesh::HalfedgeCRef Halfedge_Mesh::Halfedge::prev() const {
    auto iter = _twin;
    while(iter->next()->id() != this->id()) {
        iter = iter->next()->twin();
    }
    return iter;
}

/*
 Creates a pair of halfedges from the start vertex to the end vertex, returns the corresponding EdgeRef.
 The associated halfedge of the EdgeRef start at the start vertex.
*/
Halfedge_Mesh::EdgeRef Halfedge_Mesh::new_halfedge_pair(VertexRef start, VertexRef end) {
    auto newE = this->new_edge();
    auto newHe1 = this->new_halfedge();
    auto newHe2 = this->new_halfedge();
    newE->_halfedge = newHe1;
    newHe1->_vertex = start;
    newHe2->_vertex = end;
    newHe1->_twin = newHe2;
    newHe2->_twin = newHe1;
    newHe1->_edge = newE;
    newHe2->_edge = newE;
    return newE;
}

/*
    This method should replace the given vertex and all its neighboring
    edges and faces with a single face, returning the new face.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::erase_vertex(Halfedge_Mesh::VertexRef v) {
    
    auto he = v->halfedge();
    auto fKeep = he->face();
    auto iter = he;

    // probably shouldn't delete vertices on the border
    if(v->on_boundary()) return std::nullopt;
    // also don't delete if there's any neighbouring 2-valence vertices (no dangling edge)
    do
    {
        if(iter->twin()->vertex()->degree() < 3) return std::nullopt;
        iter->next()->vertex()->halfedge() = iter->next();
    }
    while (iter->id() != he->id());

    // link up the edges surrounding the kept face
    iter = he;
    do {
        iter->twin()->prev()->next() = iter->next();
        iter = iter->twin()->next();
        iter->next()->vertex()->halfedge() = iter->next();
    } while(iter->id() != he->id());

    // point the edges to the kept face
    iter = he->next();
    do
    {
        iter->face() = fKeep;
        iter = iter->next();
    }
    while (iter->id() != he->next()->id());
    fKeep->halfedge() = he->next();

    // delete the faces, edges, halfedges and vertex
    iter = he;
    do {
        this->erase(iter);
        this->erase(iter->twin());
        this->erase(iter->edge());
        if(iter->face()->id() != fKeep->id()) {
            this->erase(iter->face());
        }
        iter = iter->twin()->next();
    } while(iter->id() != he->id());
    this->erase(v);
    this->validate();
    return fKeep;
	// (void)v;
    // return std::nullopt;
}

/*
    This method should erase the given edge and return an iterator to the
    merged face.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::erase_edge(Halfedge_Mesh::EdgeRef e, bool validate) {

    if(e->halfedge()->is_boundary()) return std::nullopt; // no border
    if(e->halfedge()->vertex()->degree() < 3 || e->halfedge()->twin()->vertex()->degree() < 3)
        return std::nullopt; // no dangling edge

    auto he = e->halfedge();
    auto the = he->twin();
    auto fKeep = he->face();
    auto fErase = the->face();
    auto next = he->next();
    auto prev = he->prev();
    auto tnext = the->next();
    auto tprev = the->prev();
    auto startV = he->vertex();
    auto endV = the->vertex();

    startV->halfedge() = tnext;
    endV->halfedge() = next;
    fKeep->halfedge() = next;
    prev->next() = tnext;
    tprev->next() = next;

    auto iter = tnext;
    do
    {
        iter->face() = fKeep;
        iter = iter->next();
    }
    while (iter->id() != tnext->id());

    this->erase(he);
    this->erase(the);
    this->erase(e);
    this->erase(fErase);
    if (validate) this->validate();

    return fKeep;

    // (void)e;
    // return std::nullopt;
}

bool Halfedge_Mesh::check_edge_collapse_eligibility(EdgeRef e)
{
    auto he = e->halfedge(), the = he->twin();
    if(!e->on_boundary() && he->vertex()->on_boundary() && the->vertex()->on_boundary()) {
        std::cout
            << "Edge is not on the boundary but both vertices of the edge are on the boundary\n";
        return false;
    }
    // do not collapse a pure edge
    if(he->is_boundary() && the->is_boundary()) {
        std::cout << "Pure Edge\n";
        return false;
    }
    // do not collapse a single triangle
    if((he->is_boundary() && !the->is_boundary() && the->face()->degree() == 3) ||
       (the->is_boundary() && !he->is_boundary() && he->face()->degree() == 3)) {
        std::cout << "single triangle\n";
        return false;
    }
    // do not collapse triangles if the vertex not on the edge is divalent (creates dangling edge)
    if((he->face()->degree() == 3 && he->next()->next()->vertex()->degree() == 2) ||
       (the->face()->degree() == 3 && the->next()->next()->vertex()->degree() == 2)) {
        std::cout << "Divalent Edge, he/the: " << he->next()->next()->vertex()->degree() << ' '
                  << the->next()->next()->vertex()->degree() << "\n";
        return false;
    }

	int allowed_shared_neighbours = 0;
    if(!he->is_boundary() && he->face()->degree() == 3) allowed_shared_neighbours++;
    if(!the->is_boundary() && the->face()->degree() == 3) allowed_shared_neighbours++;

    auto he_start_halfedges = he->vertex()->get_halfedges();
    auto the_start_halfedges = the->vertex()->get_halfedges();
    int count = 0;
    for(auto he_start_halfedge : he_start_halfedges) {
        for(auto the_start_halfedge : the_start_halfedges) {
            if(he_start_halfedge->twin()->vertex()->id() ==
               the_start_halfedge->twin()->vertex()->id()) {
                count++;
            }
            if(count > allowed_shared_neighbours) {
                std::cout << count << " shared neighbours when allowed "
                          << allowed_shared_neighbours << '\n';
                return false;
            }
        }
    }
    return true;
}


/*
    This method should collapse the given edge and return an iterator to
    the new vertex created by the collapse.
    The following implementation can handle the collapse of boundary edges.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_edge(Halfedge_Mesh::EdgeRef e, bool validate) {
    if(!this->check_edge_collapse_eligibility(e)) {
        return std::nullopt;
    }
    auto he = e->halfedge(), the = he->twin();
    // if face is triangular, do an edge erase first to merge it with an adjacent face
    if(he->face()->degree() == 3) {
        if(!he->next()->edge()->on_boundary())
            this->erase_edge(he->next()->edge(), false);
        else
            this->erase_edge(he->next()->next()->edge(), false);
    }
    if(the->face()->degree() == 3) {
        if(!the->next()->edge()->on_boundary())
            this->erase_edge(the->next()->edge(), false);
        else
            this->erase_edge(the->next()->next()->edge(), false);
    }

    auto fLeft = he->face(), fRight = the->face();
    auto vStartKeep = he->vertex(), vEndErase = the->vertex();
    auto next = he->next();
    auto prev = he->prev();
    auto tnext = the->next();
    auto tprev = the->prev();

    // reconnect all halfedges associated with vEndErase to vStartRemove
    
    auto iter = the;
    do
    {
        iter->vertex() = vStartKeep;
        iter = iter->twin()->next();
    }
    while (iter->id() != the->id());
    
    vStartKeep->halfedge() = next;
    
    // bypass the edge
    prev->next() = next;
    tprev->next() = tnext;

    // set halfedge of faces
    fLeft->halfedge() = next;
    fRight->halfedge() = tnext;

    // set position
    vStartKeep->pos = (vEndErase->pos + vStartKeep->pos) / 2;

    // erase the halfedges, edge and vertex;
    this->erase(he);
    this->erase(the);
    this->erase(he->edge());
    this->erase(vEndErase);
    if (validate) this->validate();
    return vStartKeep;


    // (void)e;
    // return std::nullopt;
}

/*
    This method should collapse the given face and return an iterator to
    the new vertex created by the collapse.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_face(Halfedge_Mesh::FaceRef f) {

    (void)f;
    return std::nullopt;
}

/*
    This method should flip the given edge and return an iterator to the
    flipped edge.
*/
std::optional<Halfedge_Mesh::EdgeRef> Halfedge_Mesh::flip_edge(Halfedge_Mesh::EdgeRef e) {

    if(e->on_boundary()) return std::nullopt;
    auto he = e->halfedge();
    auto the = he->twin();
    auto next = he->next();
    auto tnext = the->next();
    auto prev = he->prev();
    auto tprev = the->prev();
    auto next2 = next->next();
    auto tnext2 = tnext->next();

    auto startV = he->vertex();
    auto endV = the->vertex();

    he->_vertex = tnext2->vertex(); 
    the->_vertex = next2->vertex();

    tprev->_next = next;
    next->_next = the;
    the->_next = tnext2;
    prev->_next = tnext;
    tnext->_next = he;
    he->_next = next2;
    
    if(startV->halfedge()->id() == he->id()) {
        startV->_halfedge = tnext;
    }
    if(endV->halfedge()->id() == the->id()) {
        endV->_halfedge = next;
    }

    if(he->face()->halfedge()->id() == next->id()) {
        he->face()->_halfedge = he;
    }
    if(the->face()->halfedge()->id() == tnext->id()) {
        the->face()->_halfedge = the;
    }

    next->_face = the->face();
    tnext->_face = he->face();

    this->validate();
    return e;
}

/*
    This method should split the given edge and return an iterator to the
    newly inserted vertex. The halfedge of this vertex should point along
    the edge that was split, rather than the new edges.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::split_edge(Halfedge_Mesh::EdgeRef e) {
    if(e->halfedge()->face()->degree() != 3 || e->halfedge()->twin()->face()->degree() != 3) {
        return std::nullopt; // split edge should only work on triangle meshes
    }
    auto he = e->halfedge();
    auto the = he->twin();
    auto prev = he->prev();
    auto next = he->next();
    auto tprev = the->prev();
    auto tnext = the->next();

    auto startV = he->vertex();
    auto endV = the->vertex();
    auto leftV = prev->vertex();
    auto rightV = tprev->vertex();
    auto leftF = he->face(), rightF = the->face();

    auto newV = this->new_vertex();
    he->_vertex = newV;
    newV->_halfedge = he;
    startV->_halfedge = tnext;
    newV->pos = (startV->pos + endV->pos + leftV->pos + rightV->pos) / 4;
    auto newEV = this->new_halfedge_pair(startV, newV);
    auto newEL = this->new_halfedge_pair(leftV, newV);
    auto newER = this->new_halfedge_pair(rightV, newV);
    auto newFL = this->new_face();
    auto newFR = this->new_face();

    newEV->halfedge()->_next = newEL->halfedge()->twin();
    newEL->halfedge()->_next = he;
    newER->halfedge()->_next = newEV->halfedge()->twin();

    newEV->halfedge()->twin()->_next = tnext;
    newEL->halfedge()->twin()->_next = prev;
    newER->halfedge()->twin()->_next = tprev;

    the->_next = newER->halfedge()->twin();
    prev->_next = newEV->halfedge();

    next->_next = newEL->halfedge();
    tnext->_next = newER->halfedge();

    rightF->_halfedge = the;
    leftF->_halfedge = he;
    newFR->_halfedge = newEV->halfedge()->twin();
    newFL->_halfedge = newEV->halfedge();

    FaceRef allFaces[] = {rightF, leftF, newFR, newFL};

    for (auto& f : allFaces)
    {
	    auto iter = f->halfedge();
        do
        {
            iter->_face = f;
            iter = iter->next();
        }
        while (iter->id() != f->halfedge()->id());
    }
    // std::cout << "yeah time to validate" << std::endl;
    this->validate();
    return newV;

    // (void)e;
    // return std::nullopt;
}

std::optional<Halfedge_Mesh::HalfedgeRef> Halfedge_Mesh::split_face(HalfedgeRef start,
                                                                    HalfedgeRef end, bool validate) {
    if(start->face()->id() != end->face()->id() || start->face()->is_boundary())
        return std::nullopt;
    if(start->next()->id() == end->id() || end->next()->id() == start->id() || start->id() == end->id()) 
        return std::nullopt;
    auto oldF = start->face(), newF = this->new_face();
    auto newEdge = this->new_halfedge_pair(start->vertex(), end->vertex());
    start->prev()->next() = newEdge->halfedge();
    newEdge->halfedge()->next() = end;
    end->prev()->next() = newEdge->halfedge()->twin();
    newEdge->halfedge()->twin()->next() = start;
    newEdge->halfedge()->face() = oldF;
    oldF->halfedge() = newEdge->halfedge();
    newEdge->halfedge()->twin()->face() = newF;
    auto iter = newEdge->halfedge()->twin();
    newF->halfedge() = iter;
    do
    {
        iter->face() = newF;
        iter = iter->next();
    }
    while (iter->id() != newEdge->halfedge()->twin()->id());
    if (validate) this->validate();
    return newEdge->halfedge();

}


/* Note on the beveling process:

    Each of the bevel_vertex, bevel_edge, and bevel_face functions do not represent
    a full bevel operation. Instead, they should update the _connectivity_ of
    the mesh, _not_ the positions of newly created vertices. In fact, you should set
    the positions of new vertices to be exactly the same as wherever they "started from."

    When you click on a mesh element while in bevel mode, one of those three functions
    is called. But, because you may then adjust the distance/offset of the newly
    beveled face, we need another method of updating the positions of the new vertices.

    This is where bevel_vertex_positions, bevel_edge_positions, and
    bevel_face_positions come in: these functions are called repeatedly as you
    move your mouse, the position of which determins the normal and tangent offset
    parameters. These functions are also passed an array of the original vertex
    positions: for  bevel_vertex, it has one element, the original vertex position,
    for bevel_edge,  two for the two vertices, and for bevel_face, it has the original
    position of each vertex in halfedge order. You should use these positions, as well
    as the normal and tangent offset fields to assign positions to the new vertices.

    Finally, note that the normal and tangent offsets are not relative values - you
    should compute a particular new position from them, not a delta to apply.
*/

/*
    This method should replace the vertex v with a face, corresponding to
    a bevel operation. It should return the new face.  NOTE: This method is
    responsible for updating the *connectivity* of the mesh only---it does not
    need to update the vertex positions.  These positions will be updated in
    Halfedge_Mesh::bevel_vertex_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_vertex(Halfedge_Mesh::VertexRef v) {

    // Reminder: You should set the positions of new vertices (v->pos) to be exactly
    // the same as wherever they "started from."

    (void)v;
    return std::nullopt;
}

/*
    This method should replace the edge e with a face, corresponding to a
    bevel operation. It should return the new face. NOTE: This method is
    responsible for updating the *connectivity* of the mesh only---it does not
    need to update the vertex positions.  These positions will be updated in
    Halfedge_Mesh::bevel_edge_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_edge(Halfedge_Mesh::EdgeRef e) {

    // Reminder: You should set the positions of new vertices (v->pos) to be exactly
    // the same as wherever they "started from."

    (void)e;
    return std::nullopt;
}

/*
    This method should replace the face f with an additional, inset face
    (and ring of faces around it), corresponding to a bevel operation. It
    should return the new face.  NOTE: This method is responsible for updating
    the *connectivity* of the mesh only---it does not need to update the vertex
    positions. These positions will be updated in
    Halfedge_Mesh::bevel_face_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_face(Halfedge_Mesh::FaceRef f) {

    // Reminder: You should set the positions of new vertices (v->pos) to be exactly
    // the same as wherever they "started from."
    auto oldVertices = std::vector<VertexRef>();
    auto newVertices = std::vector<VertexRef>();
    auto oldFaceHalfedges = std::vector<HalfedgeRef>();
    auto newFaceEdges = std::vector<EdgeRef>();
    auto newLinkEdges = std::vector<EdgeRef>();
    auto newFaces = std::vector<FaceRef>();
    auto iter = f->halfedge();
    // auto centerFace = this->new_face();
    auto fdeg = f->degree();
    for(unsigned int i = 0; i < fdeg; i++) {
        newVertices.push_back(this->new_vertex());
        oldVertices.push_back(iter->vertex());
        newVertices.back()->pos = oldVertices.back()->pos;
        oldFaceHalfedges.push_back(iter);
        newFaces.push_back(this->new_face());
        newFaces.back()->halfedge() = iter;
        iter->face() = newFaces.back();
        iter = iter->next();
    }
    for(unsigned int i = 0; i < fdeg; i++) {
        newLinkEdges.push_back(this->new_halfedge_pair(oldVertices[i], newVertices[i]));
        newFaceEdges.push_back(
            this->new_halfedge_pair(newVertices[i], newVertices[(i + 1) % fdeg]));
        newVertices[i]->halfedge() = newFaceEdges[i]->halfedge();
    }
    for(unsigned int i = 0; i < fdeg; i++) {
        oldFaceHalfedges[i]->next() = newLinkEdges[(i + 1) % fdeg]->halfedge();
        newLinkEdges[(i + 1) % fdeg]->halfedge()->face() = newFaces[i];
        newLinkEdges[(i + 1) % fdeg]->halfedge()->next() = newFaceEdges[i]->halfedge()->twin();
        newFaceEdges[i]->halfedge()->face() = f;
        newFaceEdges[i]->halfedge()->next() = newFaceEdges[(i + 1) % fdeg]->halfedge();
        newFaceEdges[i]->halfedge()->twin()->face() = newFaces[i];
        newFaceEdges[i]->halfedge()->twin()->next() = newLinkEdges[i]->halfedge()->twin();
        newLinkEdges[i]->halfedge()->twin()->face() = newFaces[i];
        newLinkEdges[i]->halfedge()->twin()->next() = oldFaceHalfedges[i];
    }
    f->halfedge() = newFaceEdges[0]->halfedge();
    this->validate();
    return f;
    // (void)f;
    // return std::nullopt;
}

/*
    Compute new vertex positions for the vertices of the beveled vertex.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the original vertex position and its associated outgoing edge
    to compute a new vertex position along the outgoing edge.
*/
void Halfedge_Mesh::bevel_vertex_positions(const std::vector<Vec3>& start_positions,
                                           Halfedge_Mesh::FaceRef face, float tangent_offset) {

    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while(h != face->halfedge());

    (void)new_halfedges;
    (void)start_positions;
    (void)face;
    (void)tangent_offset;
}

/*
    Compute new vertex positions for the vertices of the beveled edge.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the preceding and next vertex position from the original mesh
    (in the orig array) to compute an offset vertex position.

    Note that there is a 1-to-1 correspondence between halfedges in
    newHalfedges and vertex positions
    in orig.  So, you can write loops of the form

    for(size_t i = 0; i < new_halfedges.size(); i++)
    {
            Vector3D pi = start_positions[i]; // get the original vertex
            position corresponding to vertex i
    }
*/
void Halfedge_Mesh::bevel_edge_positions(const std::vector<Vec3>& start_positions,
                                         Halfedge_Mesh::FaceRef face, float tangent_offset) {

    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while(h != face->halfedge());

    (void)new_halfedges;
    (void)start_positions;
    (void)face;
    (void)tangent_offset;
}

/*
    Compute new vertex positions for the vertices of the beveled face.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the preceding and next vertex position from the original mesh
    (in the start_positions array) to compute an offset vertex
    position.

    Note that there is a 1-to-1 correspondence between halfedges in
    new_halfedges and vertex positions
    in orig. So, you can write loops of the form

    for(size_t i = 0; i < new_halfedges.size(); i++)
    {
            Vec3 pi = start_positions[i]; // get the original vertex
            position corresponding to vertex i
    }
*/
void Halfedge_Mesh::bevel_face_positions(const std::vector<Vec3>& start_positions,
                                         Halfedge_Mesh::FaceRef face, float tangent_offset,
                                         float normal_offset) {

    if(flip_orientation) normal_offset = -normal_offset;
    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while(h != face->halfedge());

    auto normal_vector = face->normal();
    for(unsigned long long i = 0; i < start_positions.size(); i++) {
        Vec3 tangent_vector = (face->center() - start_positions[i]).normalize();
        // unsigned long long prev = (i + start_positions.size() - 1) % start_positions.size();
        // unsigned long long next = (i + 1) % start_positions.size();
        new_halfedges[i]->vertex()->pos = start_positions[i] + normal_vector * normal_offset + tangent_vector * tangent_offset;
    }

    // (void)new_halfedges;
    // (void)start_positions;
    // (void)face;
    // (void)tangent_offset;
    // (void)normal_offset;
}

/*
    Splits all non-triangular faces into triangles.
*/
void Halfedge_Mesh::triangulate() {
    auto oldFaces = this->faces;
    for (auto old_face : oldFaces)
    {
        auto iter = old_face.halfedge();
        while (iter->face()->degree() > 3)
        {
            auto newIter = iter->next()->next();
            this->split_face(iter, newIter, false);
            iter = newIter;
        }
    }
    this->validate();
    // For each face...
}

/* Note on the quad subdivision process:

        Unlike the local mesh operations (like bevel or edge flip), we will perform
        subdivision by splitting *all* faces into quads "simultaneously."  Rather
        than operating directly on the halfedge data structure (which as you've
        seen is quite difficult to maintain!) we are going to do something a bit nicer:
           1. Create a raw list of vertex positions and faces (rather than a full-
              blown halfedge mesh).
           2. Build a new halfedge mesh from these lists, replacing the old one.
        Sometimes rebuilding a data structure from scratch is simpler (and even
        more efficient) than incrementally modifying the existing one.  These steps are
        detailed below.

  Step I: Compute the vertex positions for the subdivided mesh.
        Here we're going to do something a little bit strange: since we will
        have one vertex in the subdivided mesh for each vertex, edge, and face in
        the original mesh, we can nicely store the new vertex *positions* as
        attributes on vertices, edges, and faces of the original mesh. These positions
        can then be conveniently copied into the new, subdivided mesh.
        This is what you will implement in linear_subdivide_positions() and
        catmullclark_subdivide_positions().

  Steps II-IV are provided (see Halfedge_Mesh::subdivide()), but are still detailed
  here:

  Step II: Assign a unique index (starting at 0) to each vertex, edge, and
        face in the original mesh. These indices will be the indices of the
        vertices in the new (subdivided mesh).  They do not have to be assigned
        in any particular order, so long as no index is shared by more than one
        mesh element, and the total number of indices is equal to V+E+F, i.e.,
        the total number of vertices plus edges plus faces in the original mesh.
        Basically we just need a one-to-one mapping between original mesh elements
        and subdivided mesh vertices.

  Step III: Build a list of quads in the new (subdivided) mesh, as tuples of
        the element indices defined above. In other words, each new quad should be
        of the form (i,j,k,l), where i,j,k and l are four of the indices stored on
        our original mesh elements.  Note that it is essential to get the orientation
        right here: (i,j,k,l) is not the same as (l,k,j,i).  Indices of new faces
        should circulate in the same direction as old faces (think about the right-hand
        rule).

  Step IV: Pass the list of vertices and quads to a routine that clears
        the internal data for this halfedge mesh, and builds new halfedge data from
        scratch, using the two lists.
*/

/*
    Compute new vertex positions for a mesh that splits each polygon
    into quads (by inserting a vertex at the face midpoint and each
    of the edge midpoints).  The new vertex positions will be stored
    in the members Vertex::new_pos, Edge::new_pos, and
    Face::new_pos.  The values of the positions are based on
    simple linear interpolation, e.g., the edge midpoints and face
    centroids.
*/
void Halfedge_Mesh::linear_subdivide_positions() {

    // For each vertex, assign Vertex::new_pos to
    // its original position, Vertex::pos.

    // For each edge, assign the midpoint of the two original
    // positions to Edge::new_pos.

    // For each face, assign the centroid (i.e., arithmetic mean)
    // of the original vertex positions to Face::new_pos. Note
    // that in general, NOT all faces will be triangles!

    for (auto&& vertex : this->vertices)
    {
        vertex.new_pos = vertex.pos;
    }
    for (auto && edge : this->edges)
    {
        edge.new_pos = edge.center();
    }
    for (auto && face : this->faces)
    {
        face.new_pos = face.center();
    }

}

/*
    Compute new vertex positions for a mesh that splits each polygon
    into quads (by inserting a vertex at the face midpoint and each
    of the edge midpoints).  The new vertex positions will be stored
    in the members Vertex::new_pos, Edge::new_pos, and
    Face::new_pos.  The values of the positions are based on
    the Catmull-Clark rules for subdivision.

    Note: this will only be called on meshes without boundary
*/
void Halfedge_Mesh::catmullclark_subdivide_positions() {

    // The implementation for this routine should be
    // a lot like Halfedge_Mesh:linear_subdivide_positions:(),
    // except that the calculation of the positions themsevles is
    // slightly more involved, using the Catmull-Clark subdivision
    // rules. (These rules are outlined in the Developer Manual.)

    // Faces
    for(auto&& face : this->faces) {
        face.new_pos = face.center();
    }
    // Edges
    for(auto&& edge : this->edges) {
        edge.new_pos = edge.center() / 2 + (edge.halfedge()->face()->new_pos +
                                            edge.halfedge()->twin()->face()->new_pos) / 4;
    }
    for(auto&& vertex : this->vertices) {
        Vec3 q = Vec3(0, 0, 0);
        Vec3 r = Vec3(0, 0, 0);
        auto iter = vertex.halfedge();
        do
        {
            q += iter->face()->new_pos;
            r += iter->edge()->center();
            iter = iter->twin()->next();
        }
        while (iter->id() != vertex.halfedge()->id());
        float n = static_cast<float>(vertex.degree());
        vertex.new_pos = (q / n + 2 * r / n + (n - 3) * vertex.pos) / n;
    }
    // Vertices
}

/*
        This routine should increase the number of triangles in the mesh
        using Loop subdivision. Note: this is will only be called on triangle meshes.
*/
void Halfedge_Mesh::loop_subdivide() {

    // Compute new positions for all the vertices in the input mesh, using
    // the Loop subdivision rule, and store them in Vertex::new_pos.
    // -> At this point, we also want to mark each vertex as being a vertex of the
    //    original mesh. Use Vertex::is_new for this.
    // -> Next, compute the updated vertex positions associated with edges, and
    //    store it in Edge::new_pos.
    // -> Next, we're going to split every edge in the mesh, in any order.  For
    //    future reference, we're also going to store some information about which
    //    subdivided edges come from splitting an edge in the original mesh, and
    //    which edges are new, by setting the flat Edge::is_new. Note that in this
    //    loop, we only want to iterate over edges of the original mesh.
    //    Otherwise, we'll end up splitting edges that we just split (and the
    //    loop will never end!)
    // -> Now flip any new edge that connects an old and new vertex.
    // -> Finally, copy the new vertex positions into final Vertex::pos.

    // Each vertex and edge of the original surface can be associated with a
    // vertex in the new (subdivided) surface.
    // Therefore, our strategy for computing the subdivided vertex locations is to
    // *first* compute the new positions
    // using the connectivity of the original (coarse) mesh; navigating this mesh
    // will be much easier than navigating
    // the new subdivided (fine) mesh, which has more elements to traverse.  We
    // will then assign vertex positions in
    // the new mesh based on the values we computed for the original mesh.

    // Compute updated positions for all the vertices in the original mesh, using
    // the Loop subdivision rule.

    // Next, compute the updated vertex positions associated with edges.

    // Next, we're going to split every edge in the mesh, in any order. For
    // future reference, we're also going to store some information about which
    // subdivided edges come from splitting an edge in the original mesh, and
    // which edges are new.
    // In this loop, we only want to iterate over edges of the original
    // mesh---otherwise, we'll end up splitting edges that we just split (and
    // the loop will never end!)

    // Finally, flip any new edge that connects an old and new vertex.

    // Copy the updated vertex positions to the subdivided mesh.
}

/*
    Isotropic remeshing. Note that this function returns success in a similar
    manner to the local operations, except with only a boolean value.
    (e.g. you may want to return false if this is not a triangle mesh)
*/
bool Halfedge_Mesh::isotropic_remesh() {

    // Compute the mean edge length.
    // Repeat the four main steps for 5 or 6 iterations
    // -> Split edges much longer than the target length (being careful about
    //    how the loop is written!)
    // -> Collapse edges much shorter than the target length.  Here we need to
    //    be EXTRA careful about advancing the loop, because many edges may have
    //    been destroyed by a collapse (which ones?)
    // -> Now flip each edge if it improves vertex degree
    // -> Finally, apply some tangential smoothing to the vertex positions

    // Note: if you erase elements in a local operation, they will not be actually deleted
    // until do_erase or validate are called. This is to facilitate checking
    // for dangling references to elements that will be erased.
    // The rest of the codebase will automatically call validate() after each op,
    // but here simply calling collapse_edge() will not erase the elements.
    // You should use collapse_edge_erase() instead for the desired behavior.

    return false;
}

/* Helper type for quadric simplification */
struct Edge_Record {
    Edge_Record() {
    }
    Edge_Record(std::unordered_map<Halfedge_Mesh::VertexRef, Mat4>& vertex_quadrics,
                Halfedge_Mesh::EdgeRef e)
        : edge(e) {

        // Compute the combined quadric from the edge endpoints.
        // -> Build the 3x3 linear system whose solution minimizes the quadric error
        //    associated with these two endpoints.
        // -> Use this system to solve for the optimal position, and store it in
        //    Edge_Record::optimal.
        // -> Also store the cost associated with collapsing this edge in
        //    Edge_Record::cost.
        Mat4 sum = vertex_quadrics[e->halfedge()->vertex()] +
                   vertex_quadrics[e->halfedge()->twin()->vertex()];
        if(abs(sum.det()) < 0.0000001) {
            // this->optimal =
            //     (e->halfedge()->vertex()->pos + e->halfedge()->twin()->vertex()->pos) / 2;

            Vec4 v0 = Vec4(e->halfedge()->vertex()->pos, 1);
            Vec4 v1 = Vec4(e->halfedge()->twin()->vertex()->pos, 1);
            
            float v0cost = dot(v0, sum * v0);
            float v1cost = dot(v1, sum * v1);
            Vec4 vm = (v0 + v1) / 2;
            float vmcost = dot(vm, sum * vm);
            if(v0cost > v1cost) {
                if(v0cost > vmcost) {
                    this->optimal = e->halfedge()->vertex()->pos;
                }
            } else if(v1cost > vmcost) {
                this->optimal = e->halfedge()->twin()->vertex()->pos;
            } else {
                this->optimal =
                    (e->halfedge()->vertex()->pos + e->halfedge()->twin()->vertex()->pos) / 2;
            }    




     //        Vec4 dir = v1 - v0;
     //        float denom = dot(dir, sum * dir);
     //        float numer = dot(dir, sum * v0);
     //        if(abs(denom) < 0.0000001) {
     //            float v0cost = dot(v0, sum * v0);
     //            float v1cost = dot(v1, sum * v1);
     //            Vec4 vm = (v0 + v1) / 2;
     //            float vmcost = dot(vm, sum * vm);
     //            if(v0cost < v1cost) {
     //                if(v0cost < vmcost) {
     //                    this->optimal = e->halfedge()->vertex()->pos;
     //                }
     //            }
     //        	else if(v1cost < vmcost) {
     //                this->optimal = e->halfedge()->twin()->vertex()->pos;
     //            } else {
					// this->optimal = (e->halfedge()->vertex()->pos + e->halfedge()->twin()->vertex()->pos) / 2;
     //            }                
     //        }
     //    	else {
     //            float x = - numer / denom;
     //            std::cout << x << '\n';
     //            x = x < 0 ? 0 : x > 1 ? 1 : x;
     //            Vec4 opt = v0 + x * dir;
     //            this->optimal = Vec3(opt.x / opt.w, opt.y / opt.w, opt.z / opt.w);
     //        }
        }
    	else {
			Vec4 vopt = sum.inverse() * Vec4(0, 0, 0, 1);
            this->optimal = Vec3(vopt.x / vopt.w, vopt.y / vopt.w, vopt.z / vopt.w);
        }
        Vec4 v = Vec4(this->optimal, 1);
        Vec4 qv = sum * v;
        this->cost = dot(v, qv);
    }
    Halfedge_Mesh::EdgeRef edge;
    Vec3 optimal;
    float cost;
};

/* Comparison operator for Edge_Records so std::set will properly order them */
bool operator<(const Edge_Record& r1, const Edge_Record& r2) {
    if(r1.cost != r2.cost) {
        return r1.cost < r2.cost;
    }
    Halfedge_Mesh::EdgeRef e1 = r1.edge;
    Halfedge_Mesh::EdgeRef e2 = r2.edge;
    return &*e1 < &*e2;
}

/** Helper type for quadric simplification
 *
 * A PQueue is a minimum-priority queue that
 * allows elements to be both inserted and removed from the
 * queue.  Together, one can easily change the priority of
 * an item by removing it, and re-inserting the same item
 * but with a different priority.  A priority queue, for
 * those who don't remember or haven't seen it before, is a
 * data structure that always keeps track of the item with
 * the smallest priority or "score," even as new elements
 * are inserted and removed.  Priority queues are often an
 * essential component of greedy algorithms, where one wants
 * to iteratively operate on the current "best" element.
 *
 * PQueue is templated on the type T of the object
 * being queued.  For this reason, T must define a comparison
 * operator of the form
 *
 *    bool operator<( const T& t1, const T& t2 )
 *
 * which returns true if and only if t1 is considered to have a
 * lower priority than t2.
 *
 * Basic use of a PQueue might look
 * something like this:
 *
 *    // initialize an empty queue
 *    PQueue<myItemType> queue;
 *
 *    // add some items (which we assume have been created
 *    // elsewhere, each of which has its priority stored as
 *    // some kind of internal member variable)
 *    queue.insert( item1 );
 *    queue.insert( item2 );
 *    queue.insert( item3 );
 *
 *    // get the highest priority item currently in the queue
 *    myItemType highestPriorityItem = queue.top();
 *
 *    // remove the highest priority item, automatically
 *    // promoting the next-highest priority item to the top
 *    queue.pop();
 *
 *    myItemType nextHighestPriorityItem = queue.top();
 *
 *    // Etc.
 *
 *    // We can also remove an item, making sure it is no
 *    // longer in the queue (note that this item may already
 *    // have been removed, if it was the 1st or 2nd-highest
 *    // priority item!)
 *    queue.remove( item2 );
 *
 */
template<class T> struct PQueue {
    void insert(const T& item) {
        queue.insert(item);
    }
    void remove(const T& item) {
        if(queue.find(item) != queue.end()) {
            queue.erase(item);
        }
    }
    const T& top(void) const {
        return *(queue.begin());
    }
    void pop(void) {
        queue.erase(queue.begin());
    }
    size_t size() {
        return queue.size();
    }

    std::set<T> queue;
};

/*
    Mesh simplification. Note that this function returns success in a similar
    manner to the local operations, except with only a boolean value.
    (e.g. you may want to return false if you can't simplify the mesh any
    further without destroying it.)
*/
bool Halfedge_Mesh::simplify() {

    std::unordered_map<VertexRef, Mat4> vertex_quadrics;
    std::unordered_map<FaceRef, Mat4> face_quadrics;
    std::unordered_map<EdgeRef, Edge_Record> edge_records;
    PQueue<Edge_Record> edge_queue;

    auto face_quadric = [](FaceRef face) {
        Vec3 normal = face->normal();
        float d = -dot(normal, face->halfedge()->vertex()->pos);
        Vec4 p = Vec4(normal, d);
        Mat4 Kp = outer(p, p);
        return Kp;
    };

    auto vertex_quadric = [&face_quadrics](VertexRef vertex) {
        Vec4 zero = Vec4(0);
        Mat4 Kv = Mat4(zero, zero, zero, zero);
        auto iter = vertex->halfedge();
        do {
            auto f = iter->face();
            Kv = Kv + face_quadrics[iter->face()];
            iter = iter->twin()->next();
        } while(iter->id() != vertex->halfedge()->id());
        return Kv;
    };

    for (auto face = this->faces_begin(); face != this->faces_end(); ++face)
    {
        if(face->degree() != 3)
        {
            std::cout << "Not a triangular mesh\n";
	        return false;
        }
        face_quadrics.emplace(face, face_quadric(face));
    }

    for(auto vertex = this->vertices_begin(); vertex != this->vertices_end(); ++vertex) {
        vertex_quadrics.emplace(vertex, vertex_quadric(vertex));
    }

    for(auto edge = this->edges_begin(); edge != this->edges_end(); ++edge) {
        Edge_Record er = Edge_Record(vertex_quadrics, edge);
        edge_queue.insert(er);
        edge_records.emplace(edge, er);
    }

    auto init_face_count = this->n_faces();
    auto face_count = init_face_count;

    while(face_count * 4 > init_face_count) {
        // find a best valid edge
        auto best = edge_queue.top();
        auto invalid_bests = std::list<Edge_Record>();
        while(true) {
            if(this->eerased.find(best.edge) != this->eerased.end()) { //an edge to be erased, must do the check before eligibility check
                std::cout << "Edge to be erased";
                edge_queue.pop();
                edge_records.erase(best.edge);
                best = edge_queue.top();
            } else {
				if(check_edge_collapse_eligibility(best.edge)) break; //break if edge is valid
	            // std::cout << "encountered invalid edge\n";
	            invalid_bests.push_back(best);
	            edge_queue.pop(); 
	            best = edge_queue.top();
            }
            
        }
        // pop the best valid edge before adding the invalid ones back
        edge_queue.pop();
        edge_records.erase(best.edge);
        for (Edge_Record invalid_best : invalid_bests)
        {
            edge_queue.insert(invalid_best);
        }

        auto iter = best.edge->halfedge()->next();
        // faces and edges immediately adjacent to the collapsed face
        // also need to add the 1-ring edges to the adjacent edges as their faces also changed
        auto removeEdges = std::list<EdgeRef>();
        // std::cout << "begin first loop check\n";
        do
        {
            // std::cout << "loop 1\n";
            removeEdges.push_back(iter->edge());
            // adj_edges.push_back(iter->edge());
            if(iter->id() != best.edge->halfedge()->next()->id()) {
                removeEdges.push_back(iter->next()->edge());
            }
            iter = iter->twin()->next();
        } while(iter->id() != best.edge->halfedge()->twin()->id());

        iter = best.edge->halfedge()->twin()->next();
        // std::cout << "begin second loop check\n";
        do {
            // std::cout << "loop 2\n";
            removeEdges.push_back(iter->edge());
            if(iter->id() != best.edge->halfedge()->twin()->next()->id()) {
                removeEdges.push_back(iter->next()->edge());
            }
            iter = iter->twin()->next();
        } while(iter->id() != best.edge->halfedge()->id());

        for (auto remove_edge : removeEdges)
        {
            edge_queue.remove(edge_records[remove_edge]);
        }

        // std::cout << "adding vert and edge back\n";

        if(auto collapse_vertex_result = this->collapse_edge(best.edge, false)) {
            auto collapse_vertex = collapse_vertex_result.value();
            collapse_vertex->pos = best.optimal;
            auto viter = collapse_vertex->halfedge();
            do {
                auto new_adj_face = viter->face();
                face_quadrics[new_adj_face] = face_quadric(new_adj_face);
                viter = viter->twin()->next();
            } while(viter->id() != collapse_vertex->halfedge()->id());
            vertex_quadrics[collapse_vertex] = vertex_quadric(collapse_vertex);
            viter = collapse_vertex->halfedge();
            do {
                auto new_adj_edge = viter->edge();
                auto new_ring_edge = viter->next()->edge();
                auto er_adj = Edge_Record(vertex_quadrics, new_adj_edge);
                auto er_ring = Edge_Record(vertex_quadrics, new_ring_edge);
                edge_records[new_adj_edge] = er_adj;
                edge_records[new_ring_edge] = er_ring;
                edge_queue.insert(er_adj);
                edge_queue.insert(er_ring);
                viter = viter->twin()->next();
            } while(viter->id() != collapse_vertex->halfedge()->id());
            // collapse_vertex.value()->pos = best.optimal;
            // auto new_adj_faces = collapse_vertex.value()->get_faces();
            // auto new_adj_halfedges = collapse_vertex.value()->get_halfedges();
            // for (auto new_adj_face : new_adj_faces)
            // {
            //     face_quadrics[new_adj_face] = face_quadric(new_adj_face);
            // }
            // vertex_quadrics[collapse_vertex.value()] = vertex_quadric(collapse_vertex.value());
            // for(auto new_adj_halfedge : new_adj_halfedges) {
            //     auto er_adj = Edge_Record(vertex_quadrics, new_adj_halfedge->edge());
            //     // auto er_ring = Edge_Record(vertex_quadrics, new_adj_halfedge->next()->edge());
            //     edge_records[new_adj_halfedge->edge()] = er_adj;
            //     // edge_records[new_adj_halfedge->next()->edge()] = er_ring;
            //     edge_queue.insert(er_adj);
            //     // edge_queue.insert(er_ring);
            // }
            // std::cout << "cleanup done\n";
            face_count -= 2;
            // std::cout << face_count << ' ' << init_face_count;
        }
    	else {
            // std::cout << "Failed to collapse edge\n";
            return false;
        }
    }
    this->validate();
    // Compute initial quadrics for each face by simply writing the plane equation
    // for the face in homogeneous coordinates. These quadrics should be stored
    // in face_quadrics
    // -> Compute an initial quadric for each vertex as the sum of the quadrics
    //    associated with the incident faces, storing it in vertex_quadrics
    // -> Build a priority queue of edges according to their quadric error cost,
    //    i.e., by building an Edge_Record for each edge and sticking it in the
    //    queue. You may want to use the above PQueue<Edge_Record> for this.
    // -> Until we reach the target edge budget, collapse the best edge. Remember
    //    to remove from the queue any edge that touches the collapsing edge
    //    BEFORE it gets collapsed, and add back into the queue any edge touching
    //    the collapsed vertex AFTER it's been collapsed. Also remember to assign
    //    a quadric to the collapsed vertex, and to pop the collapsed edge off the
    //    top of the queue.

    // Note: if you erase elements in a local operation, they will not be actually deleted
    // until do_erase or validate are called. This is to facilitate checking
    // for dangling references to elements that will be erased.
    // The rest of the codebase will automatically call validate() after each op,
    // but here simply calling collapse_edge() will not erase the elements.
    // You should use collapse_edge_erase() instead for the desired behavior.

    return true;
}
