#include "student_code.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL
{

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (class member).
   *
   * @param points A vector of points in 2D
   * @return A vector containing intermediate points or the final interpolated vector
   */

  std::vector<Vector2D> BezierCurve::evaluateStep(std::vector<Vector2D> const &points)
  { 
    // TODO Part 1.
    std::vector<Vector2D> evalPoints;
    for (size_t i = 0; i < points.size() - 1; i++) {
        evalPoints.push_back(Vector2D((1 - this->t) * points[i].x + this->t * points[i + 1].x, 
            (1 - this->t) * points[i].y + this->t * points[i + 1].y));
    }
    return evalPoints;
  }

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (function parameter).
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector3D> BezierPatch::evaluateStep(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Part 2.
    std::vector<Vector3D> evalPoints;
    for (size_t i = 0; i < points.size() - 1; i++) {
        evalPoints.push_back(Vector3D((1 - t) * points[i].x + t * points[i + 1].x,
            (1 - t) * points[i].y + t * points[i + 1].y,
            (1 - t) * points[i].z + t * points[i + 1].z));
    }
    return evalPoints;
  }

  /**
   * Fully evaluates de Casteljau's algorithm for a vector of points at scalar parameter t
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Part 2.
    std::vector<Vector3D> currentPoints = points;
    while (currentPoints.size() > 1) {
        currentPoints = evaluateStep(currentPoints, t);
    }
    return currentPoints[0];
  }

  /**
   * Evaluates the Bezier patch at parameter (u, v)
   *
   * @param u         Scalar interpolation parameter
   * @param v         Scalar interpolation parameter (along the other axis)
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate(double u, double v) const 
  {  
    // TODO Part 2.
    std::vector<Vector3D> tempPoints;
    for (const auto& row : controlPoints) {
        tempPoints.push_back(evaluate1D(row, u));
    }
    return evaluate1D(tempPoints, v);
  }

  Vector3D Vertex::normal( void ) const
  {
    // TODO Part 3.
    // Returns an approximate unit normal at this vertex, computed by
    // taking the area-weighted average of the normals of neighboring
    // triangles, then normalizing.
    Vector3D N(0, 0, 0); 

      
    HalfedgeCIter h = halfedge();
    do {
          
        Vector3D v1 = h->vertex()->position;
        Vector3D v2 = h->next()->vertex()->position;
        Vector3D v3 = h->next()->next()->vertex()->position;

        Vector3D faceNormal = cross(v2 - v1, v3 - v1);

        double area = faceNormal.norm() / 2.0;

        N += faceNormal.unit() * area;

        h = h->twin()->next();
    } while (h != halfedge()); 

    if (N.norm() > 0) {
        N.normalize();
    }

    return N;
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    // TODO Part 4.
    // This method should flip the given edge and return an iterator to the flipped edge.
      if (e0->isBoundary()) {
          return e0; 
      }

      // Get the halfedges
      HalfedgeIter h0 = e0->halfedge();
      HalfedgeIter h1 = h0->twin();

      // vert (to flip) 
      VertexIter va = h0->next()->next()->vertex(); 
      VertexIter vb = h1->next()->next()->vertex(); 

      // half init
      HalfedgeIter h2 = h0->next();
      HalfedgeIter h3 = h2->next();
      HalfedgeIter h4 = h1->next();
      HalfedgeIter h5 = h4->next();

      // vertices
      VertexIter v0 = h0->vertex();
      VertexIter v1 = h1->vertex();
      VertexIter v2 = h2->next()->vertex();
      VertexIter v3 = h4->next()->vertex();

      // faces
      FaceIter f0 = h0->face();
      FaceIter f1 = h1->face();

      // halfedge->face
      h0->face() = f0;
      h1->face() = f1;
      h2->face() = f1;
      h3->face() = f0;
      h4->face() = f0;
      h5->face() = f1;

      // vertex->halfedge
      v0->halfedge() = h4;
      v1->halfedge() = h2;
      v2->halfedge() = h3;
      v3->halfedge() = h5;

      // face->halfedge
      f0->halfedge() = h0;
      f1->halfedge() = h1;

      // halfedge->next
      h0->next() = h3;
      h3->next() = h4;
      h4->next() = h0;

      h1->next() = h5;
      h5->next() = h2;
      h2->next() = h1;

      // halfedge->vertex
      h0->vertex() = v3;
      h1->vertex() = v2;

      return e0;

  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    // TODO Part 5.
    // This method should split the given edge and return an iterator to the newly inserted vertex.
    // The halfedge of this vertex should point along the edge that was split, rather than the new edges.
      if (e0->isBoundary()) {
          return VertexIter(); 
      }
      // init vert, half edge ,face + new vert-edge-face, update vert/edge/face->haf, hf->neighbors 
      //new vert
      VertexIter v0 = newVertex();
      v0->position = (e0->halfedge()->vertex()->position + e0->halfedge()->twin()->vertex()->position) / 2.0;

      //new edge 
      EdgeIter e1 = newEdge(), e2 = newEdge(), e3 = newEdge();

      // new face
      FaceIter f2 = newFace(), f3 = newFace();

      // orig edge
      HalfedgeIter h0 = e0->halfedge();
      HalfedgeIter h1 = h0->twin();
      HalfedgeIter h2 = h0->next();
      HalfedgeIter h3 = h2->next();
      HalfedgeIter h4 = h1->next();
      HalfedgeIter h5 = h4->next();

      // orig face
      FaceIter f0 = h0->face();
      FaceIter f1 = h1->face();

      // orig vert
      VertexIter v1 = h0->vertex();
      VertexIter v2 = h1->vertex();
      VertexIter v3 = h3->vertex();
      VertexIter v4 = h5->vertex();

      // new halfedges init
      HalfedgeIter h6 = newHalfedge();
      HalfedgeIter h7 = newHalfedge();
      HalfedgeIter h8 = newHalfedge();
      HalfedgeIter h9 = newHalfedge();
      HalfedgeIter h10 = newHalfedge();
      HalfedgeIter h11 = newHalfedge();

      // new halfedges 
      h6->setNeighbors(h2, h7, v0, e1, f3);
      h7->setNeighbors(h8, h6, v2, e1, f2);

      h8->setNeighbors(h5, h9, v0, e2, f2);
      h9->setNeighbors(h1, h8, v4, e2, f1);

      h10->setNeighbors(h3, h11, v0, e3, f0);
      h11->setNeighbors(h6, h10, v3, e3, f3);

      // old halfedges
      h0->setNeighbors(h10, h1, v1, e0, f0);
      h1->setNeighbors(h4, h0, v0, e0, f1);

      h2->setNeighbors(h11, h2->twin(), h2->vertex(), h2->edge(), f3);
      h3->setNeighbors(h0, h3->twin(), h3->vertex(), h3->edge(), f0);

      h4->setNeighbors(h9, h4->twin(), h4->vertex(), h4->edge(), f1);
      h5->setNeighbors(h7, h5->twin(), h5->vertex(), h5->edge(), f2);

      // vertex->halfedge pointers
      v0->halfedge() = h10;
      v1->halfedge() = h0;
      v2->halfedge() = h7;
      v3->halfedge() = h11;
      v4->halfedge() = h9;

      // edge->halfedge pointers
      e0->halfedge() = h0;
      e1->halfedge() = h7;
      e2->halfedge() = h9;
      e3->halfedge() = h11;

      // face->halfedge pointers
      f0->halfedge() = h3;
      f1->halfedge() = h4;
      f2->halfedge() = h5;
      f3->halfedge() = h2;

      e2->halfedge()->edge()->isNew = true;
      e3->halfedge()->edge()->isNew = true;

      // Return the new vertex iterator
      return v0;
  }



  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {
    // TODO Part 6.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
    // One possible solution is to break up the method as listed below.

    // 1. Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
    // and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
    // a vertex of the original mesh.
    
    // 2. Compute the updated vertex positions associated with edges, and store it in Edge::newPosition.
    
    // 3. Split every edge in the mesh, in any order. For future reference, we're also going to store some
    // information about which subdivide edges come from splitting an edge in the original mesh, and which edges
    // are new, by setting the flat Edge::isNew. Note that in this loop, we only want to iterate over edges of
    // the original mesh---otherwise, we'll end up splitting edges that we just split (and the loop will never end!)
    
    // 4. Flip any new edge that connects an old and new vertex.

    // 5. Copy the new vertex positions into final Vertex::position.

    for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
        Vector3D sumAdjacentPoints(0, 0, 0);
        int n = 0; // Degree

        //Halfedges -> update degree -> update adj points 
        HalfedgeIter h = v->halfedge();
        do {
            sumAdjacentPoints += h->twin()->vertex()->position;
            n++;
            h = h->twin()->next();
        } while (h != v->halfedge());

        double u = (n == 3) ? 3.0 / 16 : 3.0 / (8.0 * n);

        v->newPosition = (1 - n * u) * v->position + u * sumAdjacentPoints;
        v->isNew = false; // Reset
    }

    // 2. Compute the updated vertex positions associated with edges, and store it in Edge::newPosition.
    for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
       //Find abcd -> compute 
        Vector3D A = e->halfedge()->vertex()->position;
        Vector3D B = e->halfedge()->twin()->vertex()->position;
        Vector3D C = e->halfedge()->next()->next()->vertex()->position;
        Vector3D D = e->halfedge()->twin()->next()->next()->vertex()->position;

        e->newPosition = 3.0 / 8.0 * (A + B) + 1.0 / 8.0 * (C + D);
        
    }

    // 3. Split every edge in the mesh, in any order. For future reference, we're also going to store some
    // information about which subdivide edges come from splitting an edge in the original mesh, and which edges
    // are new, by setting the flat Edge::isNew. Note that in this loop, we only want to iterate over edges of
    // the original mesh---otherwise, we'll end up splitting edges that we just split (and the loop will never end!)

    // Reset all e-isnew here first. Iterate, split. How to find new edges?? 
    std::vector<EdgeIter> originalEdges;
    for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
        originalEdges.push_back(e);
        e->isNew = false; 
    }
    for (EdgeIter e : originalEdges) {
        if (!e->isNew) {
            VertexIter newV = mesh.splitEdge(e); 
            //update pos, isnew? update edge if new. 
            newV->position = e->newPosition; 
            newV->isNew = true;

            
        }
    }
    
    // 4. Flip any new edge that connects an old and new vertex.
    for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) { 
        if (e->isNew) {
            VertexIter v1 = e->halfedge()->vertex();
            VertexIter v2 = e->halfedge()->twin()->vertex();

            
            if ((v1->isNew && !v2->isNew) || (!v1->isNew && v2->isNew)) {
                mesh.flipEdge(e); 
            }

        }
    }

    // 5. Copy the new vertex positions into final Vertex::position.
    for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
        if (!v->isNew) {   
            v->position = v->newPosition;
        }
    }
      
    }
}
