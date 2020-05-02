//
// Created by Jackson Hall on 4/27/2020.
//

#include "Object4D.h"

#include <utility>


/** ========== Constructors ========== */
Object4D::Object4D() :
        Object4D(
            std::vector<std::shared_ptr<point4d>>(),
            std::vector<std::shared_ptr<edge4d>>()
        ) {
}
Object4D::Object4D(const std::vector<point4d>& vertices,
                   const std::vector<edge4d>& edges) {
    // Put all items into new vector of smart pointers
    std::vector<std::shared_ptr<point4d>> tempVertices;
    tempVertices.reserve(vertices.size());
    for (auto & vertex : vertices){
        tempVertices.push_back(std::make_shared<point4d>(vertex));
    }
    this->vertices = tempVertices;

    std::vector<std::shared_ptr<edge4d>> tempEdges;
    tempEdges.reserve(edges.size());
    for (auto & edge : edges){
        tempEdges.push_back(std::make_shared<edge4d>(edge));
    }
    this->edges = tempEdges;
}
Object4D::Object4D(std::vector<std::shared_ptr<point4d>> vertices,
                   std::vector<std::shared_ptr<edge4d>> edges) {
    this->vertices = std::move(vertices);
    this->edges = std::move(edges);
}

/** ========== Getters ========== */
const std::vector<std::shared_ptr<point4d>> &Object4D::getVertices() const {
    return vertices;
}

const std::vector<std::shared_ptr<edge4d>> &Object4D::getEdges() const {
    return edges;
}


/** ========== Other Methods ========== */
/**
 * Draw this object on the screen by projecting them through the given
 * camera(s).
 */
/**
 * Draw this object on the screen by projecting them through the given
 * camera(s).
 */
void Object4D::draw(const Camera3D& camera3d, const Camera4D& camera4d) const {
    // Get drawable (x', y') coordinates for all vertices (x, y, z) in object
    std::vector<edge2d> drawableEdges;

    for (auto & edge : edges){
        // First get the location of the 4d point cast to 3-space
        edge3d tempEdge3d(
            *camera4d.projectPoint(*edge->p1), *camera4d.projectPoint(*edge->p2)
        );

        // Draw a line on the screen between the vertices of the projected edge
        edge2d(*camera3d.projectPoint(*tempEdge3d.p1),
                *camera3d.projectPoint(*tempEdge3d.p2)).draw();
    }
}
