//
// Created by Jackson Hall on 4/27/2020.
//

#include "Object3D.h"
#include <utility>


/** ========== Constructors ========== */
Object3D::Object3D() :
        Object3D(
            std::vector<std::shared_ptr<point3d>>(),
            std::vector<std::shared_ptr<edge3d>>()
        ) {
}
Object3D::Object3D(const std::vector<point3d>& vertices,
        const std::vector<edge3d>& edges) {
    // Put all items into new vector of smart pointers
    std::vector<std::shared_ptr<point3d>> tempVertices;
    tempVertices.reserve(vertices.size());
    for (auto & vertex : vertices){
        tempVertices.push_back(std::make_shared<point3d>(vertex));
    }
    this->vertices = tempVertices;

    std::vector<std::shared_ptr<edge3d>> tempEdges;
    tempEdges.reserve(edges.size());
    for (auto & edge : edges){
        tempEdges.push_back(std::make_unique<edge3d>(edge));
    }
    this->edges = tempEdges;
}
Object3D::Object3D(std::vector<std::shared_ptr<point3d>> vertices,
        std::vector<std::shared_ptr<edge3d>> edges) {
    this->vertices = std::move(vertices);
    this->edges = std::move(edges);
}

/** ========== Getters ========== */
const std::vector<std::shared_ptr<point3d>>& Object3D::getVertices() const {
    return vertices;
}

const std::vector<std::shared_ptr<edge3d>> &Object3D::getEdges() const {
    return edges;
}


/** ========== Other Methods ========== */
/**
 * Draw this object on the screen by projecting them through the given
 * camera(s).
 */
void Object3D::draw(const Camera3D& camera3d, const Camera4D& camera4d) const {
    // Get drawable (x', y') coordinates for all vertices (x, y, z) in object
    std::vector<edge2d> drawableEdges;

    for (auto & edge : edges){
        // Draw a line on the screen between the vertices of the projected edge
        edge2d(
            *camera3d.projectPoint(*edge->p1), *camera3d.projectPoint(*edge->p2)
        ).draw();
    }
}
