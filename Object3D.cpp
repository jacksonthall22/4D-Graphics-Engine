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
        ){
}

//Object3D::Object3D(const std::vector<edge3d>& edges){
//    // Add all vertices from given edges to vertices vector
//    for (auto & edge : edges){
//
//    }
//}

Object3D::Object3D(const std::vector<point3d>& vertices,
        const std::vector<edge3d>& edges){
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
        std::vector<std::shared_ptr<edge3d>> edges){
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
 * Takes the current set of vertices and extrudes them into the given direction,
 * creating edges between the appropriate points.
 */
void Object3D::extrude(const spatialVector &direction){
    if (direction.components.size() <= 3){
        // Pad empty dimensions in given vector with 0s
        std::vector<double> temp = direction.components;
        while (temp.size() < 3){
            temp.push_back(0.0);
        }

        // For every edge, extrude both points in the given direction
        // and add 3 new edges to this object:
        //      1. Between the 2 extruded points
        //      2. Between the first point and the extruded first point
        //      3. Between the second point and the extruded second point
        //
        // Make temp vector of the newly created edges so as not to change
        // size of vector while looping
        std::vector<std::shared_ptr<edge3d>> newEdges;
        for (auto & edge : edges){
            point3d extrudedP1(*edge->p1);
            extrudedP1.move(direction);
            point3d extrudedP2(*edge->p2);
            extrudedP2.move(direction);

            // Add new points
            vertices.push_back(std::make_unique<point3d>(extrudedP1));
            vertices.push_back(std::make_unique<point3d>(extrudedP2));

            // Add new edges
            newEdges.push_back(std::make_shared<edge3d>(extrudedP1, extrudedP2));
            newEdges.push_back(std::make_shared<edge3d>(*edge->p1, extrudedP1));
            newEdges.push_back(std::make_shared<edge3d>(*edge->p2, extrudedP2));
        }

        // Add the new edges
        for (auto & edge : newEdges){
            edges.push_back(edge);
        }

    } else {
        std::cout << "Warning: Invalid input to:\n\tvoid extrude(const "
                     "spatialVector& direction)\n\t(Object3D.cpp)"
                     << std::endl;
    }
}

/**
 * Draw this object on the screen by projecting them through the given
 * camera(s).
 */
void Object3D::draw(const Camera3D& camera3d, const Camera4D& camera4d) const {
    // Get drawable (x', y') coordinates for all vertices (x, y, z) in object
    for (auto & edge : edges){
        // Draw a line on the screen between the vertices of the projected edge
        edge2d(
            *camera3d.projectPoint(*edge->p1), *camera3d.projectPoint(*edge->p2)
        ).draw();
    }
}
