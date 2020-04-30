//
// Created by Jackson Hall on 4/27/2020.
//

#ifndef PART_2___GRAPHICS_ALTERNATE_OBJECT_H
#define PART_2___GRAPHICS_ALTERNATE_OBJECT_H

#include "Camera3D.h"
#include "Camera4D.h"
#include <vector>
#include <memory>

class Object {
public:
    /** Constructors */
    Object();

    /** Getters */
    /* Utility */
    std::vector<std::unique_ptr<point>>& getVertices();
    std::vector<std::unique_ptr<edge>>& getEdges();

    /** Setters */
    /* Utility */
    virtual void setVertices(std::vector<std::vector<point>> vertices) = 0;
    virtual void setEdges(std::vector<std::vector<std::vector<point>>> sides)
            = 0;

    /** Other Methods */
    virtual void draw(const Camera3D& camera3d, const Camera4D& camera4d) = 0;

private:
    /** Fields */
    // Either point3ds or point4ds
    std::vector<std::unique_ptr<point>> vertices;

    // Pairs between the points in vertices vector
    std::vector<std::unique_ptr<edge>> edges;
};


#endif //PART_2___GRAPHICS_ALTERNATE_OBJECT_H
