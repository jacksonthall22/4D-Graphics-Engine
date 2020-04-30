//
// Created by Jackson Hall on 4/27/2020.
//

#ifndef PART_2___GRAPHICS_ALTERNATE_OBJECT_4_D_H
#define PART_2___GRAPHICS_ALTERNATE_OBJECT_4_D_H

#include "Object.h"


class Object4D : protected Object {
public:
    /** Constructors */
    Object4D();
    Object4D(std::vector<point4d> vertices, std::vector<edge> edges);

    /** Getters */
    std::vector<point4d> getVertices() const;
    std::vector<edge> getEdges() const;

    /** Setters */
    void setVertices(std::vector<point4d> vertices);
    void setEdges(std::vector<edge> edges);

    /** Other Methods */

private:
    std::vector<point4d> vertices;
    std::vector<edge> edges;
};


#endif //PART_2___GRAPHICS_ALTERNATE_OBJECT_4_D_H
