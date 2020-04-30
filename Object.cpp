//
// Created by Jackson Hall on 4/27/2020.
//

#include "Object.h"

Object::Object() {

}

std::vector<point *> &Object::getVertices() {
    return vertices;
}

std::vector<edge *> &Object::getEdges() {
    return edges;
}
