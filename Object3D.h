//
// Created by Jackson Hall on 4/27/2020.
//

#ifndef PART_2___GRAPHICS_ALTERNATE_OBJECT3D_H
#define PART_2___GRAPHICS_ALTERNATE_OBJECT3D_H

#include "Object.h"

class Object3D : protected Object {
public:
    /** Constructors */
    Object3D();
    Object3D(std::vector<point3d> vertices, std::vector<edge> sides);

    /** Getters */


};


#endif //PART_2___GRAPHICS_ALTERNATE_OBJECT3D_H
