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
    /** Other Methods */
    virtual void draw(const Camera3D& camera3d, const Camera4D& camera4d)
            const = 0;

};


#endif //PART_2___GRAPHICS_ALTERNATE_OBJECT_H
