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
    /// TODO These methods segfault
//    virtual void extrude(const std::vector<double>& direction) = 0;
//    virtual void extrude(const spatialVector& direction) = 0;
    virtual void draw(const Camera3D& camera3d, const Camera4D& camera4d)
            const = 0;

};


#endif //PART_2___GRAPHICS_ALTERNATE_OBJECT_H
