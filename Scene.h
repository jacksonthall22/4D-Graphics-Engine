//
// Created by Jackson Hall on 4/27/2020.
//

#ifndef PART_2___GRAPHICS_ALTERNATE_SCENE_H
#define PART_2___GRAPHICS_ALTERNATE_SCENE_H

#include "Camera.h"
#include "Object.h"
#include "Camera3D.h"
#include "Camera4D.h"
#include "Object3D.h"
#include "Object4D.h"


class Scene {
public:
    static const int DEFAULT_WINDOW_WIDTH;
    static const int DEFAULT_WINDOW_HEIGHT;
    static const double DEFAULT_OBJECT_COLOR_RGB[3];

    /** Constructors */
    Scene();
    Scene(const std::vector<Object3D>& object3ds,
        const std::vector<Object4D>& object4ds);
    Scene(const Camera3D& camera3d,
        const Camera4D& camera4d,
        const std::vector<Object3D>& object3ds,
        const std::vector<Object4D>& object4ds);

    /** Getters */
    /* Utility */
    Camera3D const& getCamera3D() const;
    Camera4D const& getCamera4D() const;
    std::vector<Object *> const& getObjects() const;

    /** Other Methods */
    void addObject(Object* obj);
    void draw() const;

protected:
    Camera3D camera3d;
    Camera4D camera4d;
    std::vector<Object *> objects;
};


#endif //PART_2___GRAPHICS_ALTERNATE_SCENE_H
