//
// Created by Jackson Hall on 4/27/2020.
//

#ifndef PART_2___GRAPHICS_ALTERNATE_SCENE_H
#define PART_2___GRAPHICS_ALTERNATE_SCENE_H


#include "Camera.h"
#include "Object.h"
#include "Camera3D.h"
#include "Camera4D.h"


class Scene {
public:
    /** Constructors */
    Scene();
    explicit Scene(const Camera3D& camera3d);
    Scene(const Camera3D& camera3d, const Camera4D& camera4d);
    explicit Scene(const std::vector<Object *>& objects);
    Scene(const Camera3D& camera3d, const Camera4D& camera4d, std::vector<Object *> objects);

    /** Getters */
    /* Utility */
    Camera3D const& getCamera3D() const;
    Camera4D const& getCamera4D() const;
    Camera3D& getCamera3D();
    Camera4D& getCamera4D();
    std::vector<Object *>& getObjects();

    /** Other Methods */
    void draw(); // TODO make this use polymorphism with Object3D and Object4d

protected:
    Camera3D camera3d;
    Camera4D camera4d;
    std::vector<Object *> objects;

};


#endif //PART_2___GRAPHICS_ALTERNATE_SCENE_H
