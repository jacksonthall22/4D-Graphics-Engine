//
// Created by Jackson Hall on 4/27/2020.
//

#include "Scene.h"

#include <utility>

/** ========== Constructors ========== */
Scene::Scene() :
        Scene(Camera3D()) {
}

Scene::Scene(const Camera3D &camera3d) :
        Scene(camera3d, Camera4D()) {
}

Scene::Scene(const Camera3D& camera3d, const Camera4D& camera4d,
        std::vector<Object *> objects) {
    this->camera3d = camera3d;
    this->camera4d = camera4d;
    this->objects = std::move(objects);
}

Scene::Scene(const std::vector<Object *>& objects) :
        Scene(Camera3D(), Camera4D(), objects) {
}

/** ========== Getters ========== */

Camera3D* Scene::getCamera3D() const {
    return *camera3d;
}

Camera4D* Scene::getCamera4D() const {
    return *camera4d;
}

std::vector<Object *>& Scene::getObjects() {
    return objects;
}

void Scene::draw() {
    for (auto & object : objects) {
        object->draw(getCamera3D(), getCamera4D());
    }
}
