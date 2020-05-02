//
// Created by Jackson Hall on 4/27/2020.
//

#include "Scene.h"
#include "Object3D.h"
#include <utility>


/** ========== Static Fields ========== */
const int Scene::DEFAULT_WINDOW_WIDTH = 960;
const int Scene::DEFAULT_WINDOW_HEIGHT = 540;
const double Scene::DEFAULT_OBJECT_COLOR_RGB[3] = {0.0, 0.0, 0.0};

/** ========== Constructors ========== */
Scene::Scene() :
        Scene(std::vector<Object3D>(), std::vector<Object4D>()) {
}
Scene::Scene(const std::vector<Object3D>& object3ds,
        const std::vector<Object4D>& object4ds) :
        camera3d(Camera3D()), camera4d(Camera4D()){
}
Scene::Scene(
        const Camera3D& camera3d,
        const Camera4D& camera4d,
        const std::vector<Object3D>& object3ds,
        const std::vector<Object4D>& object4ds) {
    this->camera3d = camera3d;
    this->camera4d = camera4d;
}

/** ========== Getters ========== */

Camera3D const& Scene::getCamera3D() const {
    return camera3d;
}

Camera4D const& Scene::getCamera4D() const {
    return camera4d;
}

std::vector<Object3D> const& Scene::get3dObjects() const {
    return object3ds;
}

std::vector<Object4D> const& Scene::get4dObjects() const {
    return object4ds;
}

void Scene::draw() const {
    for (auto & obj : object3ds){
        obj.draw(camera3d, camera4d);
    }

    for (auto & obj : object4ds){
        obj.draw(camera3d, camera4d);
    }
}
