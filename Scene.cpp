//
// Created by Jackson Hall on 4/27/2020.
//

#include "Scene.h"
#include "Object3D.h"


/** ========== Static Const Vars ========== */
const int Scene::TICKS_PER_SECOND = 30;
const double Scene::SECONDS_PER_TICK = 1.0 / Scene::TICKS_PER_SECOND;
const int Scene::DEFAULT_WINDOW_WIDTH = 960;
const int Scene::DEFAULT_WINDOW_HEIGHT = 540;
const double Scene::DEFAULT_OBJECT_COLOR_RGB[3] = {0.0, 0.0, 0.0};

/** ========== Constructors ========== */
Scene::Scene() :
        Scene(std::vector<Object3D>(), std::vector<Object4D>()){
}
Scene::Scene(const std::vector<Object3D>& object3ds,
        const std::vector<Object4D>& object4ds) :
        camera3d(Camera3D()),
        camera4d(Camera4D()),
        activeCamera(Camera::CameraType::Camera3D){
}
Scene::Scene(
        const Camera3D& camera3d,
        const Camera4D& camera4d,
        const std::vector<Object3D>& object3ds,
        const std::vector<Object4D>& object4ds) :
        activeCamera(Camera::CameraType::Camera3D){
    this->camera3d = camera3d;
    this->camera4d = camera4d;
    this->camera4d.setNormal();
    this->camera3d.setNormal();
}

/** ========== Getters ========== */

Camera3D& Scene::getCamera3D(){
    return camera3d;
}

Camera4D& Scene::getCamera4D(){
    return camera4d;
}

Camera::CameraType Scene::getActiveCamera() const {
    return activeCamera;
}

std::vector<Object3D> const& Scene::getObjects3d() const {
    return objects3d;
}

std::vector<Object4D> const& Scene::getObjects4d() const {
    return objects4d;
}

void Scene::addObject(const Object3D& obj){
    objects3d.push_back(obj);
}

void Scene::addObject(const Object4D& obj){
    objects4d.push_back(obj);
}

void Scene::toggleActiveCamera(){
    if (activeCamera == Camera::CameraType::Camera3D){
        activeCamera = Camera::CameraType::Camera4D;
    } else if (activeCamera == Camera::CameraType::Camera4D){
        activeCamera = Camera::CameraType::Camera3D;
    } else {
        std::cout << "Warning: Invalid activeCamera in:"
                     "\n\tvoid Scene::toggleActiveCamera()"
                     "\n\t(Scene.cpp)" << std::endl;
    }
}

void Scene::draw() const {
    for (auto & obj : objects3d){
        obj.draw(camera3d, camera4d);
    }

    for (auto & obj : objects4d){
        obj.draw(camera3d, camera4d);
    }
}
