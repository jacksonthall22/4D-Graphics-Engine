//
// Created by Jackson Hall on 4/27/2020.
//

#include "Camera.h"

#include <utility>

double DEFAULT_STEP = 0.1;

spatialVector const &Camera::getNormal() const {
    return normal;
}

sphericalAngle const &Camera::getSphericalDirection() const {
    return *sphericalDirection;
}

point const &Camera::getFocus() const {
    return *focus;
}

double Camera::getFocalDistance() const {
    return focalDistance;
}
