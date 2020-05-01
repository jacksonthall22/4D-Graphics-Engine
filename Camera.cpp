//
// Created by Jackson Hall on 4/27/2020.
//

#include "Camera.h"
#include <cmath>


/** Static Fields */
const double Camera::DEFAULT_MOVE_DISTANCE = 0.1;

const double Camera::DEFAULT_ROTATION_ANGLE = 1;

const double Camera::DEFAULT_FOV = 160;

/** Constructors */
Camera::Camera(double focalDistance) : focalDistance(focalDistance) {
}

/** Static Methods */
/**
 * Return the distance of the focus from a Camera's plane/hyperplane to
 * render with a field of view of the given degree angle.
 */
double Camera::getFocalDistanceFromFOV(const double fovDegrees) {
    // Focal distance is the cosine of half the field of view
    return cos(fovDegrees / 2.0 * M_PI / 180);
}

/** Getters */
double Camera::getFocalDistance() const {
    return focalDistance;
}

/** Setters */
void Camera::setFocalDistance(const double newFocalDistance) {
    focalDistance = newFocalDistance;
}

