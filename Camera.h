//
// Created by Jackson Hall on 4/27/2020.
//

#ifndef PART_2___GRAPHICS_ALTERNATE_CAMERA_H
#define PART_2___GRAPHICS_ALTERNATE_CAMERA_H

#include "utils.h"
#include <memory>
#include <vector>

class Camera {
public:
    /** ---------- Static Fields ---------- */
    // Distance in blocks the camera moves
    static const double DEFAULT_MOVE_DISTANCE;

    // Angle in degrees the camera rotates by default
    static const double DEFAULT_ROTATION_ANGLE;

    // Default field of view in degrees
    static const double DEFAULT_FOV;

    // Values used to update camera velocity
    static const double FORWARD_ACCELERATION; // blocks/second^2
    static const double STRAFE_ACCELERATION;
    static const double UP_ACCELERATION;
    static const double FORWARD_DRAG; // blocks/second^2
    static const double STRAFE_DRAG;
    static const double UP_DRAG;
    static const double FORWARD_BRAKE; // blocks/second^2
    static const double STRAFE_BRAKE;
    static const double UP_BREAK;
    static const double MAX_FORWARD_VELOCITY; // blocks/second
    static const double MAX_STRAFE_VELOCITY;
    static const double MAX_UP_VELOCITY;

    /** ---------- Constructors ---------- */
    explicit Camera(double focalDistance);

    /** ---------- Static Methods ---------- */
    static double getFocalDistanceFromFOV(double fovDegrees);

    /** ---------- Getters ---------- */
    double getFocalDistance() const;

    /** ---------- Setters ---------- */
    /* Utility */
    void setFocalDistance(double newFocalDistance);

    virtual void setFocus() = 0;

    /* Movement */
    virtual void setLocation(std::vector<double> newLocation) = 0;

    /* Rotation */
    virtual void setNormal() = 0;
    virtual void setSphericalDirection(std::vector<double> newAngles) = 0;

    /** ---------- Other Methods ---------- */
    /* Movement */
    virtual void move(std::vector<double> dPosition) = 0;
    virtual void move(const spatialVector& dPosition) = 0;

    /* Rotation */
    virtual void rotate(std::vector<double> dAngles) = 0;

protected:
    // Distance in the direction opposite the Camera's normal from its
    // coordinate location to the focus. Used to set the focus point
    double focalDistance{};
};


#endif //PART_2___GRAPHICS_ALTERNATE_CAMERA_H
