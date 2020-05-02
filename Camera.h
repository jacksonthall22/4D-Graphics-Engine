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
    /** Static Fields */
    // Distance the camera moves when moved using up(), down(), etc.
    static const double DEFAULT_MOVE_DISTANCE;

    // Angle the camera moves when rotated using rotateUp(), rotateDown(), etc.
    static const double DEFAULT_ROTATION_ANGLE;

    // Default field of view in degrees
    static const double DEFAULT_FOV;

    /** Constructors */
    explicit Camera(double focalDistance);

    /** Static Methods */
    static double getFocalDistanceFromFOV(double fovDegrees);

    /** Getters */
    double getFocalDistance() const;

    /** Setters */
    /* Utility */
    void setFocalDistance(double newFocalDistance);

    virtual void setFocus() = 0;

    /* Movement */
    virtual void setLocation(std::vector<double> newLocation) = 0;

    /* Rotation */
    virtual void setNormal() = 0;
    virtual void setSphericalDirection(std::vector<double> newAngles) = 0;

    /** Other Methods */
    /* Movement */
    virtual void move(std::vector<double> dPosition) = 0;
    virtual void move(spatialVector dPosition) = 0;

    /* Rotation */
    virtual void rotate(std::vector<double> dAngles) = 0;

protected:
    // Distance in the direction opposite the Camera's normal from its
    // coordinate location to the focus. Used to set the focus point
    double focalDistance{};

};


#endif //PART_2___GRAPHICS_ALTERNATE_CAMERA_H
