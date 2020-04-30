//
// Created by Jackson Hall on 4/27/2020.
//

#ifndef PART_2___GRAPHICS_ALTERNATE_CAMERA_H
#define PART_2___GRAPHICS_ALTERNATE_CAMERA_H

#include "utils.cpp"
#include <memory>
#include <vector>


class Camera {
public:
    /** Static Fields */
    // Distance the camera moves when moved using up(), down(), etc.
    const static double DEFAULT_MOVE_DISTANCE;

    // Angle the camera moves when rotated using rotateUp(), rotateDown(), etc.
    const static double DEFAULT_ROTATION_ANGLE;

    /** Getters */
    /* Utility */
    spatialVector const& getNormal() const;
    sphericalAngle const& getSphericalDirection() const;
    point const& getFocus() const;
    double getFocalDistance() const;
    virtual std::vector<double> getHyperplane() const = 0;

    /** Setters */
    /* Utility */
    virtual void setFocalDistance(double newFocalDistance) = 0;

    /* Movement */
    virtual void setLocation(std::unique_ptr<point> newLocation) = 0;
    virtual void setLocation(std::vector<double> newLocation) = 0;

    /* Rotation */
    virtual void setNormal(spatialVector newNormal) = 0;
    virtual void setSphericalDirection(std::unique_ptr<sphericalAngle>
            newAngles) = 0;
    virtual void setSphericalDirection(std::vector<double> newAngles) = 0;

    /** Other Methods */
    /* Movement */
    virtual void move(std::vector<double> dPosition) = 0;
    virtual void move(spatialVector dPosition) = 0;

    /* Rotation */
    virtual void rotate(std::vector<double> dAngles) = 0;
    virtual void rotate(std::unique_ptr<sphericalAngle> dAngles) = 0;


protected:
    /** Fields */
    // Direction the camera is facing. Used to determine 2d plane/3d
    // hyperplane of the camera onto which 3d/4d points get projected
    spatialVector normal;

    // Direction the camera is facing in spherical coordinates. Used to
    // rotate the camera smoothly
    std::unique_ptr<sphericalAngle> sphericalDirection;

    // A point P in the Scene is projected onto the plane/hyperplane of the
    // Scene's camera by finding the intersection point of the line between
    // P and the Camera's focus and the Camera's plane/hyperplane
    // Note: this means the focus is "behind" the direction the Camera sees
    std::unique_ptr<point> focus;

    // Distance in the direction opposite the Camera's normal from its
    // coordinate location to the focus. Used to set the focus point
    double focalDistance{};

    /** Setters */
    virtual void setFocus(point* newFocus) = 0;
    virtual void setFocus(std::vector<double> newFocus) = 0;

};


#endif //PART_2___GRAPHICS_ALTERNATE_CAMERA_H
