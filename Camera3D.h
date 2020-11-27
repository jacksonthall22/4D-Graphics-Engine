//
// Created by Jackson Hall on 4/29/2020.
//

#ifndef PART_2___GRAPHICS_ALTERNATE_CAMERA3D_H
#define PART_2___GRAPHICS_ALTERNATE_CAMERA3D_H

#include "Camera.h"


class Camera3D : protected Camera {
public:
    /** Constructors */
    Camera3D();
    Camera3D(const Camera3D& other); // Implicitly deleted copy constructor
    Camera3D(const point3d& location,
            const spatialVector& normal,
            const sphericalAngle3d& sphericalDirection,
            const point3d& focus,
            const double& focalDistance);

    /** Getters */
    spatialVector getUnitUpVector() const;
    spatialVector getUnitRightVector() const;

    spatialVector const& getNormal() const;
    sphericalAngle3d const& getSphericalDirection() const;
    point3d const& getFocus() const;

    /** Setters */
    /* Utility */
    void setFocus() override;

    /* Movement */
    void setLocation(std::vector<double> newLocation) override;

    void setLocation(const point3d& newLocation);
    void setLocation(double x, double y, double z);

    /* Rotation */
    void setNormal() override;
    void setSphericalDirection(std::vector<double> newAngles) override;
    void setSphericalDirection(const sphericalAngle3d& newAngles);
    void setSphericalDirection(double polarAngle, double azimuthAngle);
    void setPolar(double polarAngle);
    void setAzimuth(double azimuthAngle);

    /** Other Methods */
    /* Utility */
    optional<point2d> projectPoint(const point3d& p) const;

    /* Movement */
    // Move by given values
    void move(std::vector<double> dPosition) override;
    void move(const spatialVector& dPosition) override;
    void move(double dX, double dY, double dZ);
    void moveX(double dX);
    void moveY(double dY);
    void moveZ(double dZ);

    // Move by custom amounts
    void moveLeftCustom(double dL);
    void moveRightCustom(double dR);
    void moveUpCustom(double dU);
    void moveDownCustom(double dD);
    void moveForwardCustom(double dF);
    void moveBackCustom(double dB);

    // Move by Camera.DEFAULT_MOVE_DISTANCE
    void moveLeftDefault();
    void moveRightDefault();
    void moveUpDefault();
    void moveDownDefault();
    void moveForwardDefault();
    void moveBackDefault();

    // Move according to values in this.velocityVec
    void moveVelocity();
    void moveForwardVelocity();
    void moveRightVelocity();
    void moveUpVelocity();

    /**
     * Set movingForward, movingStrafe, and movingUp to given new values
     * (each must be 1, 0, or -1)
     */
    void setMovementStatuses(int movingForward_,
                             int movingStrafe_,
                             int movingUp_);
    // Update the velocity of the camera given current movement statuses
    // (ie. movingForward, etc.)
    void updateVelocities();

    /* Rotation */
    void rotate(std::vector<double> dAngles) override;
    void rotate(const sphericalAngle3d& dAngles);
    void rotate(double dPolarAngle, double dAzimuthAngle);
    void rotatePolar(double dPolarAngle);
    void rotatePolar(double dPolarAngle, bool updateNormal);
    void rotateAzimuth(double dAzimuthAngle);
    void rotateAzimuth(double dAzimuthAngle, bool updateNormal);
    void rotateLeft();
    void rotateRight();
    void rotateUp();
    void rotateDown();

protected:
    /** Fields */
    // Location of the camera
    point3d location;

    // Holds 1, -1, or 0 denoting positive, negative, or no movement
    // (acceleration) along forward/backward, left/right, and up/down
    // directions from perspective of the Camera (must then be translated
    // to get absolute x/y/z movement velocities)
    int movingForward;
    int movingStrafe; // Moving towards Camera.getUnitRightVec()
    int movingUp;

    // Store the velocity of the camera movement from the perspective
    // of the camera (not x/y/z velocities)
    spatialVector velocityVec;

    // Direction the camera is facing. Used to determine 2d plane/3d
    // hyperplane of the camera onto which 3d/4d points get projected
    spatialVector normal;

    // A point P in the Scene is projected onto the plane of the
    // Scene's camera by finding the intersection point of the line between
    // P and the Camera's focus and the Camera's plane
    // Note: this means the focus is "behind" the direction the Camera sees
    point3d focus;

    // Direction the camera is facing in spherical coordinates. Used to
    // rotate the camera smoothly
    sphericalAngle3d sphericalDirection;

};


#endif //PART_2___GRAPHICS_ALTERNATE_CAMERA3D_H
