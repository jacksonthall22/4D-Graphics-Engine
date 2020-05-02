//
// Created by Jackson Hall on 4/29/2020.
//

#ifndef PART_2___GRAPHICS_ALTERNATE_CAMERA4D_H
#define PART_2___GRAPHICS_ALTERNATE_CAMERA4D_H

#include "Camera.h"


class Camera4D : protected Camera {
public:
    /** Constructors */
    Camera4D();
    Camera4D(const Camera4D& other); // Implicitly deleted copy constructor
    Camera4D(const point4d& location,
            const spatialVector& normal,
            const sphericalAngle4d& sphericalDirection,
            const point4d& focus,
            const double& focalDistance);

    /** Getters */
    spatialVector getUnitUpVector() const;
    spatialVector getUnitRightVector() const;
    spatialVector getUnitOutVector() const;

    spatialVector const& getNormal() const;
    sphericalAngle4d const& getSphericalDirection() const;
    point4d const& getFocus() const;

    /** Setters */
    /* Utility */
    void setFocus() override;

    /* Movement */
    void setLocation(std::vector<double> newLocation) override;

    void setLocation(const point4d& newLocation);
    void setLocation(double x, double y, double z, double a);

    /* Rotation */
    void setNormal() override;
    void setSphericalDirection(std::vector<double> newAngles) override;

    void setSphericalDirection(const sphericalAngle4d& newAngles);
    void setSphericalDirection(double polarAngle, double azimuthAngle,
            double phiAngle);
    void setPolarAngle(double polarAngle);
    void setAzimuthAngle(double azimuthAngle);
    void setPhiAngle(double phiAngle);

    /** Other Methods */
    /* Utility */
    optional<point3d> projectPoint(const point4d& p) const;

    /* Movement */
    void move(std::vector<double> dPosition) override;
    void move(spatialVector dPosition) override;

    void move(double x, double y, double z, double a);
    void moveX(double dx);
    void moveY(double dy);
    void moveZ(double dz);
    void moveA(double da);
    void left();
    void right();
    void up();
    void down();
    void forward();
    void back();
    void in();
    void out();

    /* Rotation */
    void rotate(std::vector<double> dAngles) override;

    void rotate(const sphericalAngle4d& dAngles);
    void rotate(double dPolarAngle, double dAzimuthAngle, double phiAngle);
    void rotatePolar(double dPolarAngle);
    void rotateAzimuth(double dAzimuthAngle);
    void rotatePhi(double dPhiAngle);
    void rotateLeft();
    void rotateRight();
    void rotateUp();
    void rotateDown();
    void rotateIn();
    void rotateOut();

protected:
    /** Fields */
    // Location of the camera
    point4d location;

    // Direction the camera is facing. Used to determine 2d plane/3d
    // hyperplane of the camera onto which 3d/4d points get projected
    spatialVector normal;

    // A point P in the Scene is projected onto the hyperplane of the
    // Scene's camera by finding the intersection point of the line between
    // P and the Camera's focus and the Camera's hyperplane
    // Note: this means the focus is "behind" the direction the Camera sees
    point4d focus;

    // Direction the camera is facing in spherical coordinates. Used to
    // rotate the camera smoothly
    sphericalAngle4d sphericalDirection;

};


#endif //PART_2___GRAPHICS_ALTERNATE_CAMERA4D_H
