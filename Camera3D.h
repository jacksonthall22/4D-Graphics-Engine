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
    Camera3D(double polarAngle, double azimuthAngle);

    /** Static Methods */
    /// TODO Implement last or not at all
    static double getFocalDistanceFromFOV(double fov);

    /** Getters */
    /// TODO Returns coefficients of 2-hyperplane equation orthogonal to normal
    std::vector<double> getHyperplane() const override;

    /** Setters */
    /* Utility */
    void setFocalDistance(double newFocalDistance) override;

    /* Movement */
    void setLocation(std::unique_ptr<point> newLocation) override;
    void setLocation(std::vector<double> newLocation) override;

    void setLocation(double x, double y, double z);

    /* Rotation */
    void setNormal(spatialVector newNormal) override;
    void setSphericalDirection(std::unique_ptr<sphericalAngle> newAngles)
            override;
    void setSphericalDirection(std::vector<double> newAngles) override;

    void setSphericalDirection(double polarAngle, double azimuthAngle);
    void setPolarAngle(double polarAngle);
    void setAzimuthAngle(double azimuthAngle);

    /** Other Methods */
    /* Utility */
    point2d projectPoint(point3d);

    /* Movement */
    void move(std::vector<double> dPosition) override;
    void move(spatialVector dPosition) override;

    void move(double x, double y, double z);
    void moveX(double dx);
    void moveY(double dy);
    void moveZ(double dz);
    void left();
    void right();
    void up();
    void down();
    void forward();
    void back();

    /* Rotation */
    void rotate(std::vector<double> dAngles) override;
    void rotate(std::unique_ptr<sphericalAngle> dAngles) override;

    void rotate(double dPolarAngle, double dAzimuthAngle);
    void rotatePolarAngle(double dPolarAngle);
    void rotateAzimuthAngle(double dAzimuthAngle);
    void rotateLeft();
    void rotateRight();
    void rotateUp();
    void rotateDown();

};


#endif //PART_2___GRAPHICS_ALTERNATE_CAMERA3D_H
