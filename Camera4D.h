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
    Camera4D(double polarAngle, double azimuthAngle, double phiAngle);

    /** Static Methods */
    /// TODO Implement last or not at all
    static double getFocalDistanceFromFOV(double fov);

    /** Getters */
    /// TODO Returns coefficients of 3-hyperplane equation orthogonal to normal
    std::vector<double> getHyperplane() const override;

    /** Setters */
    /* Utility */
    void setFocalDistance(double newFocalDistance) override;

    /* Movement */
    void setLocation(std::unique_ptr<point> newLocation) override;
    void setLocation(std::vector<double> newLocation) override;

    void setLocation(double x, double y, double z, double a);

    /* Rotation */
    void setNormal(spatialVector newNormal) override;
    void setSphericalDirection(std::unique_ptr<sphericalAngle> newAngles)
            override;
    void setSphericalDirection(std::vector<double> newAngles) override;

    void setSphericalDirection(double polarAngle, double azimuthAngle,
            double phiAngle);
    void setPolarAngle(double polarAngle);
    void setAzimuthAngle(double azimuthAngle);
    void setPhiAngle(double phiAngle);

    /** Other Methods */
    /* Utility */
    point3d projectPoint(point4d);

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
    void rotate(std::unique_ptr<sphericalAngle> dAngles) override;

    void rotate(double dPolarAngle, double dAzimuthAngle, double phiAngle);
    void rotatePolarAngle(double dPolarAngle);
    void rotateAzimuthAngle(double dAzimuthAngle);
    void rotatePhiAngle(double dPhiAngle);
    void rotateLeft();
    void rotateRight();
    void rotateUp();
    void rotateDown();
    void rotateIn();
    void rotateOut();

};


#endif //PART_2___GRAPHICS_ALTERNATE_CAMERA4D_H
