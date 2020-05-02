//
// Created by Jackson Hall on 5/1/2020.
//

#ifndef PART_2___GRAPHICS_ALTERNATE_UTILS_H
#define PART_2___GRAPHICS_ALTERNATE_UTILS_H

#include <utility>
#include <vector>
#include <cmath>
#include <iostream>
#include <memory>
#if __cplusplus < 201700
    #include <experimental/optional>
#include <GL/gl.h>

using std::experimental::optional;
    using std::experimental::nullopt;
    using std::experimental::make_optional;
#else
    #include <optional>
#endif

namespace utils{


double square(double n);
double rad(double deg);
double mod(double a, double n);
void modEquals(double *a, double n);
struct spatialVector {
    /** Fields */
    std::vector<double> components;

    /** Constructors */
    spatialVector();
    spatialVector(const spatialVector& other);
    explicit spatialVector(const std::vector<double> components);

    /** Other Methods */
    /* Utility */
    /**
     * Multiply all components of a vector by the given scalar.
     */
    void scale(const double scalar);
    /**
     * Return the magnitude of the vector.
     */
    double magnitude() const;
    /**
     * Compute the dot product of this vector and another.
     */
    double dot(const spatialVector& other) const;
    /**
     * Return the cosine of the angle between this vector and another in
     * degrees.
     */
    double cosOfAngleBetween(const spatialVector& other) const;
    /**
     * Return the scalar projection of this vector onto another.
     */
    double scalarProjectOnto(const spatialVector& other) const;
    /**
     * Return the magnitude of the vector rejection of this from other.
     */
    double scalarRejectFrom(const spatialVector& other) const;
};

/**
 * Represents a point. Abstract, derived by point2d, point3d, and point4d.
 */
struct point {
    /** Fields */
    std::vector<double> coords;

    /** Constructors */
    explicit point(std::vector<double> coords);

    /** Setters */
    virtual void setCoords(std::vector<double> coords) = 0;

    /** Other Methods */
    /**
     * Return the Euclidean distance from this point to another.
     */
    double distanceTo(const point& other) const;

    /* Movement */
    virtual void move(const std::vector<double>& dPosition) = 0;
    virtual void move(const spatialVector& dPosition) = 0;
};
/**
 * Represents a point in 2 dimensions.
 */
struct point2d : public point {
    /** Fields */
    double x, y;

    /** Constructors */
    point2d();
    point2d(const point2d& other);
    point2d(double x, double y);

    /** Setters */
    void setCoords(std::vector<double> coords) override;
    void setCoords(double newX, double newY);

    /** Other Methods */
    /* Movement */
    void moveX(double dX);
    void moveY(double dY);
    void move(double dX, double dY);
    void move(const std::vector<double>& dPosition) override;
    void move(const spatialVector& dPosition) override;
};
/**
 * Represents a point in 3 dimensions.
 */
struct point3d : public point {
    /** Fields */
    double x, y, z;

    /** Constructors */
    point3d();
    point3d(const point3d& other);
    point3d(double x, double y, double z);

    /** Setters */
    void setCoords(std::vector<double> coords) override;
    void setCoords(double newX, double newY, double newZ);

    /** Other Methods */
    /* Movement */
    void moveX(double dX);
    void moveY(double dY);
    void moveZ(double dZ);
    void move(double dX, double dY, double dZ);
    void move(const std::vector<double>& dPosition) override;
    void move(const spatialVector& dPosition) override;
};
/**
 * Represents a point in 4 dimensions.
 */
struct point4d : public point {
    /** Fields */
    double x, y, z, a;

    /** Constructors */
    point4d();
    point4d(const point4d& other);
    point4d(double x, double y, double z, double a);

    /** Setters */
    void setCoords(std::vector<double> coords) override;
    void setCoords(double newX, double newY, double newZ, double newA);

    /** Other Methods */
    /* Movement */
    void moveX(double dX);
    void moveY(double dY);
    void moveZ(double dZ);
    void moveA(double dA);
    void move(double dX, double dY, double dZ, double dA);
    void move(const std::vector<double>& dPosition) override;
    void move(const spatialVector& dPosition) override;
};

/**
 * Stores pairs of pointers to point objects.
 */
struct edge {
    /** Virtual Destructor */
    virtual ~edge() = 0;

    /** Other Methods */
    /**
     * Return the Euclidean distance between the endpoints of this edge.
     */
    virtual double length() const = 0;
};

/**
 * Stores a pair of pointers to point2d objects.
 */
struct edge2d : edge {
    /** Fields */
    std::unique_ptr<point2d> p1, p2;
    std::pair<std::unique_ptr<point2d>, std::unique_ptr<point2d>> endpoints;

    /** Constructors */
    edge2d();
    edge2d(const edge2d& other);
    edge2d(const point2d& p1, const point2d& p2);

    /** Other Methods */
    /**
     * Return the Euclidean distance between the endpoints of this edge.
     */
    double length() const override;

    /**
     * Draws a line on the OpenGL screen between p1 and p2.
     */
    void draw() const;
};

/**
 * Stores a pair of pointers to point2d objects.
 */
struct edge3d : edge {
    /** Fields */
    std::unique_ptr<point3d> p1, p2;
    std::pair<std::unique_ptr<point3d>, std::unique_ptr<point3d>> endpoints;


    /** Constructors */
    edge3d();
    edge3d(const edge3d& other);
    edge3d(const point3d& p1, const point3d& p2);

    /** Other Methods */
    /**
     * Return the Euclidean distance between the endpoints of this edge.
     */
    double length() const override;
};

/**
 * Stores a pair of pointers to point2d objects.
 */
struct edge4d : edge {
    /** Fields */
    std::unique_ptr<point4d> p1, p2;
    std::pair<std::unique_ptr<point4d>, std::unique_ptr<point4d>> endpoints;


    /** Constructors */
    edge4d();
    edge4d(const edge4d& other);
    edge4d(const point4d& p1, const point4d& p2);

    /** Other Methods */
    /**
     * Return the Euclidean distance between the endpoints of this edge.
     */
    double length() const override;

};

/**
 * Stores n-1 angles that can be used to define any direction in
 * n-dimensional space. Abstract, derived by sphericalAngle3d and
 * sphericalAngle4d.
 */
struct sphericalAngle {
    /// Note: All calculations done in degrees
    /// Note: Left-handed coordinate system

    /** Fields */
    std::vector<double> angles;

    /** Constructors */
    explicit sphericalAngle(std::vector<double> angles);

    /** Getters */
    /**
     * Return the unit vector in the direction defined by this object.
     */
    virtual spatialVector getUnitVector() = 0;
    virtual void rotate(std::vector<double> angles) = 0;

};

/**
 * Stores 2 angles that can be used to define any direction in 3d space.
 */
struct sphericalAngle3d : public sphericalAngle {
    /// Note: All calculations done in degrees
    /// Note: Left-handed coordinate system

    /** Fields */
    // polarAngle rotates around the (vertical) z-axis, ranges from [0, 360)
    //      degrees and where 0 is towards the positive y-axis, and increasing
    //      rotates east (CCW rotation from top-down)
    // azimuthAngle ranges from [0, 180] degrees where 0 is straight up and
    //      180 is straight down (90 is forward)
    double polarAngle, azimuthAngle;

    /** Constructors */
    sphericalAngle3d();
    sphericalAngle3d(sphericalAngle3d const &other);
    sphericalAngle3d(double polarAngle, double azimuthAngle);

    /** Setters */
    /**
     * Set polar angle to given value, capping between [0, 360)
     */
    void setPolar(double newPolarAngle);
    /**
     * Set azimuth angle to given value, capping between [0, 180]
     */
    void setAzimuth(double newAzimuthAngle);

    /** Other Methods */
    /* Utility */
    /**
     * Return the unit vector pointing in the direction defined by this object.
     */
    spatialVector getUnitVector() override;

    /* Rotation */
    /**
     * Increase rotation of polarAngle by dPolarAngle degrees, staying within
     * [0, 360).
     */
    void rotatePolar(double dPolarAngle);
    /**
     * Increase rotation of azimuthAngle by dAzimuthAngle degrees, staying
     * within [0, 180].
     */
    void rotateAzimuth(double dAzimuthAngle);
    /**
     * Increase rotation of polarAngle and azimuthAngle by values of the given
     * vector (must be of size() 2).
     */
    void rotate(std::vector<double> dAngles) override;
};

/**
 * Stores 3 angles that can be used to define any direction in 4d space.
 */
struct sphericalAngle4d : public sphericalAngle {
    /// Note: All calculations done in degrees
    /// Note: Left-handed coordinate system
    /// Note: Read comments in sphericalAngle3d to make more sense of below

    // polarAngle rotates around the (vertical) z-axis, ranges from [0, 180]
    //      degrees and where 0 is towards the positive y-axis, and increasing
    //      rotates east (CCW rotation from top-down)
    // azimuthAngle ranges from [0, 180] degrees where 0 is straight up and
    //      180 is straight down (90 is forward)
    // phiAngle ranges from [0, 360) degrees, where 0 lies as it would in our 3
    //      dimensions, 90 lies orthogonally "outward" to the 3d dimension,
    //      180 lies opposite to 0 would in our 3 dimensions, and 270 lies
    //      orthogonally "inward" to the 3rd dimension (to give arbitrary
    //      names to 4d directions)
    double polarAngle, azimuthAngle, phiAngle;

    /** Constructors */
    sphericalAngle4d();
    sphericalAngle4d(const sphericalAngle4d& other);
    sphericalAngle4d(double polarAngle, double azimuthAngle, double phiAngle);

    /** Setters */
    /**
     * Set polar angle to given value, capping between [0, 180]
     */
    void setPolar(double newPolarAngle);
    /**
     * Set azimuth angle to given value, capping between [0, 180]
     */
    void setAzimuth(double newAzimuthAngle);
    /**
     * Set phi angle to given value, capping between [0, 360)
     */
    void setPhi(double newPhiAngle);

    /** Other Methods */
    /* Utility */
    /**
     * Return the vector pointing in the direction defined by this object.
     */
    spatialVector getUnitVector() override;

    /* Rotation */
    /**
     * Increase rotation of polarAngle by dPolarAngle degrees, staying within
     * [0, 180].
     */
    void rotatePolar(double dPolarAngle);
    /**
     * Increase rotation of azimuthAngle by dAzimuthAngle degrees, staying
     * within [0, 180].
     */
    void rotateAzimuth(double dAzimuthAngle);
    /**
     * Increase rotation of phiAngle by dPhiAngle degrees, staying within
     * [0, 180].
     */
    void rotatePhi(double dPhiAngle);
    /**
     * Increase rotation of polarAngle and azimuthAngle by values of the given
     * vector (must be of size() 3).
     */
    void rotate(std::vector<double> dAngles) override;

};

}

#endif //PART_2___GRAPHICS_ALTERNATE_UTILS_H
