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


/* ========== Functions ========== */

/**
 * Square a value.
 */
double square(double n);
/**
 * Convert a degree value to radians.
 */
double rad(double deg);
/**
 * Convert a radian value to degrees.
 */
double deg(double rad);
/**
 * Return `a % n`.
 */
double mod(double a, double n);
/**
 * Set `a` to `a % n` in-place.
 */
void modEquals(double *a, double n);
/**
 * Clamp the first argument to the `min` or `max` if it's out of range.
 * `min` must be less than `max`.
 */
double clamp(double n, double min, double max);
/**
 * Return true iff the first argument is between `min` and `max`.
 */
bool isBetween(double n, double min, double max);
/**
 * Return true iff the two arguments are both negative or both positive.
 * If either argument is `0`, returns true. Does not support the `(0, inf)` case.
 * https://stackoverflow.com/a/2922888/7304977
 */
bool sameSign(double n1, double n2);
/**
 * Return the average of the elements in the array.
 */
double average(std::vector<double>& arr);

/* ========== Structs ========== */
/**
 * Represents a vector with any number of dimensional components.
 */
struct spatialVector {
    std::vector<double> components;

    spatialVector();
    explicit spatialVector(int numDimensions);
    spatialVector(const spatialVector& other);
    explicit spatialVector(const std::vector<double>& components);

    /**
     * Add all of other's components to this's components.
     */
    void plus(const spatialVector& other);
    /**
    * Subtract all of other's components from this's components.
    */
    void minus(const spatialVector& other);
    /**
     * Multiply all components of a vector by the given scalar.
     */
    void scale(double scalar);
    /**
     * Return the magnitude of the vector.
     */
    double magnitude() const;
    /**
     * Compute the dot product of `this` and `other`.
     */
    double dot(const spatialVector& other) const;
    /**
     * Return the cosine of the angle between `this` and `other` in radians.
     */
    double cosOfAngleBetween(const spatialVector& other) const;
    /**
     * Return the scalar projection of `this` onto `other`.
     */
    double scalarProjectOnto(const spatialVector& other) const;
    /**
     * Return the magnitude of the vector rejection of `this` from `other`.
     */
    double scalarRejectFrom(const spatialVector& other) const;
};

/**
 * Represents a point. Abstract, derived by `point2d`, `point3d`, and `point4d`.
 */
struct point {
    /* Other Methods */
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

    /** Other Methods */
    /* Utility */
    /**
     * Return the Euclidean distance from this point to another.
     */
    double distanceTo(const point2d& other) const;

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
    /* Fields */
    double x, y, z;

    /* Constructors */
    point3d();
    point3d(const point3d& other);
    point3d(double x, double y, double z);

    /* Other Methods */
    /* Utility */
    /**
     * Return the Euclidean distance from this point to another.
     */
    double distanceTo(const point3d& other) const;

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
    /* Fields */
    double x, y, z, a;

    /* Constructors */
    point4d();
    point4d(const point4d& other);
    point4d(double x, double y, double z, double a);

    /* Other Methods */
    /* Utility */
    /**
     * Return the Euclidean distance from this point to another.
     */
    double distanceTo(const point4d& other) const;

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
    /* Virtual Destructor */
    virtual ~edge() = 0;

    /* Other Methods */
    /**
     * Return the Euclidean distance between the endpoints of this edge.
     */
    virtual double length() const = 0;
};

/**
 * Stores a pair of pointers to point2d objects.
 */
struct edge2d : edge {
    /* Fields */
    std::shared_ptr<point2d> p1, p2;

    /* Constructors */
    edge2d();
    edge2d(const edge2d& other);
    edge2d(const point2d& p1, const point2d& p2);

    /* Other Methods */
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
    /* Fields */
    std::shared_ptr<point3d> p1, p2;

    /* Constructors */
    edge3d();
    edge3d(const edge3d& other);
    edge3d(const point3d& p1, const point3d& p2);

    /* Other Methods */
    /**
     * Return the Euclidean distance between the endpoints of this edge.
     */
    double length() const override;
};

/**
 * Stores a pair of pointers to point2d objects.
 */
struct edge4d : edge {
    /* Fields */
    std::shared_ptr<point4d> p1, p2;

    /* Constructors */
    edge4d();
    edge4d(const edge4d& other);
    edge4d(const point4d& p1, const point4d& p2);

    /* Other Methods */
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

    /* Getters */
    /**
     * Return the unit vector in the direction defined by this object.
     */
    virtual spatialVector getUnitVector() = 0;
    virtual void rotate(const std::vector<double>& angles) = 0;
};

/**
 * Stores 2 angles that can be used to define any direction in 3d space.
 */
struct sphericalAngle3d : public sphericalAngle {
    /// Note: All calculations done in degrees
    /// Note: Left-handed coordinate system

    /* Fields */
    // polarAngle rotates around the (vertical) z-axis, ranges from [0, 360)
    //      degrees and where 0 is towards the positive y-axis, and increasing
    //      rotates east (CCW rotation from top-down)
    // azimuthAngle ranges from [0, 180] degrees where 0 is straight up and
    //      180 is straight down (90 is forward)
    double polarAngle, azimuthAngle;

    /* Constructors */
    sphericalAngle3d();
    sphericalAngle3d(sphericalAngle3d const &other);
    sphericalAngle3d(double polarAngle, double azimuthAngle);

    /* Setters */
    /**
     * Set polar angle to given value, capping between [0, 360)
     */
    void setPolar(double newPolarAngle);
    /**
     * Set azimuth angle to given value, capping between [0, 180]
     */
    void setAzimuth(double newAzimuthAngle);

    /* Other Methods */
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
    void rotate(const std::vector<double>& dAngles) override;
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

    /* Constructors */
    sphericalAngle4d();
    sphericalAngle4d(const sphericalAngle4d& other);
    sphericalAngle4d(double polarAngle, double azimuthAngle, double phiAngle);

    /* Setters */
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

    /* Other Methods */
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
    void rotate(const std::vector<double>& dAngles) override;

};



#endif //PART_2___GRAPHICS_ALTERNATE_UTILS_H
