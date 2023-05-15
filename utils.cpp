//
// Created by Jackson Hall on 4/29/2020.
//

#include "utils.h"
#include "graphics.h"
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


/** ========== Functions ========== */
/**
 * Return the square of the given double value.
 */
double square(double n){
    return pow(n, 2);
}

/**
 * Return the given degrees value converted to radians.
 */
double rad(double deg){
    return deg * M_PI / 180;
}

double deg(double rad){
    return rad * 180 / M_PI;
}

/**
 * Return a % n, which is not natively defined for doubles.
 */
double mod(double a, double n){
    return a - floor(a / n) * n;
}

/**
 * Set a to a % n, which is not natively defined for doubles.
 */
void modEquals(double *a, double n){
    *a = mod(*a, n);
}




/** ========== Structs ========== */
/** ===== Start spatialVector Struct ===== */
    /**
     * Represents a vector with any number of dimensional components.
     */
    spatialVector::spatialVector() : spatialVector(0){
    }

    spatialVector::spatialVector(const int numDimensions) :
            spatialVector(std::vector<double>(numDimensions, 0.0)){
    }

    spatialVector::spatialVector(const spatialVector& other) : spatialVector(other.components){
    }

    spatialVector::spatialVector(const std::vector<double>& components){
        this->components = components;
    }

    void spatialVector::plus(const spatialVector& other){
        if (other.components.size() <= components.size()){
            for (int i = 0; i < other.components.size(); ++i){
                components[i] += other.components[i];
            }
        } else {
            std::cout << "Warning: Invalid input to:\n\tvoid "
                         "spatialVector::plus(const spatialVector& other)\n\t"
                         "(utils.cpp)" << std::endl;
        }
    }

    void spatialVector::minus(const spatialVector& other){
        if (other.components.size() <= components.size()){
            for (int i = 0; i < other.components.size(); ++i){
                components[i] -= other.components[i];
            }
        } else {
            std::cout << "Warning: Invalid input to:\n\tvoid "
                         "spatialVector::minus(const spatialVector& other)\n\t"
                         "(utils.cpp)" << std::endl;
        }
    }

    void spatialVector::scale(const double scalar){
        for (auto & c : components){
            c *= scalar;
        }
    }

    double spatialVector::magnitude() const {
        double sumOfSquares = 0;

        for (auto & component : components){
            sumOfSquares += square(component);
        }

        return sqrt(sumOfSquares);
    }

    double spatialVector::dot(const spatialVector& other) const {
        if (components.size() == other.components.size()){
            // Sum products of components
            double sum = 0;
            for (int i = 0; i < components.size(); ++i){
                sum += components[i] * other.components[i];
            }
            return sum;

        } else {
            // Bad input
            std::cout << "Warning: Invalid input in:\n\tdouble dot(spatialVector "
                         "v1, spatialVector v2)\n\t(utils.cpp)" << std::endl;
            return -1;
        }
    }

    double spatialVector::cosOfAngleBetween(const spatialVector& other) const {
        return this->dot(other) / (magnitude() * other.magnitude());
    }

    double spatialVector::scalarProjectOnto(const spatialVector& other) const{
        if (components.size() == other.components.size()){
            return this->dot(other) / other.magnitude();
        } else {
            // Bad input
            std::cout << "Warning: Invalid input in:\n\tdouble vectorProjection"
                         "(spatialVector v1, spatialVector v2)\n\t(utils.cpp)"
                         << std::endl;
            std::cout << "other.components: ";
            for (auto& e : other.components){
                std::cout << e << " | ";
            }
            std::cout << std::endl;
            return -1;
        }
    }

    double spatialVector::scalarRejectFrom(const spatialVector& other) const {
        if (components.size() == other.components.size()){
            std::vector<double> tempComponents;

            double tempScalar = this->dot(other) / other.dot(other);

            tempComponents.reserve(components.size());
            for (int i = 0; i < components.size(); ++i){
                tempComponents.push_back(
                        components[i] - tempScalar * other.components[i]
                );
            }

            return spatialVector(tempComponents).magnitude();
        } else {
            // Bad input
            std::cout << "Warning: Invalid input in:\n\tdouble "
                         "scalarRejectFrom(const spatialVector& other)\n\t"
                         "(utils.cpp)" << std::endl;
            return -1;
        }
    }
/** ===== End spatialVector Struct ===== */




/** ===== Start point2d : point Struct ===== */
    /**
     * Represents a point in 2 dimensions.
     */
    point2d::point2d() : point2d(0, 0){
    }

    point2d::point2d(const point2d& other) : point2d(other.x, other.y){
    }

    point2d::point2d(double x, double y) : x(x), y(y){
    }

    double point2d::distanceTo(const point2d& other) const {
        return sqrt(square(this->x - other.x) + square(this->y - other.y));
    }

    void point2d::moveX(double dX){
        x += dX;
    }

    void point2d::moveY(double dY){
        y += dY;
    }

    void point2d::move(double dX, double dY){
        moveX(dX);
        moveY(dY);
    }

    void point2d::move(const std::vector<double>& dPosition){
        if (dPosition.size() <= 2){
            // Can move in at most 2 directions
            // Create a new vector of length 2 (fill remaining
            // dimensions with 0s to make it possible to use a
            // spacialVector of size() < 2)
            std::vector<double> dPositionVec = dPosition;

            while (dPositionVec.size() < 2){
                dPositionVec.push_back(0.0);
            }

            move(dPositionVec[0], dPositionVec[1]);
        } else {
            std::cout << "Warning: Invalid input in:\n\tvoid moveAbsolute"
                    "(std::vector<double>& dPosition)\n\t(point2d, utils.cpp)"
                    << std::endl;
        }
    }

    void point2d::move(const spatialVector& dPosition){
        if (dPosition.components.size() <= 2){
            move(dPosition.components);
        } else {
            // Bad input
            std::cout << "Warning: Invalid input in:\n\tvoid moveAbsolute(const "
                    "spatialVector& dPosition) override\n\t(point2d, utils.cpp)"
                    << std::endl;
        }
    }
/** ===== End point2d : point Struct ===== */




/** ===== Start point3d : point Struct ===== */
    /**
     * Represents a point in 3 dimensions.
     */
    point3d::point3d() : point3d(0, 0, 0){
    }

    point3d::point3d(const point3d& other) : point3d(other.x, other.y, other.z){
    }

    point3d::point3d(double x, double y, double z) : x(x), y(y), z(z){
    }

    double point3d::distanceTo(const point3d& other) const {
        return sqrt(
                square(this->x - other.x)
                + square(this->y - other.y)
                + square(this->z - other.z)
        );
    }

    void point3d::moveX(double dX){
        x += dX;
    }

    void point3d::moveY(double dY){
        y += dY;
    }

    void point3d::moveZ(double dZ){
        z += dZ;
    }

    void point3d::move(double dX, double dY, double dZ){
        moveX(dX);
        moveY(dY);
        moveZ(dZ);
    }

    void point3d::move(const std::vector<double>& dPosition){
        if (dPosition.size() <= 3){
            // Can move in at most 3 directions
            // Create a new vector of length 3 (fill remaining
            // dimensions with 0s to make it possible to use a
            // spacialVector of size() < 3)
            std::vector<double> dPositionVec = dPosition;

            while (dPositionVec.size() < 3){
                dPositionVec.push_back(0.0);
            }

            move(dPositionVec[0], dPositionVec[1], dPositionVec[2]);
        } else {
            std::cout << "Warning: Invalid input in:\n\tvoid moveAbsolute"
                    "(std::vector<double>& dPosition)\n\t(point3d, utils.cpp)"
                    << std::endl;
        }
    }

    void point3d::move(const spatialVector& dPosition){
        if (dPosition.components.size() <= 3){
            move(dPosition.components);
        } else {
            // Bad input
            std::cout << "Warning: Invalid input in:\n\tvoid moveAbsolute(const "
                    "spatialVector& dPosition) override\n\t(point3d, utils.cpp)"
                    << std::endl;
        }
    }
/** ===== End point3d : point Struct ===== */




/** ===== Start point4d : point Struct ===== */
    /**
     * Represents a point in 4 dimensions.
     */
    point4d::point4d() : point4d(0, 0, 0, 0){
    }

    point4d::point4d(const point4d& other) : point4d(other.x, other.y, other.z, other.a){
    }

    point4d::point4d(const double x, const double y, const double z,
        const double a) :
            x(x), y(y), z(z), a(a){
    }

    double point4d::distanceTo(const point4d& other) const {
        return sqrt(
                square(this->x - other.x)
                + square(this->y - other.y)
                + square(this->z - other.z)
                + square(this->a - other.a)
        );
    }

    void point4d::moveX(double dX){
        x += dX;
    }

    void point4d::moveY(double dY){
        y += dY;
    }

    void point4d::moveZ(double dZ){
        z += dZ;
    }

    void point4d::moveA(double dA){
        a += dA;
    }

    void point4d::move(double dX, double dY, double dZ, double dA){
        moveX(dX);
        moveY(dY);
        moveZ(dZ);
        moveA(dA);
    }

    void point4d::move(const std::vector<double>& dPosition){
        if (dPosition.size() <= 4){
            // Can move in at most 4 directions
            // Create a new vector of length 4 (fill remaining
            // dimensions with 0s to make it possible to use a
            // spacialVector of size() < 4)
            std::vector<double> dPositionVec = dPosition;

            while (dPositionVec.size() < 4){
                dPositionVec.push_back(0.0);
            }

            move(dPositionVec[0], dPositionVec[1], dPositionVec[2],
                dPositionVec[3]);
        } else {
            std::cout << "Warning: Invalid input in:\n\tvoid moveAbsolute"
                    "(std::vector<double>& dPosition)\n\t(point4d, utils.cpp)"
                    << std::endl;
        }
    }

    void point4d::move(const spatialVector& dPosition){
        if (dPosition.components.size() <= 4){
            move(dPosition.components);
        } else {
            // Bad input
            std::cout << "Warning: Invalid input in:\n\tvoid moveAbsolute(const "
                    "spatialVector& dPosition) override\n\t(point4d, utils.cpp)"
                    << std::endl;
        }
    }
/** ===== End point4d : point Struct ===== */




/** ===== Start edge Struct ===== */
    /**
     * Stores pairs of pointers to point objects.
     */
    edge::~edge() = default;
/** ===== End edge Struct ===== */




/** ===== Start edge2d : edge Struct ===== */
    /**
     * Stores a pair of pointers to point2d objects.
     */
    edge2d::edge2d() : p1(nullptr), p2(nullptr){
    }

    edge2d::edge2d(const edge2d& other) : edge2d(*other.p1, *other.p2){
    }

    edge2d::edge2d(const point2d& p1, const point2d& p2){
        this->p1 = std::make_shared<point2d>(p1);
        this->p2 = std::make_shared<point2d>(p2);
    }

    double edge2d::length() const {
        return p1->distanceTo(*p2);
    }

    void edge2d::draw() const {
        glBegin(GL_LINES);
        glLineWidth(1);
        glColor3f(
            DEFAULT_LINE_COLOR_RGB[0],
            DEFAULT_LINE_COLOR_RGB[1],
            DEFAULT_LINE_COLOR_RGB[2]
        );
        glVertex2i(ORTHO_ZOOM * this->p1->x, ORTHO_ZOOM * this->p1->y);
        glVertex2i(ORTHO_ZOOM * this->p2->x, ORTHO_ZOOM * this->p2->y);
        glEnd();
    }
/** ===== End edge2d : edge Struct ===== */




/** ===== Start edge3d : edge Struct ===== */
    /**
     * Stores a pair of pointers to point2d objects.
     */
    edge3d::edge3d() : p1(nullptr), p2(nullptr){
    }

    edge3d::edge3d(const edge3d& other) : edge3d(*other.p1, *other.p2){
    }

    edge3d::edge3d(const point3d& p1, const point3d& p2){
        this->p1 = std::make_shared<point3d>(p1);
        this->p2 = std::make_shared<point3d>(p2);
    }

    double edge3d::length() const {
        return p1->distanceTo(*p2);
    }
/** ===== End edge3d : edge Struct ===== */




/** ===== Start edge4d : edge Struct ===== */
    /**
     * Stores a pair of pointers to point2d objects.
     */
    edge4d::edge4d() : p1(nullptr), p2(nullptr){
    }

    edge4d::edge4d(const edge4d& other) : edge4d(*other.p1, *other.p2){
    }

    edge4d::edge4d(const point4d& p1, const point4d& p2){
        this->p1 = std::make_shared<point4d>(p1);
        this->p2 = std::make_shared<point4d>(p2);
    }

    double edge4d::length() const {
        return p1->distanceTo(*p2);
    }
/** ===== End edge4d : edge Struct ===== */




/** ===== Start sphericalAngle Struct ===== */
    /**
     * Stores n-1 angles that can be used to define any direction in
     * n-dimensional space. Abstract, derived by sphericalAngle3d and
     * sphericalAngle4d.
     */
/** ===== End sphericalAngle Struct ===== */




/** ===== Start sphericalAngle3d : sphericalAngle Struct ===== */
    /**
     * Stores 2 angles that can be used to define any direction in 3d space.
     */
    sphericalAngle3d::sphericalAngle3d() : sphericalAngle3d(0, 90){
    }
    sphericalAngle3d::sphericalAngle3d(sphericalAngle3d const &other) :
            sphericalAngle3d(other.polarAngle, other.azimuthAngle){
    }
    sphericalAngle3d::sphericalAngle3d(double polarAngle, double azimuthAngle) :
            polarAngle(polarAngle), azimuthAngle(azimuthAngle){
    }
    void sphericalAngle3d::setPolar(double newPolarAngle){
        this->polarAngle = mod(newPolarAngle, 360);
    }
    void sphericalAngle3d::setAzimuth(double newAzimuthAngle){
        if (newAzimuthAngle < 0){
            this->azimuthAngle = 0;
        } else if (newAzimuthAngle > 180){
            this->azimuthAngle = 180;
        } else {
            this->azimuthAngle = newAzimuthAngle;
        }
    }
    /**
     * Return a spatialVector of length 1 facing in the direction of this
     * spherical angle.
     */
    spatialVector sphericalAngle3d::getUnitVector(){
        // By definition, when an angle a is plotted on the unit circle, the
        // point where the angle's ray intersects the circle is
        // (cos(a), sin(a)). This method takes those coordinates and scales
        // them down towards the circle's center by the sin of the azimuth
        // (up/down) angle to emulate the same point rotating up/down along
        // the unit sphere. Because the azimuthAngle is measured from the
        // z-axis to the point, the sin of the angle represents the non-vertical
        // component of that angle, which is parallel to the xy plane. If you
        // put these equations in Desmos:
        //      a = 1 (range 0-2pi)
        //      b = 1 (range 0-pi)
        //      (sin(b)sin(a), sin(b)cos(a)) // (and z would = cos(b))
        //      x^2+y^2=1
        // it will appear to be a top-down view of a point on the unit
        // sphere rotating up/down (b) and around (a).
        return spatialVector(std::vector<double>({
            sin(rad(azimuthAngle)) * sin(rad(polarAngle)),
            sin(rad(azimuthAngle)) * cos(rad(polarAngle)),
            cos(rad(azimuthAngle))
        }));
    }
    void sphericalAngle3d::rotatePolar(double dPolarAngle){
        setPolar(polarAngle + dPolarAngle);
    }
    void sphericalAngle3d::rotateAzimuth(double dAzimuthAngle){
        setAzimuth(azimuthAngle + dAzimuthAngle);
    }
    void sphericalAngle3d::rotate(const std::vector<double>& dAngles){
        if (dAngles.size() == 2){
            rotatePolar(dAngles[0]);
            rotateAzimuth(dAngles[1]);
        } else {
            // Bad input
            std::cout << "Warning: Invalid input in:\n\tvoid "
                         "rotate(const std::vector<double>& angles) "
                         "override\n\t(sphericalAngle3d, utils.cpp)"
                         << std::endl;
        }
    }

/** ===== End sphericalAngle3d : sphericalAngle Struct ===== */




/** ===== Start sphericalAngle4d : sphericalAngle Struct ===== */
/**
 * Stores 3 angles that can be used to define any direction in 4d space.
 */
    sphericalAngle4d::sphericalAngle4d() : sphericalAngle4d(0, 90, 90){
    }
    sphericalAngle4d::sphericalAngle4d(const sphericalAngle4d& other) :
        sphericalAngle4d(other.polarAngle, other.azimuthAngle, other.phiAngle){
    }
    sphericalAngle4d::sphericalAngle4d(
            double polarAngle, double azimuthAngle, double phiAngle) :
            polarAngle(polarAngle),
            azimuthAngle(azimuthAngle),
            phiAngle(phiAngle){
    }
    void sphericalAngle4d::setPolar(const double newPolarAngle){
        if (newPolarAngle < 0){
            this->polarAngle = 0;
        } else if (newPolarAngle > 180){
            this->polarAngle = 180;
        } else {
            this->polarAngle = newPolarAngle;
        }
    }
    void sphericalAngle4d::setAzimuth(const double newAzimuthAngle){
        if (newAzimuthAngle < 0){
            this->azimuthAngle = 0;
        } else if (newAzimuthAngle > 180){
            this->azimuthAngle = 180;
        } else {
            this->azimuthAngle = newAzimuthAngle;
        }
    }
    void sphericalAngle4d::setPhi(const double newPhiAngle){
        this->phiAngle = mod(newPhiAngle, 360);
    }
    spatialVector sphericalAngle4d::getUnitVector(){
        // It would seem at first that only the +x hemisphere of the unit
        // 3-sphere can be reached with polarAngle and azimuthAngle both only
        // ranging from [0-180], but every point on this hemisphere can be
        // mapped to the antipodal (opposite point) with a rotation through
        // the 4th dimension by 180 degrees. For example, to reach (-1, 0, 0,
        // 0), these values work with the math that has been implemented below:
        //      polarAngle = 90 deg = pi/2 rad
        //      azimuthAngle = 90 deg = pi/2 rad
        //      phiAngle = 270 deg = 3pi/2 rad
        // because
        //      x = sin(3pi/2)sin(pi/2)sin(pi/2) = -1
        //      y = sin(3pi/2)sin(pi/2)cos(pi/2) = 0
        //      z = sin(3pi/2)cos(pi/2) = 0
        //      a = cos(3pi/2) = 0
        return spatialVector(std::vector<double>({
            sin(rad(phiAngle)) * sin(rad(azimuthAngle)) * sin(rad(polarAngle)),
            sin(rad(phiAngle)) * sin(rad(azimuthAngle)) * cos(rad(polarAngle)),
            sin(rad(phiAngle)) * cos(rad(azimuthAngle)),
            cos(rad(phiAngle))
        }));
    }
    void sphericalAngle4d::rotatePolar(double dPolarAngle){
        setPolar(polarAngle + dPolarAngle);
    }
    void sphericalAngle4d::rotateAzimuth(double dAzimuthAngle){
        setAzimuth(azimuthAngle + dAzimuthAngle);
    }
    void sphericalAngle4d::rotatePhi(double dPhiAngle){
        setPhi(phiAngle + dPhiAngle);
    }
    void sphericalAngle4d::rotate(const std::vector<double>& dAngles){
        if (dAngles.size() == 3){
            rotatePolar(dAngles[0]);
            rotateAzimuth(dAngles[1]);
            rotatePhi(dAngles[2]);
        } else {
            // Bad input
            std::cout << "Warning: Invalid input in:\n\tvoid "
                         "rotate(const std::vector<double>& angles) "
                         "override\n\t(sphericalAngle4d, utils.cpp)"
                         << std::endl;
        }
    }
/** ===== End sphericalAngle4d : sphericalAngle Struct ===== */
