//
// Created by Jackson Hall on 4/29/2020.
//

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



/** ========== Constants ========== */
const int DEFAULT_WINDOW_WIDTH = 960;
const int DEFAULT_WINDOW_HEIGHT = 540;
const double DEFAULT_RENDER_COLOR_RGB[3] = {0.0, 0.0, 0.0};
const double DEFAULT_BACKGROUND_COLOR_RGB[3] = {1.0, 1.0, 0.9};

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
    *a -= floor(*a / n) * n;
}

/** ========== Structs ========== */
/**
 * Represents a vector with any number of dimensional components.
 */
struct spatialVector {
    /** Fields */
    std::vector<double> components;

    /** Constructors */
    spatialVector() : spatialVector(std::vector<double>()){
    }
    spatialVector(const spatialVector& other) : spatialVector(other.components){
    }
    explicit spatialVector(const std::vector<double> components){
        this->components = components;
    }

    /** Other Methods */
    /* Utility */
    /**
     * Multiply all components of a vector by the given scalar.
     */
    void scale(const double scalar){
        for (auto & c : components){
            c *= scalar;
        }
    }
    /**
     * Return the magnitude of the vector.
     */
    double magnitude() const {
        double sumOfSquares = 0;

        for (auto & component : components){
            sumOfSquares += square(component);
        }

        return sqrt(sumOfSquares);
    }
    /**
     * Compute the dot product of this vector and another.
     */
    double dot(const spatialVector& other) const {
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
    /**
     * Return the cosine of the angle between this vector and another in
     * degrees.
     */
    double cosOfAngleBetween(const spatialVector& other) const {
        return this->dot(other) / (other.magnitude() * other.magnitude());
    }
    /**
     * Return the scalar projection of this vector onto another.
     */
    double scalarProjectOnto(const spatialVector& other) const{
        if (components.size() == other.components.size()){
            return magnitude() / cosOfAngleBetween(other);
        } else {
            // Bad input
            std::cout << "Warning: Invalid input in:\n\tdouble vectorProjection"
                         "(spatialVector v1, spatialVector v2)\n\t(utils.cpp)"
                         << std::endl;
            return -1;
        }
    }
    /**
     * Return the magnitude of the vector rejection of this from other.
     */
    double scalarRejectFrom(const spatialVector& other) const {
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
};

/**
 * Represents a point. Abstract, derived by point2d, point3d, and point4d.
 */
struct point {
    /** Fields */
    std::vector<std::unique_ptr<double>> coords;

    /** Constructors */
    explicit point(std::vector<std::unique_ptr<double>> coords) :
            coords(std::move(coords)){
    }

    /** Setters */
    virtual void setCoords(std::vector<double> coords) = 0;

    /** Other Methods */
    /**
     * Return the Euclidean distance from this point to another.
     */
    double distanceTo(const point& other) const {
        // Create two new vectors that are definitely of equal size().
        // Whichever is shorter than the other will be filled with 0.0s until
        // size()s are equal
        std::vector<double> coordsVec;
        std::vector<double> otherCoordsVec;

        // Push back all values from this point
        for (auto & coord : this->coords){
            coordsVec.push_back(*coord);
        }

        // Push back all values from other point
        for (auto & coord : other.coords){
            otherCoordsVec.push_back(*coord);
        }

        // Try filling both with 0s until vectors are the same size()
        while (coordsVec.size() < otherCoordsVec.size()){
            coordsVec.push_back(0.0);
        }
        while (otherCoordsVec.size() < coordsVec.size()){
            otherCoordsVec.push_back(0.0);
        }

        // Vectors are the same size now, add squares of each position and
        // return square root
        double sumOfSquares = 0;
        for (int i = 0; i < coordsVec.size(); ++i){
            sumOfSquares += square(coordsVec[i] - otherCoordsVec[i]);
        }

        return sqrt(sumOfSquares);
    }

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
    point2d() : point2d(0, 0){
    }
    point2d(const point2d& other) : point2d(other.x, other.y){
    }
    point2d(double x, double y) :
            point(std::vector<std::unique_ptr<double>>({
                std::make_unique<double>(x),
                std::make_unique<double>(y)
            })), x(x), y(y) {
    }

    /** Setters */
    void setCoords(std::vector<double> coords) override {
        if (coords.size() == 2){
            setCoords(coords[0], coords[1]);
        } else {
            std::cout << "Warning: Invalid input to:\n\tvoid setCoords"
                         "(std::vector<double> "
                         "coords)\n\t(point2d, utils.cpp)" << std::endl;
        }
    }
    void setCoords(double newX, double newY){
        this->x = newX;
        this->y = newY;
    }

    /** Other Methods */
    /* Movement */
    void moveX(double dX){
        x += dX;
    }
    void moveY(double dY){
        y += dY;
    }
    void move(double dX, double dY){
        moveX(dX);
        moveY(dY);
    }
    void move(const std::vector<double>& dPosition) override {
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
            std::cout << "Warning: Invalid input in:\n\tvoid move"
                    "(std::vector<double>& dPosition)\n\t(point2d, utils.cpp)"
                    << std::endl;
        }
    }
    void move(const spatialVector& dPosition) override {
        if (dPosition.components.size() <= 2){
            move(dPosition.components);

        } else {
            // Bad input
            std::cout << "Warning: Invalid input in:\n\tvoid move(const "
                    "spatialVector& dPosition) override\n\t(point2d, utils.cpp)"
                    << std::endl;
        }
    }
};
/**
 * Represents a point in 3 dimensions.
 */
struct point3d : public point {
    /** Fields */
    double x, y, z;

    /** Constructors */
    point3d() : point3d(0, 0, 0){
    }
    point3d(const point3d& other) : point3d(other.x, other.y, other.z){
    }
    point3d(double x, double y, double z) :
            point(std::vector<std::unique_ptr<double>>({
                std::make_unique<double>(x),
                std::make_unique<double>(y),
                std::make_unique<double>(z)
            })), x(x), y(y), z(z) {
    }
    
    /** Setters */
    void setCoords(std::vector<double> coords) override {
        if (coords.size() == 3){
            setCoords(coords[0], coords[1], coords[2]);
        } else {
            std::cout << "Warning: Invalid input to:\n\tvoid setCoords"
                         "(std::vector<double> "
                         "coords)\n\t(point3d, utils.cpp)" << std::endl;
        }
    }
    void setCoords(double newX, double newY, double newZ){
        // TODO get rid of this if it works
        std::cout << "Test: utils.cpp: point3d: setCoords(double newx...): "
                << std::endl;
        std::cout << "Before updating coords, point3d.coords contains:"
                << std::endl;
        for (auto & n : this->coords){
            std::cout << *n << " | ";
        }
        std::cout << std::endl;

        this->x = newX;
        this->y = newY;
        this->z = newZ;

        std::cout << "After updating coords, point3d.coords contains:"
                << std::endl;
        for (auto & n : this->coords){
            std::cout << *n << " | ";
        }
        std::cout << std::endl;
    }

    /** Other Methods */
    /* Movement */
    void moveX(double dX){
        x += dX;
    }
    void moveY(double dY){
        y += dY;
    }
    void moveZ(double dZ){
        z += dZ;
    }
    void move(double dX, double dY, double dZ){
        moveX(dX);
        moveY(dY);
        moveZ(dZ);
    }
    void move(const std::vector<double>& dPosition) override {
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
            std::cout << "Warning: Invalid input in:\n\tvoid move"
                    "(std::vector<double>& dPosition)\n\t(point3d, utils.cpp)"
                    << std::endl;
        }
    }
    void move(const spatialVector& dPosition) override {
        if (dPosition.components.size() <= 3){
            move(dPosition.components);
        } else {
            // Bad input
            std::cout << "Warning: Invalid input in:\n\tvoid move(const "
                    "spatialVector& dPosition) override\n\t(point3d, utils.cpp)"
                    << std::endl;
        }
    }
};
/**
 * Represents a point in 4 dimensions.
 */
struct point4d : public point {
    /** Fields */
    double x, y, z, a;

    /** Constructors */
    point4d() : point4d(0, 0, 0, 0){
    }
    point4d(const point4d& other) : point4d(other.x, other.y, other.z, other.a){
    }
    point4d(double x, double y, double z, double a) :
            point(std::vector<std::unique_ptr<double>>({
                std::make_unique<double>(x),
                std::make_unique<double>(y),
                std::make_unique<double>(z),
                std::make_unique<double>(a),
            })), x(x), y(y), z(z), a(a) {
    }

    /** Setters */
    void setCoords(std::vector<double> coords) override {
        if (coords.size() == 4){
            setCoords(coords[0], coords[1], coords[2], coords[3]);
        } else {
            std::cout << "Warning: Invalid input to:\n\tvoid setCoords"
                         "(std::vector<double> "
                         "coords)\n\t(point4d, utils.cpp)" << std::endl;
        }
    }
    void setCoords(double newX, double newY, double newZ, double newA){
        this->x = newX;
        this->y = newY;
        this->z = newZ;
        this->a = newA;
        this->coords = std::vector<std::unique_ptr<double>>({
                std::make_unique<double>(x),
                std::make_unique<double>(y),
                std::make_unique<double>(z),
                std::make_unique<double>(a)
        });
    }

    /** Other Methods */
    /* Movement */
    void moveX(double dX){
        x += dX;
    }
    void moveY(double dY){
        y += dY;
    }
    void moveZ(double dZ){
        z += dZ;
    }
    void moveA(double dA){
        a += dA;
    }
    void move(double dX, double dY, double dZ, double dA){
        moveX(dX);
        moveY(dY);
        moveZ(dZ);
        moveA(dA);
    }
    void move(const std::vector<double>& dPosition) override {
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
            std::cout << "Warning: Invalid input in:\n\tvoid move"
                    "(std::vector<double>& dPosition)\n\t(point4d, utils.cpp)"
                    << std::endl;
        }
    }
    void move(const spatialVector& dPosition) override {
        if (dPosition.components.size() <= 4){
            move(dPosition.components);

        } else {
            // Bad input
            std::cout << "Warning: Invalid input in:\n\tvoid move(const "
                    "spatialVector& dPosition) override\n\t(point4d, utils.cpp)"
                    << std::endl;
        }
    }
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
    edge2d() : 
            p1(nullptr), 
            p2(nullptr), 
            endpoints(std::pair<
                std::unique_ptr<point2d>, 
                std::unique_ptr<point2d>
            >(nullptr, nullptr)){
    }
    edge2d(const edge2d& other){
        p1 = std::make_unique<point2d>(*other.p1);
        p2 = std::make_unique<point2d>(*other.p2);
        endpoints = 
                std::make_pair<
                    std::unique_ptr<point2d>,
                    std::unique_ptr<point2d>
                >(
                std::make_unique<point2d>(*p1),
                std::make_unique<point2d>(*p2)
        );
    }
    edge2d(const point2d& p1, const point2d& p2){
        this->p1 = std::make_unique<point2d>(p1);
        this->p2 = std::make_unique<point2d>(p2);
        endpoints = std::make_pair<std::unique_ptr<point2d>, 
            std::unique_ptr<point2d>>(
            static_cast<std::unique_ptr<point2d> &&>(this->p1),
            static_cast<std::unique_ptr<point2d> &&>(this->p2)
        );
    }

    /** Other Methods */
    /**
     * Return the Euclidean distance between the endpoints of this edge.
     */
    double length() const override {
        return p1->distanceTo(*p2);
    }

    /**
     * Draws a line on the OpenGL screen between p1 and p2.
     */
    void draw() const {
        glBegin(GL_LINES);
        glLineWidth(1);
        glColor3f(
                DEFAULT_RENDER_COLOR_RGB[0],
                DEFAULT_RENDER_COLOR_RGB[1],
                DEFAULT_RENDER_COLOR_RGB[2]
        );
        glVertex2i(*endpoints.first->coords[0],
                *endpoints.first->coords[1]);
        glVertex2i(*endpoints.second->coords[0],
                    *endpoints.first->coords[1]);
        glEnd();
    }
};

/**
 * Stores a pair of pointers to point2d objects.
 */
struct edge3d : edge {
    /** Fields */
    std::unique_ptr<point3d> p1, p2;
    std::pair<std::unique_ptr<point3d>, std::unique_ptr<point3d>> endpoints;


    /** Constructors */
    edge3d() : 
            p1(nullptr), 
            p2(nullptr), 
            endpoints(std::pair<
                std::unique_ptr<point3d>, 
                std::unique_ptr<point3d>
            >(nullptr, nullptr)){
    }
    edge3d(const edge3d& other){
        p1 = std::make_unique<point3d>(*other.p1);
        p2 = std::make_unique<point3d>(*other.p2);
        endpoints = 
                std::make_pair<
                    std::unique_ptr<point3d>, 
                    std::unique_ptr<point3d>
                >(
                std::make_unique<point3d>(*p1),
                std::make_unique<point3d>(*p2)
        );
    }
    edge3d(const point3d& p1, const point3d& p2){
        this->p1 = std::make_unique<point3d>(p1);
        this->p2 = std::make_unique<point3d>(p2);
        endpoints = std::make_pair<std::unique_ptr<point3d>, 
            std::unique_ptr<point3d>>(
            static_cast<std::unique_ptr<point3d> &&>(this->p1),
            static_cast<std::unique_ptr<point3d> &&>(this->p2)
        );
    }

    /** Other Methods */
    /**
     * Return the Euclidean distance between the endpoints of this edge.
     */
    double length() const override {
        return p1->distanceTo(*p2);
    }
};

/**
 * Stores a pair of pointers to point2d objects.
 */
struct edge4d : edge {
    /** Fields */
    std::unique_ptr<point4d> p1, p2;
    std::pair<std::unique_ptr<point4d>, std::unique_ptr<point4d>> endpoints;


    /** Constructors */
    edge4d() :
            p1(nullptr), 
            p2(nullptr), 
            endpoints(std::pair<
                std::unique_ptr<point4d>, 
                std::unique_ptr<point4d>
            >(nullptr, nullptr)){
    }
    edge4d(const edge4d& other){
        p1 = std::make_unique<point4d>(*other.p1);
        p2 = std::make_unique<point4d>(*other.p2);
        endpoints = 
                std::make_pair<
                    std::unique_ptr<point4d>, 
                    std::unique_ptr<point4d>
                >(
                std::make_unique<point4d>(*p1),
                std::make_unique<point4d>(*p2)
        );
    }
    edge4d(const point4d& p1, const point4d& p2){
        this->p1 = std::make_unique<point4d>(p1);
        this->p2 = std::make_unique<point4d>(p2);
        endpoints = std::make_pair<std::unique_ptr<point4d>, 
            std::unique_ptr<point4d>>(
            static_cast<std::unique_ptr<point4d> &&>(this->p1),
            static_cast<std::unique_ptr<point4d> &&>(this->p2)
        );
    }

    /** Other Methods */
    /**
     * Return the Euclidean distance between the endpoints of this edge.
     */
    double length() const override {
        return p1->distanceTo(*p2);
    }

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
    std::vector<std::unique_ptr<double>> angles;

    /** Constructors */
    explicit sphericalAngle(std::vector<std::unique_ptr<double>> angles) :
            angles(std::move(angles)){
    }

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
    sphericalAngle3d() : sphericalAngle3d(0, 90){
    }
    sphericalAngle3d(sphericalAngle3d const &other) :
            sphericalAngle3d(other.polarAngle, other.azimuthAngle){
    }
    sphericalAngle3d(double polarAngle, double azimuthAngle) :
            sphericalAngle(std::vector<std::unique_ptr<double>>({
                std::make_unique<double>(polarAngle),
                std::make_unique<double>(azimuthAngle)
            })) {
        this->polarAngle = polarAngle;
        this->azimuthAngle = azimuthAngle;
    }

    /** Setters */
    /**
     * Set polar angle to given value, capping between [0, 360)
     */
    void setPolar(double newPolarAngle) {
        this->polarAngle = mod(newPolarAngle, 360);
    }
    /**
     * Set azimuth angle to given value, capping between [0, 180]
     */
    void setAzimuth(double newAzimuthAngle) {
        if (newAzimuthAngle < 0){
            this->azimuthAngle = 0;
        } else if (newAzimuthAngle > 180){
            this->azimuthAngle = 180;
        } else {
            this->azimuthAngle = newAzimuthAngle;
        }
    }

    /** Other Methods */
    /* Utility */
    /**
     * Return the unit vector pointing in the direction defined by this object.
     */
    spatialVector getUnitVector() override {
        // By definition, when an angle a is plotted on the unit circle, the
        // point where the angle's ray intersects the circle is
        // (cos(a), sin(a)). This method takes those coordinates and scales
        // them down towards the circle's center by the sin of the azimuth
        // (up/down)  angle to emulate the same point rotating up/down along
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
    
    /* Rotation */
    /**
     * Increase rotation of polarAngle by dPolarAngle degrees, staying within 
     * [0, 360).
     */
    void rotatePolar(double dPolarAngle){
        setPolar(polarAngle + dPolarAngle);
    }
    /**
     * Increase rotation of azimuthAngle by dAzimuthAngle degrees, staying 
     * within [0, 180].
     */
    void rotateAzimuth(double dAzimuthAngle){
        setAzimuth(azimuthAngle + dAzimuthAngle);
    }
    /**
     * Increase rotation of polarAngle and azimuthAngle by values of the given
     * vector (must be of size() 2).
     */
    void rotate(std::vector<double> dAngles) override {
        if (angles.size() == 2){
            rotatePolar(dAngles[0]);
            rotateAzimuth(dAngles[1]);
        } else {
            // Bad input
            std::cout << "Warning: Invalid input in:\n\tvoid "
                         "rotate(std::vector<double> angles) override\n\t"
                         "(sphericalAngle3d, utils.cpp)" << std::endl;
        }
    }
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
    sphericalAngle4d() : sphericalAngle4d(0, 90, 90){
    }
    sphericalAngle4d(const sphericalAngle4d& other) : sphericalAngle4d(
            other.polarAngle, other.azimuthAngle, other.phiAngle) {
    }
    sphericalAngle4d(double polarAngle, double azimuthAngle, double phiAngle) :
            sphericalAngle(std::vector<std::unique_ptr<double>>({
                std::make_unique<double>(polarAngle),
                std::make_unique<double>(azimuthAngle),
                std::make_unique<double>(phiAngle),
            })) {
        this->polarAngle = polarAngle;
        this->azimuthAngle = azimuthAngle;
        this->phiAngle = phiAngle;
    }

    /** Setters */
    /**
     * Set polar angle to given value, capping between [0, 180]
     */
    void setPolar(const double newPolarAngle) {
        if (newPolarAngle < 0){
            this->polarAngle = 0;
        } else if (newPolarAngle > 180){
            this->polarAngle = 180;
        } else {
            this->polarAngle = newPolarAngle;
        }
    }
    /**
     * Set azimuth angle to given value, capping between [0, 180]
     */
    void setAzimuth(const double newAzimuthAngle) {
        if (newAzimuthAngle < 0){
            this->azimuthAngle = 0;
        } else if (newAzimuthAngle > 180){
            this->azimuthAngle = 180;
        } else {
            this->azimuthAngle = newAzimuthAngle;
        }
    }
    /**
     * Set phi angle to given value, capping between [0, 360)
     */
    void setPhi(const double newPhiAngle) {
        this->phiAngle = mod(newPhiAngle, 360);
    }
    
    /** Other Methods */
    /* Utility */
    /**
     * Return the vector pointing in the direction defined by this object.
     */
    spatialVector getUnitVector() override {
        // It would seem at first that only the +x hemisphere of the unit
        // 3-sphere can be reached with polarAngle and azimuthAngle both only
        // ranging from [0-180], but every point on this hemisphere can be
        // mapped to the antipodal (opposite point) with a rotation through
        // the 4th dimension by 180 degrees. For example, to reach (-1, 0, 0,
        // 0), these values work with the math that has been implemented below:
        //      polarAngle = 90 deg = pi/2 rad
        //      azimuthAngle = 90 deg
        //      phiAngle = 270 deg = 3pi/2 rad
        // because
        //      x = sin(phiAngle)sin(azimuthAngle)sin(polarAngle) = -1
        //      y = sin(phiAngle)sin(azimuthAngle)cos(polarAngle) = 0
        //      z = sin(phiAngle)cos(azimuthAngle) = 0
        //      a = cos(phiAngle) = 0
        return spatialVector(std::vector<double>({
            sin(rad(phiAngle)) * sin(rad(azimuthAngle)) * sin(rad(polarAngle)),
            sin(rad(phiAngle)) * sin(rad(azimuthAngle)) * cos(rad(polarAngle)),
            sin(rad(phiAngle)) * cos(rad(azimuthAngle)),
            cos(rad(phiAngle))
        }));
    }
    
    /* Rotation */
    /**
     * Increase rotation of polarAngle by dPolarAngle degrees, staying within 
     * [0, 180].
     */
    void rotatePolar(double dPolarAngle){
        setPolar(polarAngle + dPolarAngle);
    }
    /**
     * Increase rotation of azimuthAngle by dAzimuthAngle degrees, staying 
     * within [0, 180].
     */
    void rotateAzimuth(double dAzimuthAngle){
        setAzimuth(azimuthAngle + dAzimuthAngle);
    }
    /**
     * Increase rotation of phiAngle by dPhiAngle degrees, staying within 
     * [0, 180].
     */
    void rotatePhi(double dPhiAngle){
        setPhi(phiAngle + dPhiAngle);
    }
    /**
     * Increase rotation of polarAngle and azimuthAngle by values of the given
     * vector (must be of size() 3).
     */
    void rotate(std::vector<double> dAngles) override {
        if (angles.size() == 3){
            rotatePolar(dAngles[0]);
            rotateAzimuth(dAngles[1]);
            rotatePhi(dAngles[2]);
        } else {
            // Bad input
            std::cout << "Warning: Invalid input in:\n\tvoid "
                         "rotate(std::vector<double> angles) override\n\t"
                         "(sphericalAngle4d, utils.cpp)" << std::endl;
        }
    }
};
