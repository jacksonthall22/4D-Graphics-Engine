//
// Created by Jackson Hall on 4/29/2020.
//

#include <utility>
#include <vector>
#include <cmath>
#include <stdexcept>
#include <iostream>


/** ========== Functions ========== */
/**
 * Return the square of the given double value.
 */
double square(double n){
    return pow(n, 2);
}

/** ========== Structs ========== */
/**
 * Represents a vector with any number of dimensional components.
 */
struct spatialVector {
    std::vector<double> components;

    explicit spatialVector(std::vector<double> components){
        this->components = move(components);
    }

    /**
     * Return the magnitude of the vector.
     */
    double magnitude(){
        double sumOfSquares = 0;

        for (auto & component : components){
            sumOfSquares += square(component);
        }

        return sqrt(sumOfSquares);
    }
};

/**
 * Represents a point. Abstract, derived by point3d and point4d.
 */
struct point {
    std::vector<double*> coords;

    /** Setters */
    virtual void setCoords(std::vector<double> coords) = 0;

    /** Other Methods */
    /* Utility */
    /**
     * Return the Euclidean distance from this point to another.
     */
    virtual double distanceTo(const point& other) = 0;

    /* Movement */
    virtual void move(const std::vector<double>& dPosition) = 0;
    virtual void move(const spatialVector& dPosition) = 0;
};

/**
 * Represents a point in 3 dimensions.
 */
struct point3d : public point {
    double x, y, z;

    /** Constructors */
    point3d() : point3d(0, 0, 0){
    }
    point3d(double x, double y, double z){
        this->x = x;
        this->y = y;
        this->z = z;
        coords = {&x, &y, &z};
    }
    
    /** Setters */
    void setCoords(std::vector<double> coords) override {
        if (coords.size() == 3){
            setCoords(coords[0], coords[1], coords[2]);
        } else {
            std::cout << "Invalid input to:\n\tvoid setCoords(std::vector<double> "
                         "coords)\n\t(utils.cpp)" << std::endl;
        }
    }
    void setCoords(double newX, double newY, double newZ){
        // TODO get rid of this if it works
        std::cout << "Test: utils.cpp: point3d: setCoords(double newx...): "
                << std::endl;
        std::cout << "Before updating coords, point3d.coords contains:"
                << std::endl;
        for (auto & n : this->coords){
            std::cout << n << " | ";
        }
        std::cout << std::endl;

        this->x = newX;
        this->y = newY;
        this->z = newZ;

        std::cout << "After updating coords, point3d.coords contains:"
                << std::endl;
        for (auto & n : this->coords){
            std::cout << n << " | ";
        }
        std::cout << std::endl;
    }

    /** Other Methods */
    /* Utility */
    /**
     * Return the Euclidean distance from this point to another.
     */
    double distanceTo(const point& other) override {
        if (other.coords.size() == 3){
            // Regular 3d distance
            return sqrt(square(x - *other.coords[0])
                    + square(y - *other.coords[1])
                    + square(z - *other.coords[2]));
        } else if (other.coords.size() == 4){
            // This 3d point has no distance into 4th dimension (a),
            // so don't calculate difference in that direction,
            // just use value from other coords
            return sqrt(square(x - *other.coords[0])
                    + square(y - *other.coords[1])
                    + square(z - *other.coords[2])
                    + square(*other.coords[3]));
        } else {
            // Bad input
            std::cout << "Invalid input to:\n\tdouble distanceTo"
                         "(std::vector<double> "
                    "otherCoords) override\n\t(utils.cpp)" << std::endl;
            return -1;
        }
    }

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
        if (dPosition.size() == 3){
            move(dPosition[0], dPosition[1], dPosition[2]);
        } else {
            std::cout << "Warning: Invalid input in:\n\tvoid move(std::vector<double> "
                    "dPosition)"
                    "\n\t(utils.cpp)" << std::endl;
        }
    }
    void move(const spatialVector& dPosition) override {
        if (dPosition.components.size() <= 3){
            // Can move in at most 4 directions
            // Create a new vector of length 4 (fill remaining
            // dimensions with 0s to make it possible to use a
            // spacialVector of size() < 4)
            std::vector<double> dPositionVec = dPosition.components;

            while (dPositionVec.size() < 3){
                dPositionVec.push_back(0.0);
            }

            move(dPositionVec);
        }
    }
};

/**
 * Represents a point in 4 dimensions.
 */
struct point4d : public point {
    double x, y, z, a;

    /** Constructors */
    point4d() : point4d(0, 0, 0, 0){
    }
    point4d(double x, double y, double z, double a){
        this->x = x;
        this->y = y;
        this->z = z;
        this->a = a;
        this->coords = std::vector<double*>({&x, &y, &z, &a});
    }

    /** Other Methods */
    /* Utility */
    /**
     * Returns the Euclidean distance from this point to another.
     */
    double distanceTo(const point& other) override {
        if (other.coords.size() == 3){
            // A 3d point has no distance into 4th dimension (a),
            // so don't calculate difference in a direction,
            // just use value from other coords
            return sqrt(square(x - *other.coords[0])
                    + square(y - *other.coords[1])
                    + square(z - *other.coords[2])
                    + square(a));
        } else if (other.coords.size() == 4){
            // Regular 4d distance
            return sqrt(square(x - *other.coords[0])
                    + square(y - *other.coords[1])
                    + square(z - *other.coords[2])
                    + square(a - *other.coords[3]));
        } else {
            // Bad input
            std::cout << "Invalid input in:\n\tdouble distanceTo(std::vector<double> "
                    "otherCoords) override\n\t(utils.cpp)" << std::endl;
            return -1;
        }
    }

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
        if (dPosition.size() == 4){
            move(dPosition[0], dPosition[1], dPosition[2], dPosition[3]);
        } else {
            std::cout << "Invalid input in:\n\tvoid move(std::vector<double> "
                    "dPosition)\n\t(utils.cpp)" << std::endl;
        }
    }
    void move(const spatialVector& dPosition) override {
        if (dPosition.components.size() <= 4){
            // Can move in at most 4 directions
            // Create a new vector of length 4 (fill remaining
            // dimensions with 0s to make it possible to use a
            // spacialVector of size() < 4)
            std::vector<double> dPositionVec = dPosition.components;

            while (dPositionVec.size() < 4){
                dPositionVec.push_back(0.0);
            }

            move(dPositionVec);
        }
    }
};

/**
 * Stores pairs of pointers to point objects.
 */
struct edge {
    point *p1, *p2;
    std::pair<point*, point*> endpoints;

    edge(point *p1, point *p2){
        this->p1 = (point *) p1;
        this->p2 = (point *) p2;
        endpoints = std::make_pair(p1, p2);
    }

    /**
     * Return the Euclidean distance between the endpoints of this edge.
     */
    double length(){
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

    std::vector<double*> angles;

    /** Getters */
    /**
     * Return the unit vector in the direction defined by this object.
     */
    virtual spatialVector getVectorDirection() = 0;
    virtual void rotate(std::vector<double> angles) = 0;
};

/**
 * Stores 2 angles that can be used to define any direction in 3d space.
 */
struct sphericalAngle3d : public sphericalAngle {
    /// Note: All calculations done in degrees

    // polarAngle ranges from [0, 360) degrees where 0 is "north"
    //      and increasing the angle rotates east (90 is east, etc.)
    // azimuthAngle ranges from [0, 180] degrees where 0 is
    //      straight up and 180 is straight down (90 is forward)
    double polarAngle, azimuthAngle;

    /** Constructors */
    sphericalAngle3d() : sphericalAngle3d(0, 90){
    }
    sphericalAngle3d(double polarAngle, double azimuthAngle){
        this->polarAngle = polarAngle;
        this->azimuthAngle = azimuthAngle;
        angles = {&polarAngle, &azimuthAngle};
    }

    /** Other Methods */
    /* Utility */
    /**
     * Return the unit vector pointing in the direction defined by this object.
     */
    spatialVector getVectorDirection() override {
        // By definition, when an angle is plotted on the unit circle, the
        // point where the angle's ray intersects the circle is (cos(x), sin
        // (x)). This method takes those coordinates and scales them down
        // towards the circle's center by the sin of the azimuth (up/down)
        // angle to emulate the same point rotating up/down along the unit
        // sphere. Because the azimuthAngle is measured from the z-axis to
        // the point, the sin of the angle represents the non-vertical
        // component of that angle, which is parallel to the xy plane. If you
        // put these equations in Desmos:
        //      a = 1 (range 0-2pi)
        //      b = 1 (range 0-pi)
        //      (cos(a)sin(b), sin(a)sin(b))
        //      x^2+y^2=1
        // it will appear to be a top-down view of a point on the unit
        // sphere with the sphere rotating up/down (b) and around (a).
        return spatialVector(std::vector<double>({
            cos(polarAngle) * sin(azimuthAngle),
            sin(polarAngle) * sin(azimuthAngle),
            cos(azimuthAngle)
        }));
    }
    
    /* Rotation */
    /**
     * Increase rotation of polarAngle by dPolarAngle degrees, staying within 
     * [0, 360).
     */
    void rotatePolar(double dPolarAngle){
        // TODO get rid of this if angles vector is updated automatically
        std::cout << "test: utils.cpp: sphericalAngle3d: rotatePolar:"
                << std::endl;
        std::cout << "angles vector before increment of " << dPolarAngle
                << " degrees:" << std::endl;
        for (auto & angle : angles){
            std::cout << angle << " | ";
        }
        std::cout << std::endl;
        polarAngle += dPolarAngle;

        // Cap between [0, 360)
        // %= doesn't work with doubles, implement manually
        polarAngle -= floor(polarAngle / 360.0) * 360;

        std::cout << "angles vector after increment of " << dPolarAngle
                << " degrees:" << std::endl;
        for (auto & angle : angles){
            std::cout << angle << " | ";
        }
        std::cout << std::endl;
    }
    /**
     * Increase rotation of azimuthAngle by dAzimuthAngle degrees, staying 
     * within [0, 180].
     */
    void rotateAzimuth(double dAzimuthAngle){
        azimuthAngle += dAzimuthAngle;

        // Cap between [0, 180]
        if (azimuthAngle > 180){
            azimuthAngle = 180;
        } else if (azimuthAngle < 0){
            azimuthAngle = 0;
        }
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
    /// Note: Read comments in sphericalAngle3d to make more sense of below

    // polarAngle ranges from [0, 180] degrees where 0 is "north"
    //      and increasing the angle rotates east (90 is east, etc.)
    // azimuthAngle ranges from [0, 180] degrees where 0 is
    //      straight up and 180 is straight down (90 is forward)
    // phiAngle ranges from [0, 360) degrees, where 0 lies as it would in our 3
    //      dimensions, 90 lies orthogonally "outward" to the 3d dimension,
    //      180 lies opposite to 0 would in our 3 dimensions, and 270 lies
    //      orthogonally "inward" to the 3rd dimension (to give arbitrary
    //      names to 4d directions)
    double polarAngle, azimuthAngle, phiAngle;

    /** Constructors */
    sphericalAngle4d() : sphericalAngle4d(0, 90, 90){
    }
    sphericalAngle4d(double polarAngle, double azimuthAngle, double phiAngle){
        this->polarAngle = polarAngle;
        this->azimuthAngle = azimuthAngle;
        this->phiAngle = phiAngle;
        angles = {&polarAngle, &azimuthAngle, &phiAngle};
    }
    
    /** Other Methods */
    /* Utility */
    /**
     * Return the coordinate of the point on the unit sphere
     * in the direction of the spherical angles.
     */
    spatialVector getVectorDirection() override {
        // It would seem at first that only the top hemisphere of the unit
        // 3-sphere can be reached with polarAngle and azimuthAngle both only
        // ranging from [0-180], but every point on this hemisphere can be
        // mapped to the antipodal (opposite point) with a rotation through
        // the 4th dimension by 180 degrees. For example, to reach (0, 0, -1,
        // 0), these values work with the math that has been implemented below:
        //      polarAngle = 0 deg = 0 rad (or any value)
        //      azimuthAngle = 90 deg = pi/2 rad
        //      phiAngle = 180 deg = pi rad
        // because
        //      x = cos(phiAngle)cos(azimuthAngle)cos(polarAngle) = 0
        //      y = cos(phiAngle)cos(azimuthAngle)sin(polarAngle) = 0
        //      z = cos(phiAngle)sin(azimuthAngle) = -1
        //      a = sin(phiAngle) = 0
        return spatialVector(std::vector<double>({
            cos(phiAngle) * cos(azimuthAngle) * cos(polarAngle),
            cos(phiAngle) * cos(azimuthAngle) * sin(polarAngle),
            cos(phiAngle) * sin(azimuthAngle),
            sin(phiAngle)
        }));
    }
    
    /* Rotation */
    /**
     * Increase rotation of polarAngle by dPolarAngle degrees, staying within 
     * [0, 180].
     */
    void rotatePolar(double dPolarAngle){
        polarAngle += dPolarAngle;

        // Cap between [0, 180]
        if (polarAngle > 180){
            polarAngle = 180;
        } else if (polarAngle < 0){
            polarAngle = 0;
        }
    }
    /**
     * Increase rotation of azimuthAngle by dAzimuthAngle degrees, staying 
     * within [0, 180].
     */
    void rotateAzimuth(double dAzimuthAngle){
        azimuthAngle += dAzimuthAngle;

        // Cap between [0, 180]
        if (azimuthAngle > 180){
            azimuthAngle = 180;
        } else if (azimuthAngle < 0){
            azimuthAngle = 0;
        }
    }
    /**
     * Increase rotation of phiAngle by dPhiAngle degrees, staying within 
     * [0, 180].
     */
    void rotatePhi(double dPhiAngle){
        phiAngle += dPhiAngle;

        // Cap between [0, 360)
        // %= doesn't work with doubles, implement manually
        phiAngle -= floor(phiAngle / 360.0) * 360;
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
