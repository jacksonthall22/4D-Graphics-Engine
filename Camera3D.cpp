//
// Created by Jackson Hall on 4/29/2020.
//

#include "Camera3D.h"


/** ========== Constructors ========== */
Camera3D::Camera3D() : Camera3D(
        point3d(),
        spatialVector(std::vector<double>({0, 1, 0})),
        sphericalAngle3d(0, 90),
        point3d(),
        Camera::getFocalDistanceFromFOV(Camera3D::DEFAULT_FOV)){
}
Camera3D::Camera3D(const Camera3D &other) : Camera3D(
        point3d(
            other.location.x,
            other.location.y,
            other.location.z
        ),
        other.normal,
        sphericalAngle3d(
            other.sphericalDirection.polarAngle,
            other.sphericalDirection.azimuthAngle
        ),
        point3d(
            other.focus.x,
            other.focus.y,
            other.focus.z
        ),
        other.focalDistance){
}
Camera3D::Camera3D(
        const point3d& location,
        const spatialVector& normal,
        const sphericalAngle3d& sphericalDirection,
        const point3d& focus,
        const double& focalDistance) : Camera(focalDistance) {
    this->location = location;
    this->normal = normal;
    this->sphericalDirection = sphericalDirection;
    this->focus = focus;
    setFocus();
}

/** ========== Getters ========== */
/**
 * Return the unit vector that points upward relative to the camera normal.
 * In other words, return the vector that is a -90 deg azimuth rotation from
 * the normal (even past zenith/nadir).
 */
spatialVector Camera3D::getUnitUpVector() const {
    return sphericalAngle3d(
            sphericalDirection.polarAngle,
            sphericalDirection.azimuthAngle - 90).getUnitVector();
}
/**
 * Return the unit vector that points rightward relative to the camera normal.
 * In other words, return the vector that is a -90 deg polar rotation from
 * the normal.
 */
spatialVector Camera3D::getUnitRightVector() const {
    sphericalAngle3d temp(sphericalDirection);
    temp.rotatePolar(-90);
    return temp.getUnitVector();

    /// TODO Try replacing with below if this doesn't seem to work
//    return sphericalAngle3d(
//            mod(*sphericalDirection.polarAngle - 90, 360),
//            90).getUnitVector();
}

spatialVector const& Camera3D::getNormal() const {
    return normal;
}
sphericalAngle3d const& Camera3D::getSphericalDirection() const {
    return sphericalDirection;
}
point3d const& Camera3D::getFocus() const {
    return focus;
}

/** ========== Setters ========== */
/* Utility */
/**
 * Sets the focus to a point3d that is this.focalDistance units away from
 * this.location in the OPPOSITE direction of this.normal.
 */
void Camera3D::setFocus() {
    // Normalizes this.normal in case it is not already (though it normally
    // will be), scales it by -this.focalDistance, and displaces it by
    // this.location
    double normalMagnitude = normal.magnitude();

    double newX = location.x - normal.components[0] * focalDistance
            / normalMagnitude;
    double newY = location.y - normal.components[1] * focalDistance
            / normalMagnitude;
    double newZ = location.z - normal.components[2] * focalDistance
            / normalMagnitude;

    focus = point3d(newX, newY, newZ);
}

/* Movement */
void Camera3D::setLocation(std::vector<double> newLocation) {
    if (newLocation.size() == 3){
        setLocation(newLocation[0], newLocation[1], newLocation[2]);
    } else {
        // Bad input
        std::cout << "Warning: Invalid input in:\n\tvoid "
                     "Camera3D::setLocation(std::vector<double> newLocation)"
                     "\n\t(Camera3D.cpp)" << std::endl;
    }
}

void Camera3D::setLocation(const point3d& newLocation) {
    this->location = point3d(newLocation);
}
void Camera3D::setLocation(double x, double y, double z) {
    this->location = point3d(x, y, z);
}

/* Rotation */
/**
 * Sets this.normal to the unit vector pointing in the direction of
 * this.sphericalDirection.
 */
void Camera3D::setNormal() {
    this->normal = sphericalDirection.getUnitVector();


    // Reset focus
    setFocus();
}
void Camera3D::setSphericalDirection(std::vector<double> newAngles) {
    if (newAngles.size() == 2){
       setSphericalDirection(newAngles[0], newAngles[1]);
    } else {
        // Bad input
        std::cout << "Warning: Invalid input in:\n\tvoid "
                     "Camera3D::setSphericalDirection(std::vector<double> "
                     "newAngles)\n\t(Camera3d.cpp)" << std::endl;
    }
}

void Camera3D::setSphericalDirection(const sphericalAngle3d& newAngles) {
    setSphericalDirection(newAngles.polarAngle,newAngles.azimuthAngle);
}
void Camera3D::setSphericalDirection(double polarAngle, double azimuthAngle) {
    setPolar(polarAngle);
    setAzimuth(azimuthAngle);
}
void Camera3D::setPolar(double polarAngle) {
    sphericalDirection.setPolar(polarAngle);

    // Reset focus
    setFocus();
}
void Camera3D::setAzimuth(double azimuthAngle) {
    sphericalDirection.setAzimuth(azimuthAngle);

    // Reset focus
    setFocus();
}

/** ========== Other Methods ========== */
/* Utility */
optional<point2d> Camera3D::projectPoint(const point3d& p) const {
    // Plane of Camera orthogonal to normal vector n = (A, B, C):
    //      Ax + By + Cz = D
    //
    // Parametric equations of line r parallel to vector v = <xp, yp, zp>
    // and through point p = (x0, y0, z0):
    //      r = p + vt
    //       ||
    //       \/
    //      x = x0 + xp * t
    //      y = y0 + yp * t
    //      z = z0 + zp * t
    //
    // The following solves for t by plugging x, y, and z into the plane
    // equation above. Then t can be plugged back into the parametric
    // equations above to get an intersection point.
    //
    // First need to get v by calculating difference between coords of focus
    // and the point3d
    std::vector<double> test = {
        p.x - focus.x,
        p.y - focus.y,
        p.z - focus.z
    };
    spatialVector v(test);

    // If this vector has a scalar projection of <= 0 onto the normal, it is
    // behind the camera. Make sure this is not the case trying to solve for
    // an intersection point that doesn't exist (or one that would reflect
    // backwards through the focus onto the plane)
    if (v.scalarProjectOnto(normal) <= 0){
        return nullopt;
    }

    // Splitting up t calculation to keep it from getting huge
    double tNumerator, tDenominator, t;
    tNumerator = normal.components[0] * (p.x - location.x)
            + normal.components[1] * (p.y - location.y)
            + normal.components[2] * (p.z - location.z);
    tDenominator = normal.components[0] * (v.components[0])
            + normal.components[1] * (v.components[1])
            + normal.components[2] * (v.components[2]);

    // Make sure there's no division by 0 error
    if (tDenominator != 0){
        // Safe to do the calculation
        t = tNumerator / tDenominator;

        // Now that t is known plug it back into parametric equations above to
        // get 3d intersection point
        point3d intersectionPoint(
            location.x + (p.x - focus.x),
            location.y + (p.y - focus.y),
            location.z + (p.z - focus.z)
        );

        // Must find coords (x', y') that represent a point on the rotated plane
        // relative to the camera's location (i.e. the pixel offset on the
        // screen where the vertex should actually appear relative to screen
        // center (0,0))
        //
        // First create vector pointing from camera location to
        // intersectionPoint (which lies on the camera's plane)
        spatialVector cameraToIntersection(std::vector<double>({
            intersectionPoint.x - location.x,
            intersectionPoint.y - location.y,
            intersectionPoint.z - location.z
        }));

        // Now x' and y' are just the scalar projections of cameraToIntersection
        // onto the vector pointing straight "up" and the vector pointing
        // straight right. These give 2d coordinates on the screen where the
        // point should actually be drawn.
        return point2d(cameraToIntersection.scalarProjectOnto(getUnitUpVector()),
                cameraToIntersection.scalarProjectOnto(getUnitRightVector()));

    } else {
        // Would divide by 0
        return nullopt;
    }
}

/* Movement */
void Camera3D::move(std::vector<double> dPosition) {
    if (dPosition.size() == 3){
        location.move(dPosition);
    } else {
        // Bad input
        std::cout << "Warning: Invalid input in:\n\tvoid Camera3D::move"
                     "(std::vector<double> dPosition)\n\t(Camera3D.cpp)"
                     << std::endl;
    }

    // Reset focus
    setFocus();
}
void Camera3D::move(spatialVector dPosition) {
    if (dPosition.components.size() == 3){
        location.move(dPosition);
    } else {
        std::cout << "Warning: Invalid input in:\n\tvoid Camera3D::move"
                     "(spatialVector dPosition)\n\t(Camera3D.cpp)" << std::endl;
    }

    // Reset focus
    setFocus();
}

void Camera3D::move(double x, double y, double z) {
    location.move(std::vector<double>({x, y, z}));

    // Reset focus
    setFocus();
}
void Camera3D::moveX(double dx) {
    location.moveX(dx);

    // Reset focus
    setFocus();
}
void Camera3D::moveY(double dy) {
    location.moveY(dy);

    // Reset focus
    setFocus();
}
void Camera3D::moveZ(double dz) {
    location.moveZ(dz);

    // Reset focus
    setFocus();
}
/**
 * Move to the camera's left (orthogonal to normal), leaving z fixed.
 */
void Camera3D::left() {
    spatialVector dPositionVec = getUnitRightVector();
    dPositionVec.scale(-DEFAULT_MOVE_DISTANCE);
    move(dPositionVec);

    // Reset focus
    setFocus();
}
void Camera3D::right() {
    spatialVector dPositionVec = getUnitRightVector();
    dPositionVec.scale(DEFAULT_MOVE_DISTANCE);
    move(dPositionVec);

    // Reset focus
    setFocus();
}
void Camera3D::up() {
    move(std::vector<double>({
        0,
        0,
        DEFAULT_MOVE_DISTANCE
    }));

    // Reset focus
    setFocus();
}
void Camera3D::down() {
    move(std::vector<double>({
        0,
        0,
        DEFAULT_MOVE_DISTANCE
    }));

    // Reset focus
    setFocus();
}
/**
 * Move the camera forward (in direction of polarAngle) in x/z directions only
 * (locking y coordinate).
 */
void Camera3D::forward() {
    // Get vector in direction of current polarAngle
    spatialVector dPositionVec = sphericalAngle3d(
        sphericalDirection.polarAngle,
        90
    ).getUnitVector();
    dPositionVec.scale(DEFAULT_MOVE_DISTANCE);
    move(dPositionVec);

    // Reset focus
    setFocus();
}
/**
 * Move the camera backward (in direction of polarAngle) in x/z directions only
 * (locking z coordinate).
 */
void Camera3D::back() {
    // Get vector in direction of current polarAngle
    spatialVector dPositionVec = sphericalAngle3d(
        sphericalDirection.polarAngle,
        90
    ).getUnitVector();
    dPositionVec.scale(-DEFAULT_MOVE_DISTANCE);
    move(dPositionVec.components);

    // Reset focus
    setFocus();
}

/* Rotation */
void Camera3D::rotate(const std::vector<double> dAngles) {
    if (dAngles.size() == 2){
        rotate(dAngles[0], dAngles[1]);
    } else {
        std::cout << "Warning: Invalid input in:\n\tvoid Camera3D::rotate"
                     "(std::vector<double> dAngles)\n\t(Camera3D.cpp)"
                     << std::endl;
    }
}

void Camera3D::rotate(const sphericalAngle3d& dAngles) {
    rotate(dAngles.polarAngle, dAngles.azimuthAngle);
}
void Camera3D::rotate(const double dPolarAngle, const double dAzimuthAngle) {
    rotatePolar(dPolarAngle);
    rotateAzimuth(dAzimuthAngle);
}
void Camera3D::rotatePolar(const double dPolarAngle) {
    sphericalDirection.rotatePolar(dPolarAngle);

    // Reset focus
    setFocus();
}
void Camera3D::rotateAzimuth(const double dAzimuthAngle) {
    sphericalDirection.rotatePolar(dAzimuthAngle);

    // Reset focus
    setFocus();
}
void Camera3D::rotateLeft() {
    rotatePolar(DEFAULT_ROTATION_ANGLE);

    // Reset focus
    setFocus();
}
void Camera3D::rotateRight() {
    rotatePolar(-DEFAULT_ROTATION_ANGLE);

    // Reset focus
    setFocus();
}
void Camera3D::rotateUp() {
    rotateAzimuth(-DEFAULT_ROTATION_ANGLE);

    // Reset focus
    setFocus();
}
void Camera3D::rotateDown() {
    rotateAzimuth(DEFAULT_ROTATION_ANGLE);

    // Reset focus
    setFocus();
}
