//
// Created by Jackson Hall on 4/29/2020.
//

#include "Camera4D.h"


/** ========== Constructors ========== */
Camera4D::Camera4D() : Camera4D(
        point4d(),
        spatialVector(std::vector<double>({0, 0, 1, 0})),
        sphericalAngle4d(0, 0, 90),
        point4d(),
        Camera::getFocalDistanceFromFOV(Camera::DEFAULT_FOV)){
}
Camera4D::Camera4D(const Camera4D &other) : Camera4D(
        point4d(other.location),
        spatialVector(other.normal),
        sphericalAngle4d(other.sphericalDirection),
        point4d(other.focus),
        other.focalDistance){
}
Camera4D::Camera4D(
        const point4d& location,
        const spatialVector& normal,
        const sphericalAngle4d& sphericalDirection,
        const point4d& focus,
        const double& focalDistance) : Camera(focalDistance) {
    this->location = location;
    this->normal = normal;
    this->sphericalDirection = sphericalDirection;
    this->focus = focus;
    setNormal();
}

/** ========== Getters ========== */
/**
 * Return the unit vector that points "upward" relative to the camera normal.
 * In other words, return the vector that is a -90 deg azimuth rotation from
 * the normal (even past zenith/nadir).
 */
spatialVector Camera4D::getUnitUpVector() const {
    return sphericalAngle4d(
        sphericalDirection.polarAngle,
        sphericalDirection.azimuthAngle - 90,
        sphericalDirection.phiAngle
    ).getUnitVector();
}
/**
 * Return the unit vector that points "rightward" relative to the camera normal.
 * In other words, return the vector that is a -90 deg polar rotation from
 * the normal.
 */
spatialVector Camera4D::getUnitRightVector() const {
    sphericalAngle4d temp(sphericalDirection);
    temp.rotatePolar(-90);
    return temp.getUnitVector();
}
/**
 * Return the unit vector that points "outward" relative to the camera normal.
 * In other words, return the vector that is a -90 deg phi rotation from
 * the normal.
 */
spatialVector Camera4D::getUnitOutVector() const {
    sphericalAngle4d temp(sphericalDirection);
    temp.rotatePhi(-90);
    return temp.getUnitVector();
}

spatialVector const &Camera4D::getNormal() const {
    return normal;
}

sphericalAngle4d const &Camera4D::getSphericalDirection() const {
    return sphericalDirection;
}

point4d const &Camera4D::getFocus() const {
    return focus;
}

/** ========== Setters ========== */
/* Utility */
/**
 * Sets the focus to a point4d that is this.focalDistance units away from
 * this.location in the OPPOSITE direction of this.normal.
 */
void Camera4D::setFocus() {
    // Normalizes this.normal in case it is not already (though it normally
    // will be), scales it by -this.focalDistance, and displaces it by
    // this.location
    double normalMagnitude = normal.magnitude();

    double newX = location.x - (focalDistance * normal.components[0]
            / normalMagnitude);
    double newY = location.y - (focalDistance * normal.components[1]
            / normalMagnitude);
    double newZ = location.z - (focalDistance * normal.components[2]
            / normalMagnitude);
    double newA = location.a - (focalDistance * normal.components[3]
            / normalMagnitude);

    focus = point4d(newX, newY, newZ, newA);
}

/* Movement */
void Camera4D::setLocation(std::vector<double> newLocation) {
    if (newLocation.size() == 4){
        setLocation(
            newLocation[0],
            newLocation[1],
            newLocation[2],
            newLocation[3]
        );
    } else {
        // Bad input
        std::cout << "Warning: Invalid input in:\n\tvoid "
                     "Camera4D::setLocation(std::vector<double> newLocation)"
                     "\n\t(Camera4D.cpp)" << std::endl;
    }
}
void Camera4D::setLocation(const point4d &newLocation) {
    location = point4d(newLocation);
}
void Camera4D::setLocation(double x, double y, double z, double a) {
    this->location = point4d(x, y, z, a);
}

/* Rotation */
/**
 * Sets this.normal to the unit vector pointing in the direction of
 * this.sphericalDirection.
 */
void Camera4D::setNormal() {
    this->normal = sphericalDirection.getUnitVector();

    // Reset focus
    setFocus();
}
void Camera4D::setSphericalDirection(std::vector<double> newAngles) {
    if (newAngles.size() == 3){
        setSphericalDirection(newAngles[0], newAngles[1], newAngles[2]);
    } else {
        // Bad input
        std::cout << "Warning: Invalid input in:\n\tvoid "
                     "Camera4D::setSphericalDirection(std::vector<double> "
                     "newAngles)\n\t(Camera4d.cpp)" << std::endl;
    }
}

void Camera4D::setSphericalDirection(const sphericalAngle4d &newAngles) {
    setSphericalDirection(
        newAngles.polarAngle,
        newAngles.azimuthAngle,
        newAngles.phiAngle);
}
void Camera4D::setSphericalDirection(
        const double polarAngle,
        const double azimuthAngle,
        const double phiAngle){
    setPolarAngle(polarAngle);
    setAzimuthAngle(azimuthAngle);
    setPhiAngle(phiAngle);
}
void Camera4D::setPolarAngle(double polarAngle) {
    sphericalDirection.setPolar(polarAngle);
}
void Camera4D::setAzimuthAngle(double azimuthAngle) {
    sphericalDirection.setAzimuth(azimuthAngle);
}
void Camera4D::setPhiAngle(double phiAngle) {
    sphericalDirection.setPhi(phiAngle);
}

/** ========== Other Methods ========== */
/* Utility */
optional<point3d> Camera4D::projectPoint(const point4d &p) const {
    // To understand what's happening here, take a look at projectPoint() in
    // Camera3d.cpp and try to imagine a 4d line intersecting the hyperplane of
    // our reality. At least the math is the same.
    //
    // Hyperplane of Camera orthogonal to normal vector n = (A, B, C, D):
    //      Ax + By + Cz + Da = E
    // Parametric equations of line r parallel to vector v = <xp, yp, zp, ap>
    // and through point p = (x0, y0, z0, a0):
    //      r = p + vt
    //       ||
    //       \/
    //      x = x0 + xp * t
    //      ...
    //      a = a0 + ap * t

    // First determine if point is in front of the camera. If the vector from
    // the Camera's location to the point has a scalar projection < 0 onto
    // the normal, it is behind the projection hyperplane - do not render,
    // return nullopt.
    spatialVector cameraToPoint(std::vector<double>({
        p.x - location.x,
        p.y - location.y,
        p.z - location.z,
        p.a - location.a
    }));
    if (cameraToPoint.scalarProjectOnto(normal) < 0){
        return nullopt;
    }

    // Next need to get vector from focus to point (v)
    spatialVector focusToPoint(std::vector<double>({
        p.x - focus.x,
        p.y - focus.y,
        p.z - focus.z,
        p.a - focus.a
    }));

    // Calculate t, split up num. and denom. to keep from getting huge
    double tDenominator, tNumerator, t;
    tDenominator = normal.components[0] * focusToPoint.components[0]
            + normal.components[1] * focusToPoint.components[1]
            + normal.components[2] * focusToPoint.components[2]
            + normal.components[3] * focusToPoint.components[3];

    // Make sure there's no division by 0 error
    if (tDenominator == 0){
        return nullopt;
    }

    // Safe to do the calculation
    tNumerator = normal.components[0] * cameraToPoint.components[0]
            + normal.components[1] * cameraToPoint.components[1]
            + normal.components[2] * cameraToPoint.components[2]
            + normal.components[3] * cameraToPoint.components[3];
    t = tNumerator / tDenominator;

    // Now that t is known plug it back into parametric equations above to
    // get 4d intersection point
    point4d intersectionPoint(
        location.x + focusToPoint.components[0] * t,
        location.y + focusToPoint.components[1] * t,
        location.z + focusToPoint.components[2] * t,
        location.a + focusToPoint.components[3] * t
    );

    // Now must find coords (x', y', z') that is a point on the rotated
    // hyperplane relative to the camera's location (i.e. where the vertex
    // should actually appear in 3d space)
    //
    // First create vector pointing from camera location to
    // intersectionPoint (which lies on the camera's hyperplane)
    spatialVector cameraToIntersection(std::vector<double>({
        intersectionPoint.x - location.x,
        intersectionPoint.y - location.y,
        intersectionPoint.z - location.z,
        intersectionPoint.a - location.a
    }));

    // Now x', y', and z' are just the scalar projections of
    // cameraToIntersection onto the vector pointing straight "up", the
    // vector pointing straight right, and the vector pointing straight
    // "out" from the perspective of the camera. These give 3d coordinates
    // in the Scene where the point should exist.
    return point3d(cameraToIntersection.scalarProjectOnto(getUnitRightVector()),
            cameraToIntersection.scalarProjectOnto(getUnitUpVector()),
            cameraToIntersection.scalarProjectOnto(getUnitOutVector()));
}

/* Movement */
void Camera4D::move(std::vector<double> dPosition) {
    if (dPosition.size() == 4){
        location.move(dPosition);
    } else {
        // Bad input
        std::cout << "Warning: Invalid input in:\n\tvoid Camera4D::move"
                     "(std::vector<double> dPosition)\n\t(Camera4D.cpp)"
                     << std::endl;
    }

    // Reset focus
    setFocus();
}
void Camera4D::move(spatialVector dPosition) {
    if (dPosition.components.size() == 4){
        location.move(dPosition);
    } else {
        std::cout << "Warning: Invalid input in:\n\tvoid Camera4D::move"
                     "(spatialVector dPosition)\n\t(Camera4D.cpp)" << std::endl;
    }

    // Reset focus
    setFocus();
}
void Camera4D::move(double dx, double dy, double dz, double da) {
    moveX(dx);
    moveY(dy);
    moveZ(dz);
    moveA(da);
}
void Camera4D::moveX(double dx) {
    location.moveX(dx);

    // Reset focus
    setFocus();
}

void Camera4D::moveY(double dy) {
    location.moveY(dy);

    // Reset focus
    setFocus();
}

void Camera4D::moveZ(double dz) {
    location.moveZ(dz);

    // Reset focus
    setFocus();
}

void Camera4D::moveA(double da) {
    location.moveA(da);

    // Reset focus
    setFocus();
}

void Camera4D::left() {
    spatialVector dPositionVec = getUnitRightVector();
    dPositionVec.scale(-DEFAULT_MOVE_DISTANCE);
    move(dPositionVec);

    // Reset focus
    setFocus();
}

void Camera4D::right() {
    spatialVector dPositionVec = getUnitRightVector();
    dPositionVec.scale(DEFAULT_MOVE_DISTANCE);
    move(dPositionVec);

    // Reset focus
    setFocus();
}
/**
 * Move the camera upward (in direction of y-axis) in y dimension only
 * (locking all others).
 */
void Camera4D::up() {
    move(std::vector<double>({
        0,
        0,
        DEFAULT_MOVE_DISTANCE,
        0
    }));

    // Reset focus
    setFocus();
}

void Camera4D::down() {
    move(std::vector<double>({
        0,
        0,
        -DEFAULT_MOVE_DISTANCE,
        0
    }));

    // Reset focus
    setFocus();
}

/**
 * Move the camera forward (in direction of polarAngle and azimuthAngle) in
 * x/z/a directions only (locking a).
 */
void Camera4D::forward() {
    // Get vector in direction of current polarAngle and phiAngle
    spatialVector dPositionVec = sphericalAngle4d(
        sphericalDirection.polarAngle,
        sphericalDirection.azimuthAngle,
        90
    ).getUnitVector();
    dPositionVec.scale(DEFAULT_MOVE_DISTANCE);
    move(dPositionVec);

    // Reset focus
    setFocus();
}
/**
 * Move the camera backward (in the direction of polarAngle and azimuthAngle) in
 * x/y/z directions only (locking a).
 */
void Camera4D::back() {
    spatialVector dPositionVec = sphericalAngle4d(
        sphericalDirection.polarAngle,
        sphericalDirection.azimuthAngle,
        90
    ).getUnitVector();
    dPositionVec.scale(-DEFAULT_MOVE_DISTANCE);
    move(dPositionVec);

    // Reset focus
    setFocus();
}

/**
 * Move the camera straight "inward" in the a direction.
 */
void Camera4D::in() {
    move(std::vector<double>({
        0,
        0,
        0,
        -DEFAULT_MOVE_DISTANCE
    }));

    // Reset focus
    setFocus();
}

/**
 * Move the camera straight "outward" in the a direction.
 */
void Camera4D::out() {
    move(std::vector<double>({
        0,
        0,
        0,
        DEFAULT_MOVE_DISTANCE
    }));

    // Reset focus
    setFocus();
}
void Camera4D::rotate(std::vector<double> dAngles) {
    if (dAngles.size() == 3){
        rotate(dAngles[0], dAngles[1], dAngles[2]);
    } else {
        std::cout << "Warning: Invalid input in:\n\tvoid Camera4D::rotate"
                     "(std::vector<double> dAngles)\n\t(Camera4D.cpp)"
                     << std::endl;
    }
}
void Camera4D::rotate(const sphericalAngle4d &dAngles) {
    rotatePolar(dAngles.polarAngle);
    rotateAzimuth(dAngles.azimuthAngle);
    rotatePhi(dAngles.phiAngle);
}
void Camera4D::rotate(
        double dPolarAngle,
        double dAzimuthAngle,
        double phiAngle) {
    rotatePolar(dPolarAngle);
    rotateAzimuth(dAzimuthAngle);
    rotatePhi(phiAngle);
}
void Camera4D::rotatePolar(double dPolarAngle) {
    sphericalDirection.rotatePolar(dPolarAngle);

    // Reset normal and focus
    setNormal();
}
void Camera4D::rotateAzimuth(double dAzimuthAngle) {
    sphericalDirection.rotateAzimuth(dAzimuthAngle);

    // Reset normal and focus
    setNormal();
}
void Camera4D::rotatePhi(double dPhiAngle) {
    sphericalDirection.rotatePhi(dPhiAngle);

    // Reset normal and focus
    setNormal();
}
void Camera4D::rotateLeft() {
    rotatePolar(DEFAULT_ROTATION_ANGLE);
}
void Camera4D::rotateRight() {
    rotatePolar(-DEFAULT_ROTATION_ANGLE);
}
void Camera4D::rotateUp() {
    rotateAzimuth(-DEFAULT_ROTATION_ANGLE);
}
void Camera4D::rotateDown() {
    rotateAzimuth(DEFAULT_ROTATION_ANGLE);
}
void Camera4D::rotateIn() {
    rotatePhi(DEFAULT_ROTATION_ANGLE);
}
void Camera4D::rotateOut() {
    rotatePhi(-DEFAULT_ROTATION_ANGLE);
}
