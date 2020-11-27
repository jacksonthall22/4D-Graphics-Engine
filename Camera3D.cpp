//
// Created by Jackson Hall on 4/29/2020.
//

#include "Camera3D.h"
#include "Scene.h"


/** ========== Constructors ========== */
Camera3D::Camera3D() : Camera3D(
        point3d(),
        spatialVector(std::vector<double>({0, 1, 0})),
        sphericalAngle3d(0, 90),
        point3d(),
        Camera::getFocalDistanceFromFOV(Camera3D::DEFAULT_FOV)){
}
Camera3D::Camera3D(const Camera3D &other) : Camera3D(
        point3d(other.location),
        spatialVector(other.normal),
        sphericalAngle3d(other.sphericalDirection),
        point3d(other.focus),
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
    setNormal();
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
    sphericalAngle3d temp(sphericalDirection.polarAngle, 90);
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

    double newFocusX = location.x - (focalDistance * normal.components[0]
            / normalMagnitude);
    double newFocusY = location.y - (focalDistance * normal.components[1]
            / normalMagnitude);
    double newFocusZ = location.z - (focalDistance * normal.components[2]
            / normalMagnitude);

    focus = point3d(newFocusX, newFocusY, newFocusZ);
}

/* Movement */
void Camera3D::setLocation(std::vector<double> newLocation) {
    if (newLocation.size() == 3){
        setLocation(newLocation[0], newLocation[1], newLocation[2]);
    } else {
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
 * this.sphericalDirection and calls setFocus().
 */
void Camera3D::setNormal() {
    this->normal = sphericalDirection.getUnitVector();
    setFocus();
}
void Camera3D::setSphericalDirection(std::vector<double> newAngles) {
    if (newAngles.size() == 2){
       setSphericalDirection(newAngles[0], newAngles[1]);
    } else {
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

    // Reset normal and focus
    setNormal();
}
void Camera3D::setAzimuth(double azimuthAngle) {
    sphericalDirection.setAzimuth(azimuthAngle);
    setNormal();
}

/** ========== Other Methods ========== */
/* Utility */
optional<point2d> Camera3D::projectPoint(const point3d& p) const {
    // Plane of Camera orthogonal to normal vector n = <A, B, C>:
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
    // First determine if point is in front of the Camera. If the vector from
    // the Camera's location to the point has a scalar projection < 0 onto
    // the normal, it is behind the projection plane - do not render, return
    // nullopt.
    /// TODO Return inf for x if
    spatialVector cameraToPoint(std::vector<double>({
        p.x - location.x,
        p.y - location.y,
        p.z - location.z
    }));
//    if (cameraToPoint.scalarProjectOnto(normal) < 0){
//        return nullopt;
//    }

    spatialVector focusToPoint(std::vector<double>({
        p.x - focus.x,
        p.y - focus.y,
        p.z - focus.z
    }));

    // Calculate t, split up num. and denom. to keep it from getting huge
    double tDenominator, tNumerator, t;
    tDenominator = normal.components[0] * focusToPoint.components[0]
            + normal.components[1] * focusToPoint.components[1]
            + normal.components[2] * focusToPoint.components[2];

    // Make sure there's no division by 0 error; also cut off tiny values
    if (tDenominator == 0){
        return nullopt;
    }

    // Safe to do the calculation
    tNumerator = -normal.components[0] * cameraToPoint.components[0]
            - normal.components[1] * cameraToPoint.components[1]
            - normal.components[2] * cameraToPoint.components[2];
    t = tNumerator / tDenominator;

    // Now that t is known plug it back into parametric equations above to
    // get 3d intersection point
    point3d intersectionPoint(
        p.x + focusToPoint.components[0] * t,
        p.y + focusToPoint.components[1] * t,
        p.z + focusToPoint.components[2] * t
    );

    // Must find coords (x', y') that represent a point on the rotated plane
    // relative to the camera's location (i.e. the pixel offset on the
    // screen where the vertex should actually appear relative to screen
    // center, (0,0))
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
    return point2d(cameraToIntersection.scalarProjectOnto(getUnitRightVector()),
            cameraToIntersection.scalarProjectOnto(getUnitUpVector()));

}

/* Movement */
void Camera3D::move(std::vector<double> dPosition) {
    if (dPosition.size() == 3){
        location.move(dPosition);
    } else {
        std::cout << "Warning: Invalid input in:\n\tvoid Camera3D::move"
                     "(std::vector<double> dPosition)\n\t(Camera3D.cpp)"
                     << std::endl;
    }
    setFocus();
}
void Camera3D::move(const spatialVector& dPosition) {
    if (dPosition.components.size() == 3){
        location.move(dPosition);
        setFocus();
    } else {
        std::cout << "Warning: Invalid input in:\n\tvoid Camera3D::move"
                     "(const spatialVector& dPosition)\n\t(Camera3D.cpp)" <<
                     std::endl;
    }
}
void Camera3D::move(double dX, double dY, double dZ) {
    location.move(dX, dY, dZ);
    setFocus();
}
void Camera3D::moveX(double dX) {
    location.moveX(dX);
    setFocus();
}
void Camera3D::moveY(double dY) {
    location.moveY(dY);
    setFocus();
}
void Camera3D::moveZ(double dZ) {
    location.moveZ(dZ);
    setFocus();
}
void Camera3D::moveLeftCustom(double dL) {
    spatialVector dPositionVec = getUnitRightVector();
    dPositionVec.scale(-dL);
    move(dPositionVec);
}
void Camera3D::moveRightCustom(double dR){
    spatialVector dPositionVec = getUnitRightVector();
    dPositionVec.scale(dR);
    move(dPositionVec);
}
void Camera3D::moveUpCustom(double dU){
    moveZ(dU);
}
void Camera3D::moveDownCustom(double dD){
    moveZ(-dD);
}
void Camera3D::moveForwardCustom(double dF){
    // Get vector in direction of current polarAngle (locks movement
    // to horizontal x/y plane only so moving forward while looking
    // up/down cannot change z)
    spatialVector dPositionVec = sphericalAngle3d(
        sphericalDirection.polarAngle,
        90
    ).getUnitVector();
    dPositionVec.scale(dF);
    move(dPositionVec);
}
void Camera3D::moveBackCustom(double dB){
    // Get vector in direction of current polarAngle (locks movement
    // to horizontal x/y plane only so moving forward while looking
    // up/down cannot change z)
    spatialVector dPositionVec = sphericalAngle3d(
        sphericalDirection.polarAngle,
        90
    ).getUnitVector();
    dPositionVec.scale(-dB);
    move(dPositionVec);
}
void Camera3D::moveLeftDefault() {
    moveLeftCustom(DEFAULT_MOVE_DISTANCE);
}
void Camera3D::moveRightDefault(){
    moveRightCustom(DEFAULT_MOVE_DISTANCE);
}
void Camera3D::moveUpDefault() {
    moveUpCustom(DEFAULT_MOVE_DISTANCE);
}
void Camera3D::moveDownDefault(){
    moveDownCustom(DEFAULT_MOVE_DISTANCE);
}
void Camera3D::moveForwardDefault(){
    moveForwardCustom(DEFAULT_MOVE_DISTANCE);
}
void Camera3D::moveBackDefault(){
    moveBackCustom(DEFAULT_MOVE_DISTANCE);
}
void Camera3D::moveVelocity(){
    move(velocityVec);
}
void Camera3D::moveForwardVelocity(){
    moveForwardCustom(velocityVec.components[0]);
}
void Camera3D::moveRightVelocity(){
    moveRightCustom(velocityVec.components[1]);
}
void Camera3D::moveUpVelocity(){
    moveUpCustom(velocityVec.components[2]);
}
void Camera3D::setMovementStatuses(const int movingForward_,
                                   const int movingStrafe_,
                                   const int movingUp_) {
    if (movingForward_ >= -1 and movingForward_ <= 1){
        movingForward = movingForward_;
    } else {
        std::cout << "Warning: Invalid input (movingForward_) in:"
                     "\n\tvoid Camera3D::setMovementStatuses("
                     "const int movingForward_,"
                     "const int movingStrafe_, "
                     "const int movingUp_)"
                     "\n\t(Camera3D.cpp)" << std::endl;
    }

    if (movingStrafe_ >= -1 and movingStrafe_ <= 1){
        movingStrafe = movingStrafe_;
    } else {
        std::cout << "Warning: Invalid input (movingStrafe_) in:"
                     "\n\tvoid Camera3D::setMovementStatuses("
                     "const int movingForward_,"
                     "const int movingStrafe_, "
                     "const int movingUp_)"
                     "\n\t(Camera3D.cpp)" << std::endl;
    }

    if (movingUp_ >= -1 and movingUp_ <= 1){
        movingUp = movingUp_;
    } else {
        std::cout << "Warning: Invalid input (movingUp_) in:"
                     "\n\tvoid Camera3D::setMovementStatuses("
                     "const int movingForward_,"
                     "const int movingStrafe_, "
                     "const int movingUp_)"
                     "\n\t(Camera3D.cpp)" << std::endl;
    }
}
void Camera3D::updateVelocities() {
    // Update values in velocityVec by one game tick.

    /// TODO Does this method work?

    /// First account for drag in any horizontal directions with current
    /// nonzero velocity and cap at MAX_VELOCITY (vertical motion has very
    /// little drag, should appear to move almost linearly)

    // Forward drag
    if (velocityVec.components[0] != 0) {
        if (velocityVec.components[0] > 0){
            // Drag down to 0
            velocityVec.components[0] = std::max(
                    velocityVec.components[0] - FORWARD_DRAG
                        * Scene::SECONDS_PER_TICK,
                    0.
            );
        } else {
            // Drag up to 0
            velocityVec.components[0] = std::min(
                    velocityVec.components[0] + FORWARD_DRAG
                        * Scene::SECONDS_PER_TICK,
                    0.
            );
        }
    }

    // Strafe drag
    if (velocityVec.components[1] != 0) {
        if (velocityVec.components[1] > 0){
            // Drag down to 0
            velocityVec.components[1] = std::max(
                    velocityVec.components[1] - STRAFE_DRAG
                        * Scene::SECONDS_PER_TICK,
                    0.
            );
        } else {
            // Drag up to 0
            velocityVec.components[1] = std::min(
                    velocityVec.components[1] + STRAFE_DRAG
                        * Scene::SECONDS_PER_TICK,
                    0.
            );
        }
    }

    // Up drag
    if (velocityVec.components[2] != 0){
        if (velocityVec.components[2] > 0){
            // Drag down to 0
            velocityVec.components[2] = std::max(
                    velocityVec.components[2] - UP_DRAG
                        * Scene::SECONDS_PER_TICK,
                    0.
            );
        } else {
            // Drag up to 0
            velocityVec.components[2] = std::min(
                    velocityVec.components[2] + UP_DRAG
                        * Scene::SECONDS_PER_TICK,
                    0.
            );
        }
    }

    /// Now account for current movement statuses

    // Forward movement
    if (movingForward != 0){
        if (movingForward > 0){
            /// Positive acceleration
            if (velocityVec.components[0] < 0){
                // If velocity is negative, first break to 0
                velocityVec.components[0] = std::min(
                        velocityVec.components[0] + FORWARD_BRAKE
                            * Scene::SECONDS_PER_TICK,
                        0.
                );
            } else {
                // If velocity is 0 or positive already,
                // accelerate to MAX_VELOCITY
                velocityVec.components[0] = std::max(
                        velocityVec.components[0] + FORWARD_ACCELERATION
                            * Scene::SECONDS_PER_TICK,
                        MAX_FORWARD_VELOCITY
                );
            }
        } else {
            /// Negative acceleration
            if (velocityVec.components[0] > 0){
                // If velocity is positive, first break to 0
                velocityVec.components[0] = std::max(
                        velocityVec.components[0] - FORWARD_BRAKE
                            * Scene::SECONDS_PER_TICK,
                        0.
                );
            } else {
                // If velocity is 0 or negative already,
                // accelerate (negatively) to -MAX_VELOCITY
                velocityVec.components[0] = std::min(
                        velocityVec.components[0] - FORWARD_ACCELERATION
                            * Scene::SECONDS_PER_TICK,
                        -MAX_FORWARD_VELOCITY
                );
            }
        }
    }

    // Strafe movement
    if (movingStrafe != 0){
        if (movingStrafe > 0){
            /// Positive acceleration
            if (velocityVec.components[1] < 0){
                // If velocity is negative, first break to 0
                velocityVec.components[1] = std::min(
                        velocityVec.components[1] + STRAFE_BRAKE
                            * Scene::SECONDS_PER_TICK,
                        0.
                );
            } else {
                // If velocity is 0 or positive already,
                // accelerate to MAX_VELOCITY
                velocityVec.components[1] = std::max(
                        velocityVec.components[1] + STRAFE_ACCELERATION
                            * Scene::SECONDS_PER_TICK,
                        MAX_STRAFE_VELOCITY
                );
            }
        } else {
            /// Negative acceleration
            if (velocityVec.components[1] > 0){
                // If velocity is positive, first break to 0
                velocityVec.components[1] = std::max(
                        velocityVec.components[1] - STRAFE_BRAKE
                            * Scene::SECONDS_PER_TICK,
                        0.
                );
            } else {
                // If velocity is 0 or negative already,
                // accelerate (negatively) to -MAX_VELOCITY
                velocityVec.components[1] = std::min(
                        velocityVec.components[1] - STRAFE_ACCELERATION
                            * Scene::SECONDS_PER_TICK,
                        -MAX_STRAFE_VELOCITY
                );
            }
        }
    }

    // Up movement
    if (movingUp != 0){
        if (movingUp > 0){
            /// Positive acceleration
            if (velocityVec.components[2] < 0){
                // If velocity is negative, first break to 0
                velocityVec.components[2] = std::min(
                        velocityVec.components[2] + UP_BREAK
                            * Scene::SECONDS_PER_TICK,
                        0.
                );
            } else {
                // If velocity is 0 or positive already,
                // accelerate to MAX_VELOCITY
                velocityVec.components[2] = std::max(
                        velocityVec.components[2] + UP_ACCELERATION
                            * Scene::SECONDS_PER_TICK,
                        MAX_UP_VELOCITY
                );
            }
        } else {
            /// Negative acceleration
            if (velocityVec.components[2] > 0){
                // If velocity is positive, first break to 0
                velocityVec.components[2] = std::max(
                        velocityVec.components[2] - UP_BREAK
                            * Scene::SECONDS_PER_TICK,
                        0.
                );
            } else {
                // If velocity is 0 or negative already,
                // accelerate (negatively) to -MAX_VELOCITY
                velocityVec.components[2] = std::min(
                        velocityVec.components[2] - UP_ACCELERATION
                            * Scene::SECONDS_PER_TICK,
                        -MAX_UP_VELOCITY
                );
            }
        }
    }
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
    rotatePolar(dPolarAngle, false);
    rotateAzimuth(dAzimuthAngle, false);
}
void Camera3D::rotatePolar(const double dPolarAngle) {
    rotatePolar(dPolarAngle, true);
}
void Camera3D::rotatePolar(const double dPolarAngle, bool updateNormal) {
    sphericalDirection.rotatePolar(dPolarAngle);
    if (updateNormal){
        setNormal();
    }
}
void Camera3D::rotateAzimuth(const double dAzimuthAngle) {
    rotateAzimuth(dAzimuthAngle, true);
}
void Camera3D::rotateAzimuth(const double dAzimuthAngle, bool updateNormal) {
    sphericalDirection.rotateAzimuth(dAzimuthAngle);
    if (updateNormal){
        setNormal();
    }
}
void Camera3D::rotateLeft() {
    rotatePolar(DEFAULT_ROTATION_ANGLE);
}
void Camera3D::rotateRight() {
    rotatePolar(-DEFAULT_ROTATION_ANGLE);
}
void Camera3D::rotateUp() {
    rotateAzimuth(-DEFAULT_ROTATION_ANGLE);
}
void Camera3D::rotateDown() {
    rotateAzimuth(DEFAULT_ROTATION_ANGLE);
}
