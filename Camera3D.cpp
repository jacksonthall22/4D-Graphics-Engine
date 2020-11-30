//
// Created by Jackson Hall on 4/29/2020.
//

#include "Camera3D.h"
#include "Scene.h"


/** ---------- Static Const Vars ---------- */
const double Camera3D::DEFAULT_FOV_DEGREES = 160;
const double Camera3D::DEFAULT_PROJECTION_PLANE_WIDTH_BLOCKS = 10;

// Credit to:
// https://gaming.stackexchange.com/questions/327830/what-is-the-player-
// acceleration-in-minecraft-when-flying
// and
// https://github.com/ddevault/TrueCraft/wiki/Entity-Movement-And-Physics
const double Camera3D::FB_ACCEL = 7; // blocks/second^2
const double Camera3D::RL_ACCEL = 6;
const double Camera3D::UD_ACCEL = 10;
const double Camera3D::FB_DRAG = 3; // blocks/second^2
const double Camera3D::RL_DRAG = 3;
const double Camera3D::UD_DRAG = 5;
const double Camera3D::FB_BRAKE = 5; // blocks/second^2
const double Camera3D::RL_BRAKE = 5;
const double Camera3D::UD_BRAKE = 10;
const double Camera3D::FB_MAX_SPEED = 3; // blocks/second
const double Camera3D::RL_MAX_SPEED = 3;
const double Camera3D::UD_MAX_SPEED = 1;

/** ---------- Constructors ---------- */
Camera3D::Camera3D() : Camera3D(
        point3d(),
        spatialVector(std::vector<double>({0, 1, 0})),
        sphericalAngle3d(0, 90),
        point3d(),
        Camera::getFocalDistanceFromFOV(Camera3D::DEFAULT_FOV_DEGREES,
            Camera3D::DEFAULT_PROJECTION_PLANE_WIDTH_BLOCKS),
        Camera3D::DEFAULT_PROJECTION_PLANE_WIDTH_BLOCKS,
        Camera::MovementMode::Fly,
        0,
        0,
        0,
        spatialVector(3)){
}
Camera3D::Camera3D(const Camera3D &other) : Camera3D(
        point3d(other.location),
        spatialVector(other.normal),
        sphericalAngle3d(other.sphericalDirection),
        point3d(other.focus),
        other.focalDistance,
        other.projectionPlaneWidthBlocks,
        other.movementMode,
        other.acceleratingFB,
        other.acceleratingRL,
        other.acceleratingUD,
        other.velocityVec){
}
Camera3D::Camera3D(
        const point3d& location,
        const spatialVector& normal,
        const sphericalAngle3d& sphericalDirection,
        const point3d& focus,
        const double& focalDistance,
        double projectionPlaneWidthBlocks,
        Camera::MovementMode movementMode,
        int movingForward,
        int movingStrafe,
        int movingUp,
        spatialVector velocityVec) : Camera(focalDistance, movementMode){ // NOLINT(performance-unnecessary-value-param)
    this->location = location;
    this->normal = normal;
    this->sphericalDirection = sphericalDirection;
    this->focus = focus;
    this->movementMode = movementMode;
    this->projectionPlaneWidthBlocks = projectionPlaneWidthBlocks;
    this->acceleratingFB = movingForward;
    this->acceleratingRL = movingStrafe;
    this->acceleratingUD = movingUp;
    this->velocityVec = velocityVec;
    setNormal();
}

/** ---------- Getters ---------- */
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
int Camera3D::getAcceleratingFB() const {
    return acceleratingFB;
}
int Camera3D::getAcceleratingRL() const {
    return acceleratingRL;
}
int Camera3D::getAcceleratingUD() const {
    return acceleratingUD;
}
bool Camera3D::isMoving() const {
    for (double i: velocityVec.components){
        if (i != 0.0){
            return true;
        }
    }
    return false;
}

/** ---------- Setters ---------- */
/* Utility */
/**
 * Sets the focus to a point3d that is this.focalDistance units away from
 * this.location in the OPPOSITE direction of this.normal.
 */
void Camera3D::setFocus(){
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
void Camera3D::setLocation(std::vector<double> newLocation){
    if (newLocation.size() == 3){
        setLocation(newLocation[0], newLocation[1], newLocation[2]);
    } else {
        std::cout << "Warning: Invalid input in:\n\tvoid "
                     "Camera3D::setLocation(std::vector<double> newLocation)"
                     "\n\t(Camera3D.cpp)" << std::endl;
    }
}
void Camera3D::setLocation(const point3d& newLocation){
    this->location = point3d(newLocation);
}
void Camera3D::setLocation(double x, double y, double z){
    this->location = point3d(x, y, z);
}

/* Rotation */
/**
 * Sets this.normal to the unit vector pointing in the direction of
 * this.sphericalDirection and calls setFocus().
 */
void Camera3D::setNormal(){
    this->normal = sphericalDirection.getUnitVector();
    setFocus();
}
void Camera3D::setSphericalDirection(std::vector<double> newAngles){
    if (newAngles.size() == 2){
       setSphericalDirection(newAngles[0], newAngles[1]);
    } else {
        std::cout << "Warning: Invalid input in:\n\tvoid "
                     "Camera3D::setSphericalDirection(std::vector<double> "
                     "newAngles)\n\t(Camera3d.cpp)" << std::endl;
    }
}
void Camera3D::setSphericalDirection(const sphericalAngle3d& newAngles){
    setSphericalDirection(newAngles.polarAngle,newAngles.azimuthAngle);
}
void Camera3D::setSphericalDirection(double polarAngle, double azimuthAngle){
    setPolar(polarAngle);
    setAzimuth(azimuthAngle);
}
void Camera3D::setPolar(double polarAngle){
    sphericalDirection.setPolar(polarAngle);

    // Reset normal and focus
    setNormal();
}
void Camera3D::setAzimuth(double azimuthAngle){
    sphericalDirection.setAzimuth(azimuthAngle);
    setNormal();
}

/** ---------- Other Methods ---------- */
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
    // First determine if point is in front of the focus. If the vector from
    // the Camera's location to the point has a scalar projection <= 0 onto
    // the normal, it is on or behind the plane defined by the focus and the
    // camera's normal. If this is the case, do not render - return nullopt.
    spatialVector focusToPoint({
        p.x - focus.x,
        p.y - focus.y,
        p.z - focus.z
    });
    if (focusToPoint.scalarProjectOnto(normal) <= 0){
        return nullopt;
    }
    spatialVector cameraToPoint({p.x - location.x,
                                 p.y - location.y,
                                 p.z - location.z});

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
    spatialVector cameraToIntersection({
        intersectionPoint.x - location.x,
        intersectionPoint.y - location.y,
        intersectionPoint.z - location.z
    });

    // Now x' and y' are just the scalar projections of cameraToIntersection
    // onto the vector pointing straight "up" and the vector pointing
    // straight right. These give 2d coordinates on the screen where the
    // point should actually be drawn.
    return point2d(cameraToIntersection.scalarProjectOnto(getUnitRightVector()),
            cameraToIntersection.scalarProjectOnto(getUnitUpVector()));

}

/* Movement */
void Camera3D::moveAbsolute(const std::vector<double> dPosition){
    if (dPosition.size() == 3){
        location.move(dPosition);
        setFocus();
    } else {
        std::cout << "Warning: Invalid input in:\n\tvoid Camera3D::moveAbsolute"
                     "(std::vector<double> dPosition)\n\t(Camera3D.cpp)"
                     << std::endl;
    }
}
void Camera3D::moveAbsolute(const spatialVector& dPosition){
    moveAbsolute(dPosition.components);
}
void Camera3D::moveAbsolute(double dX, double dY, double dZ){
    location.move(dX, dY, dZ);
    setFocus();
}
void Camera3D::moveAbsoluteX(double dX){
    location.moveX(dX);
    setFocus();
}
void Camera3D::moveAbsoluteY(double dY){
    location.moveY(dY);
    setFocus();
}
void Camera3D::moveAbsoluteZ(double dZ){
    location.moveZ(dZ);
    setFocus();
}

void Camera3D::moveRelative(const std::vector<double>& dPosition){
    if (dPosition.size() == 3){
        moveRelativeF(dPosition[0]);
        moveRelativeR(dPosition[1]);
        moveRelativeU(dPosition[2]);
    } else {
        std::cout << "Warning: Invalid input in:"
                     "\n\tvoid Camera3D::moveRelative("
                        "std::vector<double> dPosition)"
                    "\n\t(Camera3D.cpp)" << std::endl;
    }
}
void Camera3D::moveRelative(const spatialVector &dPosition){
    moveRelative(dPosition.components);
}
void Camera3D::moveRelativeFB(double dF){
    moveRelativeF(dF);
}
void Camera3D::moveRelativeRL(double dR){
    moveRelativeR(dR);
}
void Camera3D::moveRelativeUD(double dU){
    moveRelativeU(dU);
}
void Camera3D::moveRelativeF(double dF){
    // Get vector in direction of current polarAngle (locks movement
    // to horizontal x/y plane only so moving forward while looking
    // up/down cannot change z)
    spatialVector dPositionVec = sphericalAngle3d(
        sphericalDirection.polarAngle,
        90
    ).getUnitVector();
    dPositionVec.scale(dF);
    moveAbsolute(dPositionVec);
}
void Camera3D::moveRelativeB(double dB){
    moveRelativeF(-dB);
}
void Camera3D::moveRelativeR(double dR){
    spatialVector dPositionVec = getUnitRightVector();
    dPositionVec.scale(dR);
    moveAbsolute(dPositionVec);
}
void Camera3D::moveRelativeL(double dL){
    moveRelativeR(-dL);
}
void Camera3D::moveRelativeU(double dU){
    moveAbsoluteZ(dU);
}
void Camera3D::moveRelativeD(double dD){
    moveRelativeU(-dD);
}

void Camera3D::moveRelativeFixedF(){
    moveRelativeF(DEFAULT_MOVE_DISTANCE);
}
void Camera3D::moveRelativeFixedB(){
    moveRelativeB(DEFAULT_MOVE_DISTANCE);
}
void Camera3D::moveRelativeFixedR(){
    moveRelativeR(DEFAULT_MOVE_DISTANCE);
}
void Camera3D::moveRelativeFixedL(){
    moveRelativeL(DEFAULT_MOVE_DISTANCE);
}
void Camera3D::moveRelativeFixedU(){
    moveRelativeU(DEFAULT_MOVE_DISTANCE);
}
void Camera3D::moveRelativeFixedD(){
    moveRelativeD(DEFAULT_MOVE_DISTANCE);
}

void Camera3D::moveFly(){
    updateVelocities();
    moveRelative(velocityVec);
}
void Camera3D::setAccelerating(const int acceleratingFB_,
                               const int acceleratingRL_,
                               const int acceleratingUD_){
    setAcceleratingFB(acceleratingFB_);
    setAcceleratingRL(acceleratingRL_);
    setAcceleratingUD(acceleratingUD_);
}
void Camera3D::setAcceleratingFB(int acceleratingFB_){
    if (-1 <= acceleratingFB_ and acceleratingFB_ <= 1){
        acceleratingFB = acceleratingFB_;
    } else {
        std::cout << "Warning: Invalid input (movingForward_) in:"
                     "\n\tvoid Camera3D::setAcceleratingFB(int movingForward_)"
                     "\n\t(Camera3D.cpp)" << std::endl;
    }
}
void Camera3D::setAcceleratingRL(int acceleratingRL_){
    if (-1 <= acceleratingRL_ and acceleratingRL_ <= 1){
        acceleratingRL = acceleratingRL_;
    } else {
        std::cout << "Warning: Invalid input (movingStrafe_) in:"
                     "\n\tvoid Camera3D::setAcceleratingRL(int movingStrafe_)"
                     "\n\t(Camera3D.cpp)" << std::endl;
    }
}
void Camera3D::setAcceleratingUD(int acceleratingUD_){
    if (-1 <= acceleratingUD_ and acceleratingUD_ <= 1){
        acceleratingUD = acceleratingUD_;
    } else {
        std::cout << "Warning: Invalid input (movingUp_) in:"
                     "\n\tvoid Camera3D::setAcceleratingUD(int movingUp_)"
                     "\n\t(Camera3D.cpp)" << std::endl;
    }
}
void Camera3D::setAcceleratingR(){
    setAcceleratingRL(1);
}
void Camera3D::setAcceleratingL(){
    setAcceleratingRL(-1);
}
void Camera3D::setAcceleratingU(){
    setAcceleratingUD(1);
}
void Camera3D::setAcceleratingD(){
    setAcceleratingUD(-1);
}
void Camera3D::setAcceleratingF(){
    setAcceleratingFB(1);
}
void Camera3D::setAcceleratingB(){
    setAcceleratingFB(-1);
}
void Camera3D::updateVelocities(){
    // Update values in velocityVec by one game tick.
    updateVelFB();
    updateVelRL();
    updateVelUD();
}
void Camera3D::updateVelFB() {
    // Always apply drag regardless of whether camera is accelerating
    applyDragFB();

    if (acceleratingFB != 0){
        if (velocityVec.components[0]/acceleratingFB >= 0){
            // Current velocity is 0, or trying to accelerate in same
            // direction as current velocity
            if (acceleratingFB > 0){
                // Positive acceleration until max speed is reached
                velocityVec.components[0] = std::min(
                        velocityVec.components[0] + FB_ACCEL
                            * Scene::SECONDS_PER_TICK,
                        FB_MAX_SPEED
                );
            } else {
                // Negative acceleration until min speed is reached
                velocityVec.components[0] = std::max(
                        velocityVec.components[0] - FB_ACCEL
                            * Scene::SECONDS_PER_TICK,
                        -FB_MAX_SPEED
                );
            }
        } else {
            // Accelerating against current velocity - apply brake instead
            brakeFB();
        }
    }
}
void Camera3D::updateVelRL() {
    // Always apply drag regardless of whether camera is accelerating
    applyDragRL();

    if (acceleratingRL != 0){
        if (velocityVec.components[1]/acceleratingRL >= 0){
            // Current velocity is 0, or trying to accelerate in same
            // direction as current velocity
            if (acceleratingRL > 0){
                // Positive acceleration until max speed is reached
                velocityVec.components[1] = std::min(
                        velocityVec.components[1] + RL_ACCEL
                            * Scene::SECONDS_PER_TICK,
                        RL_MAX_SPEED
                );
            } else {
                // Negative acceleration until min speed is reached
                velocityVec.components[1] = std::max(
                        velocityVec.components[1] - RL_ACCEL
                            * Scene::SECONDS_PER_TICK,
                        -RL_MAX_SPEED
                );
            }
        } else {
            // Accelerating against current velocity - apply brake instead
            brakeRL();
        }
    }
}
void Camera3D::updateVelUD() {
    // Always apply drag regardless of whether camera is accelerating
    applyDragUD();

    if (acceleratingUD != 0){
        if (velocityVec.components[2]/acceleratingUD >= 0){
            // Current velocity is 0, or trying to accelerate in same
            // direction as current velocity
            if (acceleratingUD > 0){
                // Positive acceleration until max speed is reached
                velocityVec.components[2] = std::min(
                        velocityVec.components[2] + UD_ACCEL
                            * Scene::SECONDS_PER_TICK,
                        UD_MAX_SPEED
                );
            } else {
                // Negative acceleration until min speed is reached
                velocityVec.components[2] = std::max(
                        velocityVec.components[2] - UD_ACCEL
                            * Scene::SECONDS_PER_TICK,
                        -UD_MAX_SPEED
                );
            }
        } else {
            // Accelerating against current velocity - apply brake instead
            brakeUD();
        }
    }
}
void Camera3D::applyDrag(){
    applyDragFB();
    applyDragRL();
    applyDragUD();
}
void Camera3D::applyDragFB(){
    if (velocityVec.components[0] != 0){
        // No drag if current velocity already 0
        if (velocityVec.components[0] > 0){
            // Drag down to 0
            velocityVec.components[0] = std::max(
                velocityVec.components[0] - FB_DRAG
                                            * Scene::SECONDS_PER_TICK,
                    0.
            );
        } else {
            // Drag up to 0
            velocityVec.components[0] = std::min(
                velocityVec.components[0] + FB_DRAG
                                            * Scene::SECONDS_PER_TICK,
                    0.
            );
        }
    }
}
void Camera3D::applyDragRL(){
    if (velocityVec.components[1] != 0){
        // No drag if current velocity already 0
        if (velocityVec.components[1] > 0){
            // Drag down to 0
            velocityVec.components[1] = std::max(
                velocityVec.components[1] - RL_DRAG
                                            * Scene::SECONDS_PER_TICK,
                    0.
            );
        } else {
            // Drag up to 0
            velocityVec.components[1] = std::min(
                velocityVec.components[1] + RL_DRAG
                                            * Scene::SECONDS_PER_TICK,
                    0.
            );
        }
    }
}
void Camera3D::applyDragUD(){
    if (velocityVec.components[2] != 0){
        // No drag if current velocity already 0
        if (velocityVec.components[2] > 0){
            // Drag down to 0
            velocityVec.components[2] = std::max(
                velocityVec.components[2] - UD_DRAG
                                            * Scene::SECONDS_PER_TICK,
                    0.
            );
        } else {
            // Drag up to 0
            velocityVec.components[2] = std::min(
                velocityVec.components[2] + UD_DRAG
                                            * Scene::SECONDS_PER_TICK,
                    0.
            );
        }
    }
}
void Camera3D::brakeFB(){
    if (velocityVec.components[0] > 0){
        // Break down to 0
        velocityVec.components[0] = std::max(
            velocityVec.components[0] - FB_BRAKE
                                        * Scene::SECONDS_PER_TICK,
                0.
        );

        if (velocityVec.components[0] == 0){
            // Reset acceleration when velocity gets to 0
            setAcceleratingFB(0);
        }
    } else if (velocityVec.components[0] < 0){
        // Break up to 0
        velocityVec.components[0] = std::min(
            velocityVec.components[0] + FB_BRAKE
                                        * Scene::SECONDS_PER_TICK,
                0.
        );

        if (velocityVec.components[0] == 0){
            // Reset acceleration when velocity gets to 0
            setAcceleratingFB(0);
        }
    } else {
        std::cout << "Warning: Breaking called when velocityVec.components[0]"
                     " == 0 in:"
                     "\n\tvoid Camera3D::brakeFB()"
                     "\n\t(Camera3D.cpp)" << std::endl;
    }
}
void Camera3D::brakeRL(){
    if (velocityVec.components[1] > 0){
        // Break down to 0
        velocityVec.components[1] = std::max(
            velocityVec.components[1] - RL_BRAKE
                                        * Scene::SECONDS_PER_TICK,
                0.
        );

        if (velocityVec.components[1] == 0){
            // Reset acceleration when velocity gets to 0
            setAcceleratingRL(0);
        }
    } else if (velocityVec.components[1] < 0){
        // Break up to 0
        velocityVec.components[1] = std::min(
            velocityVec.components[1] + RL_BRAKE
                                        * Scene::SECONDS_PER_TICK,
                0.
        );

        if (velocityVec.components[1] == 0){
            // Reset acceleration when velocity gets to 0
            setAcceleratingRL(0);
        }
    } else {
        std::cout << "Warning: Breaking called when velocityVec.components[1]"
                     " == 0 in:"
                     "\n\tvoid Camera3D::brakeRL()"
                     "\n\t(Camera3D.cpp)" << std::endl;
    }
}
void Camera3D::brakeUD(){
    if (velocityVec.components[2] > 0){
        // Break down to 0
        velocityVec.components[2] = std::max(
            velocityVec.components[2] - UD_BRAKE
                                        * Scene::SECONDS_PER_TICK,
                0.
        );

        if (velocityVec.components[2] == 0){
            // Reset acceleration when velocity gets to 0
            setAcceleratingUD(0);
        }
    } else if (velocityVec.components[2] < 0){
        // Break up to 0
        velocityVec.components[2] = std::min(
            velocityVec.components[2] + UD_BRAKE
                                        * Scene::SECONDS_PER_TICK,
                0.
        );

        if (velocityVec.components[2] == 0){
            // Reset acceleration when velocity gets to 0
            setAcceleratingUD(0);
        }
    } else {
        std::cout << "Warning: Breaking called when velocityVec.components[2]"
                     " == 0 in:"
                     "\n\tvoid Camera3D::brakeUD()"
                     "\n\t(Camera3D.cpp)" << std::endl;
    }
}
void Camera3D::moveR(){
    if (movementMode == Camera::MovementMode::Fixed){
        moveRelativeFixedR();
    } else if (movementMode == Camera::MovementMode::Fly){
        setAcceleratingR();
    } else {
        std::cout << "Warning: Invalid movementMode in:"
                     "\n\tvoid Camera3D::moveR()"
                     "\n\t(Camera3D.cpp)" << std::endl;
    }
}
void Camera3D::moveL(){
    if (movementMode == Camera::MovementMode::Fixed){
        moveRelativeFixedL();
    } else if (movementMode == Camera::MovementMode::Fly){
        setAcceleratingL();
    } else {
        std::cout << "Warning: Invalid movementMode in:"
                     "\n\tvoid Camera3D::moveL()"
                     "\n\t(Camera3D.cpp)" << std::endl;
    }
}
void Camera3D::moveU(){
    if (movementMode == Camera::MovementMode::Fixed){
        moveRelativeFixedU();
    } else if (movementMode == Camera::MovementMode::Fly){
        setAcceleratingU();
    } else {
        std::cout << "Warning: Invalid movementMode in:"
                     "\n\tvoid Camera3D::moveU()"
                     "\n\t(Camera3D.cpp)" << std::endl;
    }
}
void Camera3D::moveD(){
    if (movementMode == Camera::MovementMode::Fixed){
        moveRelativeFixedD();
    } else if (movementMode == Camera::MovementMode::Fly){
        setAcceleratingD();
    } else {
        std::cout << "Warning: Invalid movementMode in:"
                     "\n\tvoid Camera3D::moveD()"
                     "\n\t(Camera3D.cpp)" << std::endl;
    }
}
void Camera3D::moveF(){
    if (movementMode == Camera::MovementMode::Fixed){
        moveRelativeFixedF();
    } else if (movementMode == Camera::MovementMode::Fly){
        setAcceleratingF();
    } else {
        std::cout << "Warning: Invalid movementMode in:"
                     "\n\tvoid Camera3D::moveF()"
                     "\n\t(Camera3D.cpp)" << std::endl;
    }
}
void Camera3D::moveB(){
    if (movementMode == Camera::MovementMode::Fixed){
        moveRelativeFixedB();
    } else if (movementMode == Camera::MovementMode::Fly){
        setAcceleratingB();
    } else {
        std::cout << "Warning: Invalid movementMode in:"
                     "\n\tvoid Camera3D::moveB()"
                     "\n\t(Camera3D.cpp)" << std::endl;
    }
}

/* Rotation */
void Camera3D::rotate(const std::vector<double> dAngles){
    if (dAngles.size() == 2){
        rotate(dAngles[0], dAngles[1]);
    } else {
        std::cout << "Warning: Invalid input in:\n\tvoid Camera3D::rotate"
                     "(std::vector<double> dAngles)\n\t(Camera3D.cpp)"
                     << std::endl;
    }
}
void Camera3D::rotate(const sphericalAngle3d& dAngles){
    rotate(dAngles.polarAngle, dAngles.azimuthAngle);
}
void Camera3D::rotate(const double dPolarAngle, const double dAzimuthAngle){
    rotatePolar(dPolarAngle, false);
    rotateAzimuth(dAzimuthAngle, false);
}
void Camera3D::rotatePolar(const double dPolarAngle){
    rotatePolar(dPolarAngle, true);
}
void Camera3D::rotatePolar(const double dPolarAngle, bool updateNormal){
    sphericalDirection.rotatePolar(dPolarAngle);
    if (updateNormal){
        setNormal();
    }
}
void Camera3D::rotateAzimuth(const double dAzimuthAngle){
    rotateAzimuth(dAzimuthAngle, true);
}
void Camera3D::rotateAzimuth(const double dAzimuthAngle, bool updateNormal){
    sphericalDirection.rotateAzimuth(dAzimuthAngle);
    if (updateNormal){
        setNormal();
    }
}
void Camera3D::rotateRight(){
    rotatePolar(-DEFAULT_ROTATION_ANGLE);
}
void Camera3D::rotateLeft(){
    rotatePolar(DEFAULT_ROTATION_ANGLE);
}
void Camera3D::rotateUp(){
    rotateAzimuth(-DEFAULT_ROTATION_ANGLE);
}
void Camera3D::rotateDown(){
    rotateAzimuth(DEFAULT_ROTATION_ANGLE);
}
