//
// Created by Jackson Hall on 4/29/2020.
//

#include "Camera3D.h"
#include "Scene.h"


const double Camera3D::DEFAULT_FOV_DEGREES = 160;
const double Camera3D::DEFAULT_PROJECTION_PLANE_WIDTH_BLOCKS = 10;

// Based on constants found at:
// https://gaming.stackexchange.com/questions/327830/what-is-the-player-acceleration-in-minecraft-when-flying
// and
// https://github.com/ddevault/TrueCraft/wiki/Entity-Movement-And-Physics
;
const double Camera3D::FB_ACCEL = 8;        // blocks/second^2
const double Camera3D::RL_ACCEL = 5;        // blocks/second^2
const double Camera3D::UD_ACCEL = 10;       // blocks/second^2
const double Camera3D::FB_DRAG = 5;         // blocks/second^2
const double Camera3D::RL_DRAG = 5;         // blocks/second^2
const double Camera3D::UD_DRAG = 5;         // blocks/second^2
const double Camera3D::FB_BRAKE = 16;       // blocks/second^2
const double Camera3D::RL_BRAKE = 10;       // blocks/second^2
const double Camera3D::UD_BRAKE = 20;       // blocks/second^2
const double Camera3D::FB_MAX_SPEED = 3;    // blocks/second
const double Camera3D::RL_MAX_SPEED = 2;    // blocks/second
const double Camera3D::UD_MAX_SPEED = 1;    // blocks/second

Camera3D::Camera3D() : Camera3D(
        point3d(),
        spatialVector({0, 1, 0}),
        sphericalAngle3d(0, 90),
        point3d(),
        Camera::getFocalDistanceFromFOV(Camera3D::DEFAULT_FOV_DEGREES,
            Camera3D::DEFAULT_PROJECTION_PLANE_WIDTH_BLOCKS),
        Camera3D::DEFAULT_PROJECTION_PLANE_WIDTH_BLOCKS,
        Camera::MovementMode::Fly,
        0,
        0,
        0,
        0,
        0,
        0){
}
Camera3D::Camera3D(const Camera3D &other) : Camera3D(
        point3d(other.location),
        spatialVector(other.normal),
        sphericalAngle3d(other.sphericalDirection),
        point3d(other.focus),
        other.focalDistance,
        other.projectionPlaneWidthBlocks,
        other.movementMode,
        other.accelerationF,
        other.accelerationR,
        other.accelerationU,
        other.velocityF,
        other.velocityR,
        other.velocityU){
}
Camera3D::Camera3D(
        const point3d& location,
        const spatialVector& normal,
        const sphericalAngle3d& sphericalDirection,
        const point3d& focus,
        const double& focalDistance,
        const double& projectionPlaneWidthBlocks,
        Camera::MovementMode movementMode,
        const double& accelerationF,
        const double& accelerationR,
        const double& accelerationU,
        const double& velocityF,
        const double& velocityR,
        const double& velocityU)
        : Camera(focalDistance, movementMode){ // NOLINT(performance-unnecessary-value-param)
    this->location = location;
    this->normal = normal;
    this->sphericalDirection = sphericalDirection;
    this->focus = focus;
    this->movementMode = movementMode;
    this->projectionPlaneWidthBlocks = projectionPlaneWidthBlocks;
    this->accelerationF = accelerationF;
    this->accelerationR = accelerationR;
    this->accelerationU = accelerationU;
    this->velocityF = velocityF;
    this->velocityR = velocityR;
    this->velocityU = velocityU;
    this->normal = spatialVector();
    setNormal();
}

const point3d& Camera3D::getLocation() const {
    return location;
}
const spatialVector& Camera3D::getNormal() const {
    return normal;
}
const sphericalAngle3d& Camera3D::getSphericalDirection() const {
    return sphericalDirection;
}
const point3d& Camera3D::getFocus() const {
    return focus;
}

const double& Camera3D::getAccelerationF() const {
    return accelerationF;
}
const double& Camera3D::getAccelerationR() const {
    return accelerationR;
}
const double& Camera3D::getAccelerationU() const {
    return accelerationU;
}
const double& Camera3D::getVelocityF() const {
    return velocityF;
}
const double& Camera3D::getVelocityR() const {
    return velocityR;
}
const double& Camera3D::getVelocityU() const {
    return velocityU;
}


/** Other **/
spatialVector Camera3D::getUnitUpVector() const {
    return sphericalAngle3d(
            sphericalDirection.polarAngle,
            sphericalDirection.azimuthAngle - 90).getUnitVector();
}
spatialVector Camera3D::getUnitRightVector() const {
    sphericalAngle3d temp(sphericalDirection.polarAngle, 90);
    temp.rotatePolar(-90);
    return temp.getUnitVector();
}

/** ---------- Setters ---------- **/
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
/**
 * Sets this.normal to the unit vector pointing in the direction of
 * this.sphericalDirection and calls setFocus().
 */
void Camera3D::setNormal() {
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

/** ---------- Other Methods ---------- **/
/** Utility**/
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

/** ---------- Movement ---------- **/
/** Main API Calls **/
bool _validateMovementMode(Camera::MovementMode movementMode) {
    if (movementMode != Camera::MovementMode::Fixed
            && movementMode != Camera::MovementMode::Fly){
        std::cout << "Warning: Movement mode not supported: " << movementMode
        << std::endl;
        return false;
    }
    return true;
}

void Camera3D::moveF(){
    if (!_validateMovementMode(movementMode)) {
        return;
    }

    if (movementMode == Camera::MovementMode::Fixed){
        moveRelativeF();
    } else if (movementMode == Camera::MovementMode::Fly){
        setAccelerationF();
    }
}
void Camera3D::moveB(){
    if (!_validateMovementMode(movementMode)) {
        return;
    }

    if (movementMode == Camera::MovementMode::Fixed){
        moveRelativeB();
    } else if (movementMode == Camera::MovementMode::Fly){
        setAccelerationB();
    }
}
void Camera3D::moveR(){
    if (!_validateMovementMode(movementMode)) {
        return;
    }

    if (movementMode == Camera::MovementMode::Fixed){
        moveRelativeR();
    } else if (movementMode == Camera::MovementMode::Fly){
        setAccelerationR();
    }
}
void Camera3D::moveL(){
    if (!_validateMovementMode(movementMode)) {
        return;
    }

    if (movementMode == Camera::MovementMode::Fixed){
        moveRelativeL();
    } else if (movementMode == Camera::MovementMode::Fly){
        setAccelerationL();
    }
}
void Camera3D::moveU(){
    if (!_validateMovementMode(movementMode)) {
        return;
    }

    if (movementMode == Camera::MovementMode::Fixed){
        moveRelativeU();
    } else if (movementMode == Camera::MovementMode::Fly){
        setAccelerationU();
    }
}
void Camera3D::moveD(){
    if (!_validateMovementMode(movementMode)) {
        return;
    }

    if (movementMode == Camera::MovementMode::Fixed){
        moveRelativeD();
    } else if (movementMode == Camera::MovementMode::Fly){
        setAccelerationD();
    }
}

bool Camera3D::isMoving() const {
    return getVelocityF() != 0.0 or getVelocityR() != 0.0 or getVelocityU()
    != 0;
}

void Camera3D::moveAbsolute(const std::vector<double> dPosition){
    if (dPosition.size() != 3) {
        std::cout << "Warning: Invalid `dPosition`" << std::endl;
        return;
    }

    location.move(dPosition);
    setFocus();
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
        moveRelative(dPosition[0], dPosition[1], dPosition[2]);
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
void Camera3D::moveRelative(double dF, double dR, double dU) {
    moveRelativeF(dF);
    moveRelativeR(dR);
    moveRelativeU(dU);
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

void Camera3D::moveFly(){
    updateVelocities();
    moveRelative(getVelocityF(), getVelocityR(), getVelocityU());
}

void Camera3D::setAcceleration(const double newAccelerationF,
                               const double newAccelerationR,
                               const double newAccelerationU) {
    setAccelerationF(newAccelerationF);
    setAccelerationR(newAccelerationR);
    setAccelerationU(newAccelerationU);
}

bool _validateNewAcceleration(const double newAccel) {
    if (!isBetween(newAccel, -1, 1)) {
        std::cout
        << "Warning: Invalid input, must be between -1 and 1 "
           "(" << newAccel << ")" << std::endl;
        return false;
    }
    return true;

}
void Camera3D::setAccelerationF(const double newAccelerationF) {
    if (!_validateNewAcceleration(newAccelerationF)) {
        return;
    }
    accelerationF = newAccelerationF;
}
void Camera3D::setAccelerationR(const double newAccelerationR) {
    if (!_validateNewAcceleration(newAccelerationR)) {
        return;
    }
    accelerationR = newAccelerationR;
}
void Camera3D::setAccelerationU(const double newAccelerationU) {
    if (!_validateNewAcceleration(newAccelerationU)) {
        return;
    }
    accelerationU = newAccelerationU;
}
void Camera3D::setAccelerationB(const double newAccelerationB) {
    setAccelerationF(-newAccelerationB);
}
void Camera3D::setAccelerationL(const double newAccelerationL) {
    setAccelerationR(-newAccelerationL);
}
void Camera3D::setAccelerationD(const double newAccelerationD) {
    setAccelerationU(-newAccelerationD);
}

void Camera3D::updateVelocities(){
    // Update velocity values by one game tick.
    updateVelFB();
    updateVelRL();
    updateVelUD();
}
void Camera3D::updateVelFB() {
    if (accelerationF == 0.0) {
        applyDragFB();
        return;
    }

    if (!sameSign(velocityF, accelerationF)) {
        applyBrakeFB();
        return;
    }

    velocityF += accelerationF * FB_ACCEL * Scene::SECONDS_PER_TICK;
    velocityF = clamp(velocityF, -FB_MAX_SPEED, FB_MAX_SPEED);
}
void Camera3D::updateVelRL() {
    if (accelerationR == 0.0) {
        applyDragRL();
        return;
    }

    if (!sameSign(velocityR, accelerationR)) {
        applyBrakeRL();
        return;
    }

    velocityR += accelerationR * RL_ACCEL * Scene::SECONDS_PER_TICK;
    velocityR = clamp(velocityR, -RL_MAX_SPEED, RL_MAX_SPEED);
}
void Camera3D::updateVelUD() {
    if (accelerationU == 0.0) {
        applyDragUD();
        return;
    }

    if (!sameSign(velocityU, accelerationU)) {
        applyBrakeUD();
        return;
    }

    velocityU += accelerationU * UD_ACCEL * Scene::SECONDS_PER_TICK;
    velocityU = clamp(velocityU, -UD_MAX_SPEED, UD_MAX_SPEED);
}

void Camera3D::applyDrag() {
    applyDragFB();
    applyDragRL();
    applyDragUD();
}
void Camera3D::applyDragFB() {
    if (velocityF == 0.0) {
        return;
    }

    const double dVel = FB_DRAG * Scene::SECONDS_PER_TICK;
    if (velocityF > 0) {
        // Drag down to 0
        velocityF = std::max(velocityF - dVel, 0.0);
    } else {
        // Drag up to 0
        velocityF = std::min(velocityF + dVel, 0.0);
    }
}
void Camera3D::applyDragRL() {
    if (velocityR == 0.0) {
        return;
    }

    const double dVel = RL_DRAG * Scene::SECONDS_PER_TICK;
    if (velocityR > 0) {
        // Drag down to 0
        velocityR = std::max(velocityR - dVel, 0.0);
    } else {
        // Drag up to 0
        velocityR = std::min(velocityR + dVel, 0.0);
    }
}
void Camera3D::applyDragUD() {
    if (velocityU == 0.0) {
        return;
    }

    const double dVel = UD_DRAG * Scene::SECONDS_PER_TICK;
    if (velocityU > 0) {
        // Drag down to 0
        velocityU = std::max(velocityU - dVel, 0.0);
    } else {
        // Drag up to 0
        velocityU = std::min(velocityU + dVel, 0.0);
    }
}

void Camera3D::applyBrakeFB() {
    if (velocityF == 0.0) {
        return;
    }

    const double dVel = FB_BRAKE * Scene::SECONDS_PER_TICK;
    if (velocityF > 0) {
        // Brake down to 0
        velocityF = std::max(velocityF - dVel, 0.0);
    } else {
        // Brake up to 0
        velocityF = std::min(velocityF + dVel, 0.0);
    }

    // When velocity gets to 0, stop moving
    if (velocityF == 0.0) {
        setAccelerationF(0);
    }
}
void Camera3D::applyBrakeRL() {
    if (velocityR == 0.0) {
        return;
    }

    const double dVel = RL_BRAKE * Scene::SECONDS_PER_TICK;
    if (velocityR > 0) {
        // Brake down to 0
        velocityR = std::max(velocityR - dVel, 0.0);
    } else {
        // Brake up to 0
        velocityR = std::min(velocityR + dVel, 0.0);
    }

    // When velocity gets to 0, stop moving
    if (velocityR == 0.0) {
        setAccelerationR(0);
    }
}
void Camera3D::applyBrakeUD() {
    if (velocityU == 0.0) {
        return;
    }

    const double dVel = UD_BRAKE * Scene::SECONDS_PER_TICK;
    if (velocityU > 0) {
        // Brake down to 0
        velocityU = std::max(velocityU - dVel, 0.0);
    } else {
        // Brake up to 0
        velocityU = std::min(velocityU + dVel, 0.0);
    }

    // When velocity gets to 0, stop moving
    if (velocityU == 0.0) {
        setAccelerationU(0);
    }
}

/** ---------- Rotation ---------- **/
/** Rotation**/
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
void Camera3D::rotateRight(const double dDeg){
    rotatePolar(-dDeg);
}
void Camera3D::rotateLeft(const double dDeg){
    rotatePolar(dDeg);
}
void Camera3D::rotateUp(const double dDeg){
    rotateAzimuth(-dDeg);
}
void Camera3D::rotateDown(const double dDeg){
    rotateAzimuth(dDeg);
}
