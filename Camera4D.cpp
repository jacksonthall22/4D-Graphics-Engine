//
// Created by Jackson Hall on 4/29/2020.
//

#include "Camera4D.h"
#include "Scene.h"


/** ---------- Static Const Vars ---------- */
const double Camera4D::DEFAULT_FOV_DEGREES = 160;
const double Camera4D::DEFAULT_PROJECTION_PLANE_WIDTH_BLOCKS = 10;

// (Some) credit to:
// https://gaming.stackexchange.com/questions/327830/what-is-the-player-
// acceleration-in-minecraft-when-flying
// and
// https://github.com/ddevault/TrueCraft/wiki/Entity-Movement-And-Physics
const double Camera4D::FB_ACCEL = 6;     // blocks/second^2
const double Camera4D::RL_ACCEL = 6;
const double Camera4D::UD_ACCEL = 6;
const double Camera4D::OI_ACCEL = 10;
const double Camera4D::FB_DRAG = 3;      // blocks/second^2
const double Camera4D::RL_DRAG = 3;
const double Camera4D::UD_DRAG = 3;
const double Camera4D::OI_DRAG = 5;
const double Camera4D::FB_BRAKE = 5;     // blocks/second^2
const double Camera4D::RL_BRAKE = 5;
const double Camera4D::UD_BRAKE = 5;
const double Camera4D::OI_BRAKE = 10;
const double Camera4D::FB_MAX_SPEED = 3; // blocks/second
const double Camera4D::RL_MAX_SPEED = 3;
const double Camera4D::UD_MAX_SPEED = 3;
const double Camera4D::OI_MAX_SPEED = 1;

/** ---------- Constructors ---------- */
Camera4D::Camera4D() : Camera4D(
        point4d(),
        spatialVector({0, 0, 1, 0}),
        sphericalAngle4d(0, 0, 90),
        point4d(),
        Camera::getFocalDistanceFromFOV(Camera4D::DEFAULT_FOV_DEGREES,
            Camera4D::DEFAULT_PROJECTION_PLANE_WIDTH_BLOCKS),
        Camera4D::DEFAULT_PROJECTION_PLANE_WIDTH_BLOCKS,
        Camera::MovementMode::Fly,
        0, 0, 0, 0,
        spatialVector(4)){
}
Camera4D::Camera4D(const Camera4D &other) : Camera4D(
        point4d(other.location),
        spatialVector(other.normal),
        sphericalAngle4d(other.sphericalDirection),
        point4d(other.focus),
        other.focalDistance,
        other.projectionPlaneWidthBlocks,
        other.movementMode,
        other.acceleratingFB,
        other.acceleratingRL,
        other.acceleratingUD,
        other.acceleratingOI,
        other.velocityFRUO){
}
Camera4D::Camera4D(
        const point4d& location,
        const spatialVector& normal,
        const sphericalAngle4d& sphericalDirection,
        const point4d& focus,
        const double& focalDistance,
        const double& projectionPlaneWidthBlocks,
        Camera::MovementMode movementMode,
        const int& acceleratingFB,
        const int& acceleratingRL,
        const int& acceleratingUD,
        const int& acceleratingOI,
        const spatialVector& velocityFRUO)
        : Camera(focalDistance, movementMode){
    this->location = location;
    this->normal = normal;
    this->sphericalDirection = sphericalDirection;
    this->focus = focus;
    this->movementMode = movementMode;
    this->projectionPlaneWidthBlocks = projectionPlaneWidthBlocks;
    this->acceleratingFB = acceleratingFB;
    this->acceleratingRL = acceleratingRL;
    this->acceleratingUD = acceleratingUD;
    this->acceleratingOI = acceleratingOI;
    this->velocityFRUO = velocityFRUO;
    this->normal = spatialVector();
    setNormal();
}

/** ---------- Getters ---------- */
/** Const references **/
const point4d& Camera4D::getLocation() const {
    return location;
}
const spatialVector& Camera4D::getNormal() const {
    return normal;
}
const sphericalAngle4d& Camera4D::getSphericalDirection() const {
    return sphericalDirection;
}
const point4d& Camera4D::getFocus() const {
    return focus;
}

/** Non-const references **/
point4d& Camera4D::getLocation() {
    return location;
}
spatialVector& Camera4D::getNormal() {
    return normal;
}
sphericalAngle4d& Camera4D::getSphericalDirection() {
    return sphericalDirection;
}
point4d& Camera4D::getFocus() {
    return focus;
}
int& Camera4D::getAcceleratingFB() {
    return acceleratingFB;
}
int& Camera4D::getAcceleratingRL() {
    return acceleratingRL;
}
int& Camera4D::getAcceleratingUD() {
    return acceleratingUD;
}
int& Camera4D::getAcceleratingOI() {
    return acceleratingOI;
}
double& Camera4D::getVelocityFB() {
    return this->velocityFRUO.components[0];
}
double& Camera4D::getVelocityRL() {
    return this->velocityFRUO.components[1];
}
double& Camera4D::getVelocityUD() {
    return this->velocityFRUO.components[2];
}
double& Camera4D::getVelocityOI() {
    return this->velocityFRUO.components[3];
}

/** Values **/
int Camera4D::getAcceleratingFB() const {
    return acceleratingFB;
}
int Camera4D::getAcceleratingRL() const {
    return acceleratingRL;
}
int Camera4D::getAcceleratingUD() const {
    return acceleratingUD;
}
int Camera4D::getAcceleratingOI() const {
    return acceleratingOI;
}
double Camera4D::getVelocityFB() const {
    return velocityFRUO.components[0];
}
double Camera4D::getVelocityRL() const {
    return velocityFRUO.components[1];
}
double Camera4D::getVelocityUD() const {
    return velocityFRUO.components[2];
}
double Camera4D::getVelocityOI() const {
    return velocityFRUO.components[3];
}

/** Other **/
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


/** ---------- Setters ---------- */
void Camera4D::setLocation(std::vector<double> newLocation){
    if (newLocation.size() == 4){
        setLocation(
            newLocation[0],
            newLocation[1],
            newLocation[2],
            newLocation[3]
        );
    } else {
        // Bad input
        std::cout << "Warning: Invalid input in:"
                     "\n\tvoid Camera4D::setLocation(std::vector<double> newLocation)"
                     "\n\t(Camera4D.cpp)" << std::endl;
    }
}
void Camera4D::setLocation(const point4d &newLocation){
    location = point4d(newLocation);
}
void Camera4D::setLocation(double x, double y, double z, double a){
    this->location = point4d(x, y, z, a);
}
/**
 * Sets this.normal to the unit vector pointing in the direction of
 * this.sphericalDirection.
 */
void Camera4D::setNormal(){
    this->normal = sphericalDirection.getUnitVector();
    setFocus();
}
void Camera4D::setSphericalDirection(std::vector<double> newAngles){
    if (newAngles.size() == 3){
        setSphericalDirection(newAngles[0], newAngles[1], newAngles[2]);
    } else {
        std::cout << "Warning: Invalid input in:"
                     "\n\tvoid Camera4D::setSphericalDirection(std::vector<double> newAngles)"
                     "\n\t(Camera4d.cpp)" << std::endl;
    }
}
void Camera4D::setSphericalDirection(const sphericalAngle4d &newAngles){
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
void Camera4D::setPolarAngle(double polarAngle){
    sphericalDirection.setPolar(polarAngle);
}
void Camera4D::setAzimuthAngle(double azimuthAngle){
    sphericalDirection.setAzimuth(azimuthAngle);
}
void Camera4D::setPhiAngle(double phiAngle){
    sphericalDirection.setPhi(phiAngle);
}
/**
 * Sets the focus to a point4d that is this.focalDistance units away from
 * this.location in the OPPOSITE direction of this.normal.
 */
void Camera4D::setFocus(){
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

/** ---------- Utility / Other ---------- */
optional<point3d> Camera4D::projectPoint(const point4d &p) const {
    // To understand what's happening here, take a look at projectPoint() in
    // Camera3D.cpp and try to imagine a 4d line intersecting the hyperplane of
    // our reality. At least the math is the same.
    //
    // Hyperplane of Camera orthogonal to normal vector n = <A, B, C, D>:
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
    /// TODO - if point is behind camera, don't skip, render from point to
    /// new point3d that is the intersection of edge and focus (do this in
    /// objects' draw())
    spatialVector cameraToPoint(std::vector<double>({
        p.x - location.x,
        p.y - location.y,
        p.z - location.z,
        p.a - location.a
    }));

    spatialVector focusToPoint(std::vector<double>({
        p.x - focus.x,
        p.y - focus.y,
        p.z - focus.z,
        p.a - focus.a
    }));
    if (focusToPoint.scalarProjectOnto(normal) <= 0){
        return nullopt;
    }

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

/** ---------- Movement ---------- */
/** Main API Calls **/
void Camera4D::moveF(){
    if (movementMode == Camera::MovementMode::Fixed){
        moveRelativeDefaultF();
    } else if (movementMode == Camera::MovementMode::Fly){
        setAccelerationDefaultF();
    } else {
        std::cout << "Warning: Invalid movementMode in:"
                     "\n\tvoid Camera4D::moveF()"
                     "\n\t(Camera4D.cpp)" << std::endl;
    }
}
void Camera4D::moveB(){
    if (movementMode == Camera::MovementMode::Fixed){
        moveRelativeDefaultB();
    } else if (movementMode == Camera::MovementMode::Fly){
        setAccelerationDefaultB();
    } else {
        std::cout << "Warning: Invalid movementMode in:"
                     "\n\tvoid Camera4D::moveB()"
                     "\n\t(Camera4D.cpp)" << std::endl;
    }
}
void Camera4D::moveR(){
    if (movementMode == Camera::MovementMode::Fixed){
        moveRelativeDefaultR();
    } else if (movementMode == Camera::MovementMode::Fly){
        setAccelerationDefaultR();
    } else {
        std::cout << "Warning: Invalid movementMode in:"
                     "\n\tvoid Camera4D::moveR()"
                     "\n\t(Camera4D.cpp)" << std::endl;
    }
}
void Camera4D::moveL(){
    if (movementMode == Camera::MovementMode::Fixed){
        moveRelativeDefaultL();
    } else if (movementMode == Camera::MovementMode::Fly){
        setAccelerationDefaultL();
    } else {
        std::cout << "Warning: Invalid movementMode in:"
                     "\n\tvoid Camera4D::moveL()"
                     "\n\t(Camera4D.cpp)" << std::endl;
    }
}
void Camera4D::moveU(){
    if (movementMode == Camera::MovementMode::Fixed){
        moveRelativeDefaultU();
    } else if (movementMode == Camera::MovementMode::Fly){
        setAccelerationDefaultU();
    } else {
        std::cout << "Warning: Invalid movementMode in:"
                     "\n\tvoid Camera4D::moveU()"
                     "\n\t(Camera4D.cpp)" << std::endl;
    }
}
void Camera4D::moveD(){
    if (movementMode == Camera::MovementMode::Fixed){
        moveRelativeDefaultD();
    } else if (movementMode == Camera::MovementMode::Fly){
        setAccelerationDefaultD();
    } else {
        std::cout << "Warning: Invalid movementMode in:"
                     "\n\tvoid Camera4D::moveD()"
                     "\n\t(Camera4D.cpp)" << std::endl;
    }
}
void Camera4D::moveO(){
    if (movementMode == Camera::MovementMode::Fixed){
        moveRelativeDefaultO();
    } else if (movementMode == Camera::MovementMode::Fly){
        setAccelerationDefaultO();
    } else {
        std::cout << "Warning: Invalid movementMode in:"
                     "\n\tvoid Camera4D::moveO()"
                     "\n\t(Camera4D.cpp)" << std::endl;
    }
}
void Camera4D::moveI(){
    if (movementMode == Camera::MovementMode::Fixed){
        moveRelativeDefaultI();
    } else if (movementMode == Camera::MovementMode::Fly){
        setAccelerationDefaultI();
    } else {
        std::cout << "Warning: Invalid movementMode in:"
                     "\n\tvoid Camera4D::moveI()"
                     "\n\t(Camera4D.cpp)" << std::endl;
    }
}

/** Utility **/
bool Camera4D::isMoving() const {
    return getVelocityFB() != 0.0
            or getVelocityRL() != 0.0
            or getVelocityUD() != 0.0
            or getVelocityOI() != 0.0;
}

/** Absolute Movement **/
void Camera4D::moveAbsolute(std::vector<double> dPosition){
    if (dPosition.size() == 4){
        location.move(dPosition);
        setFocus();
    } else {
        std::cout << "Warning: Invalid input in:"
                     "\n\tvoid Camera4D::moveAbsolute(std::vector<double> dPosition)"
                     "\n\t(Camera4D.cpp)" << std::endl;
    }
    setFocus();
}
void Camera4D::moveAbsolute(const spatialVector& dPosition){
    moveAbsolute(dPosition.components);
}
void Camera4D::moveAbsolute(double dX, double dY, double dZ, double dA){
    location.move(dX, dY, dZ, dA);
    setFocus();
}
void Camera4D::moveAbsoluteX(double dX){
    location.moveX(dX);
    setFocus();
}
void Camera4D::moveAbsoluteY(double dY){
    location.moveY(dY);
    setFocus();
}
void Camera4D::moveAbsoluteZ(double dZ){
    location.moveZ(dZ);
    setFocus();
}
void Camera4D::moveAbsoluteA(double dA){
    location.moveA(dA);
    setFocus();
}

/** Relative Movement **/
void Camera4D::moveRelative(const std::vector<double>& dPosition){
    if (dPosition.size() == 3){
        moveRelative(dPosition[0], dPosition[1], dPosition[2], dPosition[3]);
    } else {
        std::cout << "Warning: Invalid input in:"
                     "\n\tvoid Camera4D::moveRelative(const std::vector<double>& dPosition)"
                     "\n\t(Camera4D.cpp)" << std::endl;
    }
}
void Camera4D::moveRelative(const spatialVector &dPosition){
    moveRelative(dPosition.components);
}
void Camera4D::moveRelative(double dF, double dR, double dU, double dO) {
    moveRelativeF(dF);
    moveRelativeR(dR);
    moveRelativeU(dU);
    moveRelativeO(dO);
}
void Camera4D::moveRelativeFB(double dF) {
    moveRelativeF(dF);
}
void Camera4D::moveRelativeRL(double dR) {
    moveRelativeR(dR);
}
void Camera4D::moveRelativeUD(double dU) {
    moveRelativeU(dU);
}
void Camera4D::moveRelativeOI(double dO) {
    moveRelativeO(dO);
}

void Camera4D::moveRelativeF(double dF){
    // Get vector in direction of current polarAngle and phiAngle only
    // (Locks movement to x/y/a dimensions)
    spatialVector dPositionVec = sphericalAngle4d(
        sphericalDirection.polarAngle,
        sphericalDirection.azimuthAngle,
        90
    ).getUnitVector();
    dPositionVec.scale(dF);
    moveAbsolute(dPositionVec);
}
void Camera4D::moveRelativeB(double dB){
    moveRelativeF(-dB);
}
void Camera4D::moveRelativeR(double dR) {
    spatialVector dPositionVec = getUnitRightVector();
    dPositionVec.scale(dR);
    moveAbsolute(dPositionVec);
}
void Camera4D::moveRelativeL(double dL){
    moveRelativeR(dL);
}
void Camera4D::moveRelativeU(double dU) {
    moveAbsoluteZ(dU);
}
void Camera4D::moveRelativeD(double dD){
    moveRelativeU(-dD);
}
void Camera4D::moveRelativeO(double dO){
    moveAbsoluteA(dO);
}
void Camera4D::moveRelativeI(double dI){
    moveRelativeO(-dI);
}

void Camera4D::moveRelativeDefaultF(){
    moveRelativeF(DEFAULT_MOVE_DISTANCE);
}
void Camera4D::moveRelativeDefaultB(){
    moveRelativeB(DEFAULT_MOVE_DISTANCE);
}
void Camera4D::moveRelativeDefaultR(){
    moveRelativeR(DEFAULT_MOVE_DISTANCE);
}
void Camera4D::moveRelativeDefaultL(){
    moveRelativeL(DEFAULT_MOVE_DISTANCE);
}
void Camera4D::moveRelativeDefaultU(){
    moveRelativeU(DEFAULT_MOVE_DISTANCE);
}
void Camera4D::moveRelativeDefaultD(){
    moveRelativeD(DEFAULT_MOVE_DISTANCE);
}
void Camera4D::moveRelativeDefaultO(){
    moveRelativeO(DEFAULT_MOVE_DISTANCE);
}
void Camera4D::moveRelativeDefaultI(){
    moveRelativeI(DEFAULT_MOVE_DISTANCE);
}

void Camera4D::moveFly() {
    updateVelocities();
    moveRelative(
        getVelocityFB(),
        getVelocityRL(),
        getVelocityUD(),
        getVelocityOI()
    );
}

void Camera4D::setAcceleration(int acceleratingFB_,
                               int acceleratingRL_,
                               int acceleratingUD_,
                               int acceleratingOI_){
    setAccelerationFB(acceleratingFB_);
    setAccelerationRL(acceleratingRL_);
    setAccelerationUD(acceleratingUD_);
    setAccelerationOI(acceleratingOI_);
}
void Camera4D::setAccelerationFB(int acceleratingFB_){
    if (-1 <= acceleratingFB_ and acceleratingFB_ <= 1){
        acceleratingFB = acceleratingFB_;
    } else {
        std::cout << "Warning: Invalid input (acceleratingFB_) in:"
                     "\n\tvoid Camera4D::setAccelerationFB(int acceleratingFB_)"
                     "\n\t(Camera4D.cpp)" << std::endl;
    }
}
void Camera4D::setAccelerationRL(int acceleratingRL_){
    if (-1 <= acceleratingRL_ and acceleratingRL_ <= 1){
        acceleratingRL = acceleratingRL_;
    } else {
        std::cout << "Warning: Invalid input (acceleratingRL_) in:"
                     "\n\tvoid Camera4D::setAccelerationRL(int acceleratingRL_)"
                     "\n\t(Camera4D.cpp)" << std::endl;
    }
}
void Camera4D::setAccelerationUD(int acceleratingUD_){
    if (-1 <= acceleratingUD_ and acceleratingUD_ <= 1){
        acceleratingUD = acceleratingUD_;
    } else {
        std::cout << "Warning: Invalid input (acceleratingUD_) in:"
                     "\n\tvoid Camera4D::setAccelerationUD(int acceleratingUD_)"
                     "\n\t(Camera4D.cpp)" << std::endl;
    }
}
void Camera4D::setAccelerationOI(int acceleratingOI_) {
    if (-1 <= acceleratingOI_ and acceleratingOI_ <= 1){
        acceleratingUD = acceleratingOI_;
    } else {
        std::cout << "Warning: Invalid input (acceleratingOI_) in:"
                     "\n\tvoid Camera4D::setAccelerationOI(int acceleratingOI_)"
                     "\n\t(Camera4D.cpp)" << std::endl;
    }
}
void Camera4D::setAccelerationDefaultF(){
    setAccelerationFB(1);
}
void Camera4D::setAccelerationDefaultB(){
    setAccelerationFB(-1);
}
void Camera4D::setAccelerationDefaultR(){
    setAccelerationRL(1);
}
void Camera4D::setAccelerationDefaultL(){
    setAccelerationRL(-1);
}
void Camera4D::setAccelerationDefaultU(){
    setAccelerationUD(1);
}
void Camera4D::setAccelerationDefaultD(){
    setAccelerationUD(-1);
}
void Camera4D::setAccelerationDefaultO(){
    setAccelerationOI(1);
}
void Camera4D::setAccelerationDefaultI(){
    setAccelerationOI(-1);
}

void Camera4D::updateVelocities(){
    // Update velocity values by one game tick.
    updateVelFB();
    updateVelRL();
    updateVelUD();
    updateVelOI();
}
void Camera4D::updateVelFB() {
    if (acceleratingFB != 0){
        if (getVelocityFB()/acceleratingFB >= 0){
            // Current velocity is 0, or trying to accelerate in same
            // direction as current velocity
            if (acceleratingFB > 0){
                // Positive acceleration until max speed is reached
                getVelocityFB() = std::min(
                        getVelocityFB() + FB_ACCEL
                            * Scene::SECONDS_PER_TICK,
                        FB_MAX_SPEED
                );
            } else {
                // Negative acceleration until min speed is reached
                getVelocityFB() = std::max(
                        getVelocityFB() - FB_ACCEL
                            * Scene::SECONDS_PER_TICK,
                        -FB_MAX_SPEED
                );
            }
        } else {
            // Accelerating against current velocity - apply brake instead
            applyBrakeFB();
        }
    } else {
        // Apply drag only if camera is not accelerating
        applyDragFB();
    }
}
void Camera4D::updateVelRL() {
    if (acceleratingRL != 0){
        if (getVelocityRL()/acceleratingRL >= 0){
            // Current velocity is 0, or trying to accelerate in same
            // direction as current velocity
            if (acceleratingRL > 0){
                // Positive acceleration until max speed is reached
                getVelocityRL() = std::min(
                        getVelocityRL() + RL_ACCEL
                            * Scene::SECONDS_PER_TICK,
                        RL_MAX_SPEED
                );
            } else {
                // Negative acceleration until min speed is reached
                getVelocityRL() = std::max(
                        getVelocityRL() - RL_ACCEL
                            * Scene::SECONDS_PER_TICK,
                        -RL_MAX_SPEED
                );
            }
        } else {
            // Accelerating against current velocity - apply brake instead
            applyBrakeRL();
        }
    } else {
        // Apply drag only if camera is not accelerating
        applyDragRL();
    }
}
void Camera4D::updateVelUD() {
    if (acceleratingUD != 0){
        if (getVelocityUD()/acceleratingUD >= 0){
            // Current velocity is 0, or trying to accelerate in same
            // direction as current velocity
            if (acceleratingUD > 0){
                // Positive acceleration until max speed is reached
                getVelocityUD() = std::min(
                        getVelocityUD() + UD_ACCEL
                            * Scene::SECONDS_PER_TICK,
                        UD_MAX_SPEED
                );
            } else {
                // Negative acceleration until min speed is reached
                getVelocityUD() = std::max(
                        getVelocityUD() - UD_ACCEL
                            * Scene::SECONDS_PER_TICK,
                        -UD_MAX_SPEED
                );
            }
        } else {
            // Accelerating against current velocity - apply brake instead
            applyBrakeUD();
        }
    } else {
        // Apply drag only if camera is not accelerating
        applyDragUD();
    }
}
void Camera4D::updateVelOI() {
    if (acceleratingOI != 0){
        if (getVelocityOI()/acceleratingOI >= 0){
            // Current velocity is 0, or trying to accelerate in same
            // direction as current velocity
            if (acceleratingOI > 0){
                // Positive acceleration until max speed is reached
                getVelocityOI() = std::min(
                        getVelocityOI() + OI_ACCEL
                            * Scene::SECONDS_PER_TICK,
                        OI_MAX_SPEED
                );
            } else {
                // Negative acceleration until min speed is reached
                getVelocityOI() = std::max(
                        getVelocityOI() - OI_ACCEL
                            * Scene::SECONDS_PER_TICK,
                        -OI_MAX_SPEED
                );
            }
        } else {
            // Accelerating against current velocity - apply brake instead
            applyBrakeOI();
        }
    } else {
        // Apply drag only if camera is not accelerating
        applyDragOI();
    }
}

void Camera4D::applyDrag(){
    applyDragFB();
    applyDragRL();
    applyDragUD();
    applyDragOI();
}
void Camera4D::applyDragFB(){
    if (getVelocityFB() != 0){
        // No drag if current velocity already 0
        if (getVelocityFB() > 0){
            // Drag down to 0
            getVelocityFB() = std::max(
                getVelocityFB() - FB_DRAG
                    * Scene::SECONDS_PER_TICK,
                0.
            );
        } else {
            // Drag up to 0
            getVelocityFB() = std::min(
                getVelocityFB() + FB_DRAG
                    * Scene::SECONDS_PER_TICK,
                0.
            );
        }
    }
}
void Camera4D::applyDragRL(){
    if (getVelocityRL() != 0){
        // No drag if current velocity already 0
        if (getVelocityRL() > 0){
            // Drag down to 0
            getVelocityRL() = std::max(
                getVelocityRL() - RL_DRAG
                    * Scene::SECONDS_PER_TICK,
                0.
            );
        } else {
            // Drag up to 0
            getVelocityRL() = std::min(
                getVelocityRL() + RL_DRAG
                    * Scene::SECONDS_PER_TICK,
                0.
            );
        }
    }
}
void Camera4D::applyDragUD(){
    if (getVelocityUD() != 0){
        // No drag if current velocity already 0
        if (getVelocityUD() > 0){
            // Drag down to 0
            getVelocityUD() = std::max(
                getVelocityUD() - UD_DRAG
                    * Scene::SECONDS_PER_TICK,
                0.
            );
        } else {
            // Drag up to 0
            getVelocityUD() = std::min(
                getVelocityUD() + UD_DRAG
                    * Scene::SECONDS_PER_TICK,
                0.
            );
        }
    }
}
void Camera4D::applyDragOI(){
    if (getVelocityOI() != 0){
        // No drag if current velocity already 0
        if (getVelocityOI() > 0){
            // Drag down to 0
            getVelocityOI() = std::max(
                getVelocityOI() - OI_DRAG
                    * Scene::SECONDS_PER_TICK,
                0.
            );
        } else {
            // Drag up to 0
            getVelocityOI() = std::min(
                getVelocityOI() + OI_DRAG
                    * Scene::SECONDS_PER_TICK,
                0.
            );
        }
    }
}

void Camera4D::applyBrakeFB(){
    if (getVelocityFB() > 0){
        // Break down to 0
        getVelocityFB() = std::max(
            getVelocityFB() - FB_BRAKE
                * Scene::SECONDS_PER_TICK,
            0.
        );

        if (getVelocityFB() == 0){
            // Reset acceleration when velocity gets to 0
            setAccelerationFB(0);
        }
    } else if (getVelocityFB() < 0){
        // Break up to 0
        getVelocityFB() = std::min(
            getVelocityFB() + FB_BRAKE
                * Scene::SECONDS_PER_TICK,
            0.
        );

        if (getVelocityFB() == 0){
            // Reset acceleration when velocity gets to 0
            setAccelerationFB(0);
        }
    } else {
        std::cout << "Warning: Breaking called when velocityFB == 0 in:"
                     "\n\tvoid Camera4D::applyBrakeFB()"
                     "\n\t(Camera4D.cpp)" << std::endl;
    }
}
void Camera4D::applyBrakeRL(){
    if (getVelocityRL() > 0){
        // Break down to 0
        getVelocityRL() = std::max(
            getVelocityRL() - RL_BRAKE
                * Scene::SECONDS_PER_TICK,
            0.
        );

        if (getVelocityRL() == 0){
            // Reset acceleration when velocity gets to 0
            setAccelerationRL(0);
        }
    } else if (getVelocityRL() < 0){
        // Break up to 0
        getVelocityRL() = std::min(
            getVelocityRL() + RL_BRAKE
                * Scene::SECONDS_PER_TICK,
            0.
        );

        if (getVelocityRL() == 0){
            // Reset acceleration when velocity gets to 0
            setAccelerationRL(0);
        }
    } else {
        std::cout << "Warning: Breaking called when velocityRL == 0 in:"
                     "\n\tvoid Camera4D::applyBrakeRL()"
                     "\n\t(Camera4D.cpp)" << std::endl;
    }
}
void Camera4D::applyBrakeUD(){
    if (getVelocityUD() > 0){
        // Break down to 0
        getVelocityUD() = std::max(
            getVelocityUD() - UD_BRAKE
                * Scene::SECONDS_PER_TICK,
            0.
        );

        if (getVelocityUD() == 0){
            // Reset acceleration when velocity gets to 0
            setAccelerationUD(0);
        }
    } else if (getVelocityUD() < 0){
        // Break up to 0
        getVelocityUD() = std::min(
            getVelocityUD() + UD_BRAKE
                * Scene::SECONDS_PER_TICK,
            0.
        );

        if (getVelocityUD() == 0){
            // Reset acceleration when velocity gets to 0
            setAccelerationUD(0);
        }
    } else {
        std::cout << "Warning: Breaking called when velocityUD == 0 in:"
                     "\n\tvoid Camera4D::applyBrakeUD()"
                     "\n\t(Camera4D.cpp)" << std::endl;
    }
}
void Camera4D::applyBrakeOI(){
    if (getVelocityOI() > 0){
        // Break down to 0
        getVelocityOI() = std::max(
            getVelocityOI() - OI_BRAKE
                * Scene::SECONDS_PER_TICK,
            0.
        );

        if (getVelocityOI() == 0){
            // Reset acceleration when velocity gets to 0
            setAccelerationOI(0);
        }
    } else if (getVelocityOI() < 0){
        // Break up to 0
        getVelocityOI() = std::min(
            getVelocityOI() + OI_BRAKE
                * Scene::SECONDS_PER_TICK,
            0.
        );

        if (getVelocityOI() == 0){
            // Reset acceleration when velocity gets to 0
            setAccelerationOI(0);
        }
    } else {
        std::cout << "Warning: Breaking called when velocityFB == 0 in:"
                     "\n\tvoid Camera4D::applyBrakeOI()"
                     "\n\t(Camera4D.cpp)" << std::endl;
    }
}

/** ---------- Rotation ---------- **/
void Camera4D::rotate(std::vector<double> dAngles){
    if (dAngles.size() == 3){
        rotate(dAngles[0], dAngles[1], dAngles[2]);
    } else {
        std::cout << "Warning: Invalid input in:"
                     "\n\tvoid Camera4D::rotate(std::vector<double> dAngles)"
                     "\n\t(Camera4D.cpp)" << std::endl;
    }
}
void Camera4D::rotate(const sphericalAngle4d &dAngles){
    rotate(dAngles.polarAngle, dAngles.azimuthAngle, dAngles.phiAngle);
}
void Camera4D::rotate(
            const double dPolarAngle,
            const double dAzimuthAngle,
            const double phiAngle){
    rotatePolar(dPolarAngle, false);
    rotateAzimuth(dAzimuthAngle, false);
    rotatePhi(phiAngle, false);
    setNormal();
}
void Camera4D::rotatePolar(const double dPolarAngle){
    rotatePolar(dPolarAngle, true);
}
void Camera4D::rotatePolar(const double dPolarAngle, bool updateNormal){
    sphericalDirection.rotatePolar(dPolarAngle);
    if (updateNormal){
        setNormal();
    }
}
void Camera4D::rotateAzimuth(const double dAzimuthAngle){
    rotateAzimuth(dAzimuthAngle, true);
}
void Camera4D::rotateAzimuth(const double dAzimuthAngle, bool updateNormal){
    sphericalDirection.rotateAzimuth(dAzimuthAngle);
    if (updateNormal){
        setNormal();
    }
}
void Camera4D::rotatePhi(const double dPhiAngle){
    rotatePhi(dPhiAngle, true);
}
void Camera4D::rotatePhi(const double dPhiAngle, bool updateNormal){
    sphericalDirection.rotatePhi(dPhiAngle);
    if (updateNormal){
        setNormal();
    }
}
void Camera4D::rotateRight(){
    rotatePolar(-DEFAULT_ROTATION_ANGLE);
}
void Camera4D::rotateLeft(){
    rotatePolar(DEFAULT_ROTATION_ANGLE);
}
void Camera4D::rotateUp(){
    rotateAzimuth(-DEFAULT_ROTATION_ANGLE);
}
void Camera4D::rotateDown(){
    rotateAzimuth(DEFAULT_ROTATION_ANGLE);
}
void Camera4D::rotateOut(){
    rotatePhi(-DEFAULT_ROTATION_ANGLE);
}
void Camera4D::rotateIn(){
    rotatePhi(DEFAULT_ROTATION_ANGLE);
}
