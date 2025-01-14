//
// Created by Jackson Hall on 4/29/2020.
//

#ifndef PART_2___GRAPHICS_ALTERNATE_CAMERA3D_H
#define PART_2___GRAPHICS_ALTERNATE_CAMERA3D_H

#include "Camera.h"


class Camera3D : public Camera {
public:
    /* ---------- Static Const Vars ---------- */
    ;
    static const double DEFAULT_FOV_DEGREES;
    static const double DEFAULT_PROJECTION_PLANE_WIDTH_BLOCKS;

    // Values used to maintain camera velocity
    ;
    static const double FB_ACCEL;
    static const double RL_ACCEL;
    static const double UD_ACCEL;
    static const double FB_DRAG;
    static const double RL_DRAG;
    static const double UD_DRAG;
    static const double FB_BRAKE;
    static const double RL_BRAKE;
    static const double UD_BRAKE;
    static const double FB_MAX_SPEED;
    static const double RL_MAX_SPEED;
    static const double UD_MAX_SPEED;

    /* ---------- Constructors ---------- */
    ;
    Camera3D();
    Camera3D(const Camera3D& other);
    Camera3D(const point3d& location,
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
            const double& velocityU);

    /* ---------- Getters ---------- */
    /* Const references */
    ;
    const point3d& getLocation() const;
    const spatialVector& getNormal() const;
    const sphericalAngle3d& getSphericalDirection() const;
    const point3d& getFocus() const;
    const double& getVelocityF() const;
    const double& getVelocityR() const;
    const double& getVelocityU() const;
    const double& getAccelerationF() const;
    const double& getAccelerationR() const;
    const double& getAccelerationU() const;

    /* Other **/
    /**
     * Return the unit vector that points upward relative to the camera normal.
     * In other words, return the vector that is a -90 deg azimuth rotation from
     * the normal (even past zenith/nadir).
     */
    spatialVector getUnitUpVector() const;
    /**
     * Return the unit vector that points rightward relative to the camera normal.
     * In other words, return the vector that is a -90 deg polar rotation from
     * the normal.
     */
    spatialVector getUnitRightVector() const;

    /** ---------- Setters ---------- **/
    void setLocation(std::vector<double> newLocation) override;
    void setLocation(const point3d& newLocation);
    void setLocation(double x, double y, double z);
    void setNormal() override;
    void setSphericalDirection(std::vector<double> newAngles) override;
    void setSphericalDirection(const sphericalAngle3d& newAngles);
    void setSphericalDirection(double polarAngle, double azimuthAngle);
    void setPolar(double polarAngle);
    void setAzimuth(double azimuthAngle);
    void setFocus() override;

    /* ---------- Utility / Other ---------- */
    optional<point2d> projectPoint(const point3d& p) const;

    /* ---------- Movement ---------- */
    /* Main API Calls */
    // Use appropriate movement methods (moveFly or moveFixed) to move
    // `location`, depending on movementMode
    void moveF();
    void moveB();
    void moveR();
    void moveL();
    void moveU();
    void moveD();

    bool isMoving() const;

    /* Absolute Movement **/
    // Move by given amounts along XYZ grid axes
    void moveAbsolute(std::vector<double> dPosition) override;
    void moveAbsolute(const spatialVector& dPosition) override;
    void moveAbsolute(double dX, double dY, double dZ);
    void moveAbsoluteX(double dX);
    void moveAbsoluteY(double dY);
    void moveAbsoluteZ(double dZ);

    /* Relative Movement **/
    // Move by amounts relative to orientation of camera (Camera
    // .DEFAULT_MOVE_DISTANCE by default)
    void moveRelative(const std::vector<double>& dPosition) override;
    void moveRelative(const spatialVector& dPosition) override;
    void moveRelative(double dF, double dR, double dU);
    void moveRelativeFB(double dF); // dF > 0 = forward, dF < 0 = backward
    void moveRelativeRL(double dR); // etc.
    void moveRelativeUD(double dU);
    /**
     * Move along polar angle (x/y plane only)
     */
    void moveRelativeF(double dF = DEFAULT_MOVE_DISTANCE);
    /**
     * Move opposite polar angel (x/y plane only)
     */
    void moveRelativeB(double dB = DEFAULT_MOVE_DISTANCE);
    /**
     * Move toward `getUnitRightVector()`
     */
    void moveRelativeR(double dR = DEFAULT_MOVE_DISTANCE);
    /**
     * Move opposite `getUnitRightVector()`
     */
    void moveRelativeL(double dL = DEFAULT_MOVE_DISTANCE);
    /**
     * Equivalent to `moveAbsoluteY(dU)`
     */
    void moveRelativeU(double dU = DEFAULT_MOVE_DISTANCE);
    /**
     * Equivalent to `moveAbsoluteY(-dD)`
     */
    void moveRelativeD(double dD = DEFAULT_MOVE_DISTANCE);

    /* ---------- Acceleration & Velocity Movement ---------- */
    // Main API call to update camera location using velocity-based controls
    ;
    void moveFly();

    // Set movingForward, movingRight, movingUp flags to given values, where
    // each must be -1, 0, or 1
    ;
    void setAcceleration(double newAccelerationF,
                         double newAccelerationR,
                         double newAccelerationU);
    void setAccelerationF(double newAccelerationF = 1);
    void setAccelerationR(double newAccelerationR = 1);
    void setAccelerationU(double newAccelerationU = 1);
    void setAccelerationB(double newAccelerationB = 1);
    void setAccelerationL(double newAccelerationL = 1);
    void setAccelerationD(double newAccelerationD = 1);

//    void setAccelerationF(int accelerationF);
//    void setAccelerationB(int accelerationB);
//    void setAccelerationR(int accelerationR);
//    void setAccelerationL(int accelerationL);
//    void setAccelerationU(int accelerationU);
//    void setAccelerationD(int accelerationD);

    // Update velocity values in given direction(s) relative to orientation of
    // camera by applying drag + any current acceleration
    ;
    void updateVelocities();
    void updateVelFB();
    void updateVelRL();
    void updateVelUD();

    // Reduce velocity (make closer to 0) in forward/back, right/left, and
    // up/down direction using drag constants declared above
    ;
    void applyDrag();
    void applyDragFB();
    void applyDragRL();
    void applyDragUD();

    // Accelerate against current velocity until velocity = 0 using braking
    // constants declared above
    ;
    void applyBrakeFB();
    void applyBrakeRL();
    void applyBrakeUD();

    /* ---------- Rotation ---------- */
    ;
    void rotate(std::vector<double> dAngles) override;
    void rotate(const sphericalAngle3d& dAngles);
    void rotate(double dPolarAngle, double dAzimuthAngle);
    void rotatePolar(double dPolarAngle);
    void rotatePolar(double dPolarAngle, bool updateNormal);
    void rotateAzimuth(double dAzimuthAngle);
    void rotateAzimuth(double dAzimuthAngle, bool updateNormal);

    void rotateRight(double dDeg = DEFAULT_ROTATION_ANGLE);
    void rotateLeft(double dDeg = DEFAULT_ROTATION_ANGLE);
    void rotateUp(double dDeg = DEFAULT_ROTATION_ANGLE);
    void rotateDown(double dDeg = DEFAULT_ROTATION_ANGLE);

protected:
    /** ---------- Fields ---------- **/
    // Width of the projection plane (viewable in-game area) in blocks.
    // Used to calculate focal distance given an FOV angle.
    double projectionPlaneWidthBlocks;

    // Location of the camera, the point around which the projection plane is
    // always centered
    point3d location;

    // Holds 1, -1, or 0 denoting positive, negative, or no acceleration
    // along forward/backward, right/left, and up/down directions on the
    // next game tick from orientation of the Camera (must then be
    // translated to get absolute x/y/z movement velocities)
    double accelerationF; // ex. 1 = towards direction of polarAngle
    double accelerationR; // ex. 1 = towards Camera.getUnitRightVec()
    double accelerationU; // ex. 1 = upwards

    // Store the <forward, right, up> velocities of the camera movement from
    // the orientation of the camera (not in absolute x/y/z directions). Ex.
    // a negative `right` value = moving left by that velocity.
    double velocityF;
    double velocityR;
    double velocityU;

    // Direction the camera is facing. Used to determine 2d plane of the
    // camera onto which 3d points get projected
    spatialVector normal;

    // A point P in the Scene is projected onto the plane of the
    // Scene's camera by finding the intersection point of [the line between
    // P and the Camera's focus] and the Camera's plane
    // Note: this means the focus is "behind" the direction the Camera sees
    point3d focus;

    // Direction the camera is facing in spherical coordinates. Used to
    // rotate the camera smoothly
    sphericalAngle3d sphericalDirection;

};


#endif //PART_2___GRAPHICS_ALTERNATE_CAMERA3D_H
