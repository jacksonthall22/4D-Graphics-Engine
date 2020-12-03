//
// Created by Jackson Hall on 4/29/2020.
//

#ifndef PART_2___GRAPHICS_ALTERNATE_CAMERA3D_H
#define PART_2___GRAPHICS_ALTERNATE_CAMERA3D_H

#include "Camera.h"


class Camera3D : public Camera {
public:
    /** Static Const Vars **/
    static const double DEFAULT_FOV_DEGREES;
    static const double DEFAULT_PROJECTION_PLANE_WIDTH_BLOCKS;

    // Values used to maintain camera velocity
    static const double FB_ACCEL;     // blocks/second^2
    static const double RL_ACCEL;
    static const double UD_ACCEL;
    static const double FB_DRAG;      // blocks/second^2
    static const double RL_DRAG;
    static const double UD_DRAG;
    static const double FB_BRAKE;     // blocks/second^2
    static const double RL_BRAKE;
    static const double UD_BRAKE;
    static const double FB_MAX_SPEED; // blocks/second
    static const double RL_MAX_SPEED;
    static const double UD_MAX_SPEED;

    /** ---------- Constructors ---------- **/
    Camera3D();
    Camera3D(const Camera3D& other);
    Camera3D(const point3d& location,
            const spatialVector& normal,
            const sphericalAngle3d& sphericalDirection,
            const point3d& focus,
            const double& focalDistance,
            const double& projectionPlaneWidthBlocks,
            Camera::MovementMode movementMode,
            const int& acceleratingFB,
            const int& acceleratingRL,
            const int& acceleratingUD,
            const spatialVector& velocityFRU);

    /** ---------- Getters ---------- **/
    /** Const references **/
    const point3d& getLocation() const;
    const spatialVector& getNormal() const;
    const sphericalAngle3d& getSphericalDirection() const;
    const point3d& getFocus() const;

    /** Non-const references **/
    point3d& getLocation();
    spatialVector& getNormal();
    sphericalAngle3d& getSphericalDirection();
    point3d& getFocus();
    int& getAcceleratingFB();
    int& getAcceleratingRL();
    int& getAcceleratingUD();
    double& getVelocityFB();
    double& getVelocityRL();
    double& getVelocityUD();

    /** Values **/
    int getAcceleratingFB() const;
    int getAcceleratingRL() const;
    int getAcceleratingUD() const;
    double getVelocityFB() const;
    double getVelocityRL() const;
    double getVelocityUD() const;

    /** Other **/
    spatialVector getUnitUpVector() const;
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

    /** ---------- Utility ---------- **/
    optional<point2d> projectPoint(const point3d& p) const;

    /** ---------- Movement ---------- **/

    /** Main API Calls **/
    // Use appropriate movement methods (moveFly or moveFixed) to move
    // `location`, depending on movementMode
    void moveR();
    void moveL();
    void moveU();
    void moveD();
    void moveF();
    void moveB();

    /** Utility **/
    bool isMoving() const;

    /** Absolute Movement **/
    // Move by given amounts along XYZ grid axes
    void moveAbsolute(std::vector<double> dPosition) override;
    void moveAbsolute(const spatialVector& dPosition) override;
    void moveAbsolute(double dX, double dY, double dZ);
    void moveAbsoluteX(double dX);
    void moveAbsoluteY(double dY);
    void moveAbsoluteZ(double dZ);

    /** Relative Movement **/
    // Move by amounts relative to orientation of camera
    void moveRelative(const std::vector<double>& dPosition)/* override*/;
    void moveRelative(const spatialVector& dPosition)/* override*/;
    void moveRelative(double dF, double dR, double dU);
    void moveRelativeFB(double dF); // dF > 0 = forward, dF < 0 = backward
    void moveRelativeRL(double dR); // etc.
    void moveRelativeUD(double dU);
    void moveRelativeF(double dF);  // Move along polar angle (x/y plane only)
    void moveRelativeB(double dB);  // Move opposite polar angel (x/y plane only)
    void moveRelativeR(double dR);  // Move toward getUnitRightVector()
    void moveRelativeL(double dL);  // Move opposite getUnitRightVector()
    void moveRelativeU(double dU);  // Equivalent to moveAbsolute(dU)
    void moveRelativeD(double dD);  // Equivalent to moveAbsolute(dU)

    // Move by Camera.DEFAULT_MOVE_DISTANCE relative to orientation of camera
    void moveRelativeDefaultF(); // Same as moveRelativeF(DEFAULT_MOVE_DISTANCE)
    void moveRelativeDefaultB(); // etc.
    void moveRelativeDefaultR();
    void moveRelativeDefaultL();
    void moveRelativeDefaultU();
    void moveRelativeDefaultD();

    /** Acceleration & Velocity Movement **/
    // Main API call to update camera location using velocity-based controls
    void moveFly();

    // Set movingForward, movingRight, movingUp flags to given values, where
    // each must be -1, 0, or 1
    void setAcceleration(int acceleratingFB_,
                         int acceleratingRL_,
                         int acceleratingUD_);
    void setAccelerationFB(int acceleratingFB_);
    void setAccelerationRL(int acceleratingRL_);
    void setAccelerationUD(int acceleratingUD_);
    void setAccelerationDefaultF(); // setAccelerationFB(1)
    void setAccelerationDefaultB(); // setAccelerationFB(-1)
    void setAccelerationDefaultR(); // setAccelerationRL(1)
    void setAccelerationDefaultL(); // etc.
    void setAccelerationDefaultU();
    void setAccelerationDefaultD();

    // Update velocity values in given direction(s) relative to orientation of
    // camera by applying drag + any current acceleration
    void updateVelocities();
    void updateVelFB();
    void updateVelRL();
    void updateVelUD();

    // Reduce velocity (make closer to 0) in forward/back, right/left, and
    // up/down direction using drag constants declared above
    void applyDrag();
    void applyDragFB();
    void applyDragRL();
    void applyDragUD();

    // Accelerate against current velocity until velocity = 0 using braking
    // constants declared above
    void applyBrakeFB();
    void applyBrakeRL();
    void applyBrakeUD();

    /** ---------- Rotation ---------- **/
    void rotate(std::vector<double> dAngles) override;
    void rotate(const sphericalAngle3d& dAngles);
    void rotate(double dPolarAngle, double dAzimuthAngle);
    void rotatePolar(double dPolarAngle);
    void rotatePolar(double dPolarAngle, bool updateNormal);
    void rotateAzimuth(double dAzimuthAngle);
    void rotateAzimuth(double dAzimuthAngle, bool updateNormal);
    void rotateRight();
    void rotateLeft();
    void rotateUp();
    void rotateDown();

protected:
    /** ---------- Fields ---------- **/
    // Width of the projection plane (viewable in-game area) in blocks.
    // Used to calculate focal distance given an FOV angle.
    double projectionPlaneWidthBlocks;

    // Location of the camera, the point on which the projection plane
    // always remains centered
    point3d location;

    // Holds 1, -1, or 0 denoting positive, negative, or no movement
    // (acceleration) along forward/backward, right/left, and up/down
    // directions on the next game tick from orientation of the Camera (must
    // then be translated to get absolute x/y/z movement velocities)
    int acceleratingFB; // ex. 1 = towards direction of polarAngle
    int acceleratingRL; // ex. 1 = towards Camera.getUnitRightVec()
    int acceleratingUD; // ex. 1 = upwards

    // Store the <forward, right, up> velocities of the camera movement from
    // the orientation of the camera (not in absolute x/y/z directions). Ex.
    // a negative `right` value = moving left by that velocity.
    spatialVector velocityFRU;

    // Direction the camera is facing. Used to determine 2d plane/3d
    // hyperplane of the camera onto which 3d/4d points get projected
    spatialVector normal;

    // A point P in the Scene is projected onto the plane of the
    // Scene's camera by finding the intersection point of the line between
    // P and the Camera's focus and the Camera's plane
    // Note: this means the focus is "behind" the direction the Camera sees
    point3d focus;

    // Direction the camera is facing in spherical coordinates. Used to
    // rotate the camera smoothly
    sphericalAngle3d sphericalDirection;

};


#endif //PART_2___GRAPHICS_ALTERNATE_CAMERA3D_H
