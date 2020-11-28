//
// Created by Jackson Hall on 4/27/2020.
//

#ifndef PART_2___GRAPHICS_ALTERNATE_CAMERA_H
#define PART_2___GRAPHICS_ALTERNATE_CAMERA_H

#include "utils.h"
#include <memory>
#include <vector>

class Camera {
public:
    /** ---------- Enums ---------- */
    enum CameraType {
        Camera3D,
        Camera4D
    };
    enum MovementMode {
        // Move by a fixed distance left/right/up/etc
        Fixed,
        // Use acceleration and velocity to glide around
        Fly
    };

    /** ---------- Static Const Vars ---------- */
    // Distance in blocks the camera moves
    static const double DEFAULT_MOVE_DISTANCE;

    // Angle in degrees the camera rotates by default
    static const double DEFAULT_ROTATION_ANGLE;

    // Values used to maintain camera velocity
    static const double FB_ACCEL;     // blocks/second^2
    static const double RL_ACCEL;
    static const double UD_ACCEL;
    static const double FB_DRAG;      // blocks/second^2
    static const double RL_DRAG;
    static const double UD_DRAG;
    static const double FB_BRAKE;     // blocks/second^2
    static const double RL_BRAKE;
    static const double UD_BREAK;
    static const double FB_MAX_SPEED; // blocks/second
    static const double RL_MAX_SPEED;
    static const double UD_MAX_SPEED;

    /** ---------- Constructors ---------- */
    explicit Camera(double focalDistance, MovementMode movementMode);

    /** ---------- Static Methods ---------- */
    static double getFocalDistanceFromFOV(double fovDegrees,
            double screenWidthBlocks);

    /** ---------- Getters ---------- */
    double getFocalDistance() const;
    MovementMode getMovementMode();

    /** ---------- Setters ---------- */
    /* Utility */
    void setFocalDistance(double newFocalDistance);
    void toggleMovementMode();

    virtual void setFocus() = 0;

    /* Movement */
    virtual void setLocation(std::vector<double> newLocation) = 0;

    /* Rotation */
    virtual void setNormal() = 0;
    virtual void setSphericalDirection(std::vector<double> newAngles) = 0;

    /** ---------- Other Methods ---------- */
    /* Movement */
    virtual void moveAbsolute(std::vector<double> dPosition) = 0;
    virtual void moveAbsolute(const spatialVector& dPosition) = 0;
    /**
     * Take a vector of doubles <forward, right, up> and move in given
     * directions relative to camera orientation. Ex. if 'forward'
     * value negative, move backward, etc.
     */
//    virtual void moveRelative(std::vector<double>& dPosition) = 0;
    /**
     * Take a spatialVector <forward, right, up> and move in given directions
     * relative to camera orientation. Ex. if 'forward' value negative, move
     * backward, etc.
     * @param dPosition
     */
//    virtual void moveRelative(const spatialVector& dPosition) = 0;

    /* Rotation */
    virtual void rotate(std::vector<double> dAngles) = 0;

protected:
    // Distance in the direction opposite the Camera's normal from its
    // coordinate location to the focus. Used to set the focus point
    double focalDistance{};

    // Movement mode for this Camera.
    // Fixed mode: Pressing movement keys moves Camera by a fixed distance
    //     each tick.
    // Fly mode: Pressing movement keys sets directional acceleration that is
    //     used to update velocity fields, which decay to 0 over time when keys
    //     are released. See public static constants above.
    MovementMode movementMode;
};


#endif //PART_2___GRAPHICS_ALTERNATE_CAMERA_H
