package org.futurerobotics.temporaryname.motionprofile

import org.futurerobotics.temporaryname.math.Pose2d
import org.futurerobotics.temporaryname.math.squared
import kotlin.math.sqrt

/**
 * Represents a state in 1-dimensional motion, with position [x], velocity [v], and acceleration [a].
 * @param x position of this state
 * @param v velocity of this state
 * @param a acceleration of this state
 */
class MotionState1d(val x: Double, val v: Double, val a: Double = 0.0) {
    /**
     * Returns the motion state if this (if having constant acceleration) after time [t]
     */
    fun afterTime(t: Double): MotionState1d = MotionState1d(
        x + v * t + a * t.squared() / 2, v + a * t, a
    )

    /**
     * Returns the motion state, the case with positive velocity, after moving a displacement of  [x]
     * This may return a velocity of NaN if this state will never reach a displacement of [x]
     */
    fun afterForwardDist(x: Double): MotionState1d = //may return v of NaN if impossible
        MotionState1d(this.x + x, sqrt(v.squared() + 2 * a * x), a)
}

/**
 * Represents motion in 1-dimensional motion, with only velocity [v], and acceleration [a].
 * @param v velocity of this motion
 * @param a acceleration of this motion
 */
class Motion1d(val v: Double, val a: Double = 0.0) {
    /**
     * Returns the motion state if this (if having constant acceleration) after time [t]
     */
    fun afterTime(t: Double): Motion1d = Motion1d(v + a * t, a)

    /**
     * Returns the motion state, the case with positive velocity, after moving a displacement of [x]
     *
     * This may return a velocity of NaN if this motion will never reach a displacement of [x]
     */
    fun afterForwardDist(x: Double): Motion1d = //may return v of NaN if impossible
        Motion1d(sqrt(v.squared() + 2 * a * x), a)
}

/**
 * Represents a motion in poses; e.g. robot motion on a field.
 *
 * @property pose The current position/orientation
 * @property vel The current velocity, both translational and rotational
 * @property accel The current acceleration, both translational and rotational
 */
class PoseMotionState(val pose: Pose2d, val vel: Pose2d, val accel: Pose2d = Pose2d.ZERO) {
    /**
     * Returns the motion state if this (if having constant acceleration) after time [t]
     */
    fun afterTime(t: Double): PoseMotionState = PoseMotionState(
        pose + vel * t + accel * (t.squared() / 2), vel + accel * t, accel
    )
}
