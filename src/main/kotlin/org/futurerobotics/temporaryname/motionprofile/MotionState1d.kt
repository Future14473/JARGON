package org.futurerobotics.temporaryname.motionprofile

import org.futurerobotics.temporaryname.math.squared
import kotlin.math.sqrt


/**
 * Represents a state in 1-dimensional motion, with position [x], velocity [v], and acceleration [a].
 * @param x position of this MotionState
 * @param v velocity of this motion state
 * @param a acceleration of this motion state
 */
data class MotionState1d(val x: Double, val v: Double, val a: Double = 0.0) {
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
    fun afterForwardDist(x: Double): MotionState1d =//may return v of NaN if impossible
        MotionState1d(this.x + x, sqrt(v.squared() + 2 * a * x), a)

    override fun hashCode(): Nothing = throw UnsupportedOperationException()
}