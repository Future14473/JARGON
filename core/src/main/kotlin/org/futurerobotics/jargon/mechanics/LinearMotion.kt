package org.futurerobotics.jargon.mechanics

import org.futurerobotics.jargon.math.squared
import kotlin.math.sqrt

/**
 * Represents a [Motion} in [Double], with only velocity [v], and acceleration [a], and other
 * useful physics calculations.
 * @param v velocity of this motion
 * @param a acceleration of this motion
 */
class LinearMotion
@JvmOverloads constructor(
    override val v: Double,
    override val a: Double = 0.0
) : MotionOnly<Double> {

    /**
     * Returns the new motion after time [t], assuming constant acceleration.
     */
    fun afterTime(t: Double): LinearMotion =
        LinearMotion(v + a * t, a)

    /**
     * Returns the motion, the case with positive velocity, after moving a displacement
     * of [s] relative to the current motion.
     *
     * This may return a velocity of NaN if this state will never reach a displacement of [s]
     */
    fun afterForwardDist(s: Double): LinearMotion =
        LinearMotion(sqrt(v.squared() + 2 * a * s), a)
}

/**
 * Represents a [MotionState3] in [Double], with position [s], velocity [v], and acceleration [a].
 * @param s position of this state
 * @param v velocity of this state
 * @param a acceleration of this state
 */
class LinearMotionState3
@JvmOverloads constructor(
    override val s: Double,
    override val v: Double,
    override val a: Double = 0.0
) : MotionState3<Double> {

    /** Returns the new state after time [t], assuming constant acceleration. */
    fun afterTime(t: Double): LinearMotionState3 =
        LinearMotionState3(s + v * t + a * t.squared() / 2, v + a * t, a)

    /**
     * Returns the state, the case with positive velocity, after moving a displacement
     * of [s] relative to the current state.
     *
     * This may return a velocity of NaN if this state will never reach a displacement of [s] relative to here.
     */
    fun afterForwardDist(s: Double): LinearMotionState3 =
        LinearMotionState3(this.s + s, sqrt(v.squared() + 2 * a * s), a)

    /**
     * Returns the state, the case with positive velocity, when this state reaches a position of [s].
     *
     * This may return a velocity of NaN if this state will never reach a position of [s]
     */
    fun atDist(s: Double): LinearMotionState3 =
        LinearMotionState3(s, sqrt(v.squared() + 2 * a * (s - this.s)), a)
}
