package org.futurerobotics.jargon.mechanics

import java.io.Serializable
import kotlin.math.pow
import kotlin.math.sqrt

/**
 * Represents a [MotionOnly] of double values, with only velocity [v], and acceleration [a],
 * containing some useful physics calculations.
 * @param v velocity of this motion
 * @param a acceleration of this motion
 */
class LinearMotion
@JvmOverloads constructor(
    override val v: Double,
    override val a: Double = 0.0
) : MotionOnly<Double>, Serializable {

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
        LinearMotion(sqrt(v.pow(2) + 2 * a * s), a)

    companion object {
        private const val serialVersionUID = 2975183939284281640
    }
}

/**
 * Represents a [MotionOnly] of double values, with only velocity [v], and acceleration [a],
 * containing some useful physics calculations.
 * @param s position of this state
 * @param v velocity of this state
 * @param a acceleration of this state
 */
class LinearMotionState
@JvmOverloads constructor(
    override val s: Double,
    override val v: Double,
    override val a: Double = 0.0
) : MotionState<Double>, Serializable {

    /** Returns the new state after time [t], assuming constant acceleration. */
    fun afterTime(t: Double): LinearMotionState =
        LinearMotionState(s + v * t + a * t.pow(2) / 2, v + a * t, a)

    /**
     * Returns the state, the case with positive velocity, after moving a displacement
     * of [s] relative to the current state.
     *
     * This may return a velocity of NaN if this state will never reach a displacement of [s] relative to here.
     */
    fun afterForwardDist(s: Double): LinearMotionState =
        LinearMotionState(this.s + s, sqrt(v.pow(2) + 2 * a * s), a)

    /**
     * Returns the state, the case with positive velocity, when this state reaches a position of [s].
     *
     * This may return a velocity of NaN if this state will never reach a position of [s]
     */
    fun atDist(s: Double): LinearMotionState =
        LinearMotionState(s, sqrt(v.pow(2) + 2 * a * (s - this.s)), a)

    companion object {
        private const val serialVersionUID = 6199743470386206427
    }
}
