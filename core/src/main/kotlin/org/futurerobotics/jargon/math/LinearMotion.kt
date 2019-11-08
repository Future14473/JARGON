package org.futurerobotics.jargon.math

import java.io.Serializable
import kotlin.math.pow
import kotlin.math.sqrt

/**
 * An implementation for [MotionOnly] with double values, also containing some useful physics calculations.
 */
class LinearMotionOnly
@JvmOverloads constructor(
    override val vel: Double,
    override val accel: Double = 0.0
) : MotionOnly<Double> {

    /**
     * Returns the new motion after time [t], assuming constant acceleration.
     */
    fun afterTime(t: Double): LinearMotionOnly =
        LinearMotionOnly(vel + accel * t, accel)

    /**
     * Returns the new state after moving a displacement of [s] relative to the current motion, assuming constant
     * acceleration, giving the solution with positive velocity.
     *
     * This may return a velocity of NaN if this state will never reach a displacement of [s].
     */
    fun afterForwardDist(s: Double): LinearMotionOnly =
        LinearMotionOnly(sqrt(vel.pow(2) + 2 * accel * s), accel)

    companion object {
        private const val serialVersionUID = 2975183939284281640
    }
}

/**
 * An implementation for [MotionState] with double values, also containing some useful physics calculations.
 */
class LinearMotionState
@JvmOverloads constructor(
    override val value: Double,
    override val vel: Double,
    override val accel: Double = 0.0
) : MotionState<Double>, Serializable {

    /** Returns the new state after time [t], assuming constant acceleration. */
    fun afterTime(t: Double): LinearMotionState =
        LinearMotionState(value + vel * t + accel * t.pow(2) / 2, vel + accel * t, accel)

    /**
     * Returns the new state after moving a displacement of [s] relative to the current motion, assuming constant
     * acceleration, giving the solution with positive velocity.
     *
     * This may return a velocity of NaN if this state will never reach a displacement of [s].
     */
    fun afterForwardDist(s: Double): LinearMotionState =
        LinearMotionState(value + s, sqrt(vel.pow(2) + 2 * accel * s), accel)

    /**
     * Returns the new state when this state reaches a position of [s], assuming constant acceleration,
     * and giving the solution with positive velocity.
     *
     * This may return a velocity of NaN if this state will never reach a position of [s].
     */
    fun atDist(s: Double): LinearMotionState =
        LinearMotionState(s, sqrt(vel.pow(2) + 2 * accel * (s - value)), accel)

    companion object {
        private const val serialVersionUID = 6199743470386206427
    }
}
