package org.futurerobotics.jargon.math

import kotlin.math.pow
import kotlin.math.sqrt

/**
 * An implementation for [MotionOnly] with double values, also containing some useful physics calculations.
 */
class LinearMotionOnly
@JvmOverloads constructor(
    vel: Double,
    accel: Double = 0.0
) : ValueMotionOnly<Double>(vel, accel) {

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

    /** Adds component-wise. */
    operator fun plus(other: LinearMotionOnly): LinearMotionOnly =
        LinearMotionOnly(vel + other.vel, accel + other.accel)

    /** Subtracts component-wise. */
    operator fun minus(other: LinearMotionOnly): LinearMotionOnly =
        LinearMotionOnly(vel - other.vel, accel - other.accel)

    companion object {
        private const val serialVersionUID: Long = 2975183939284281640
    }
}

/**
 * An implementation for [MotionState] with double values, also containing some useful physics calculations.
 */
class LinearMotionState
@JvmOverloads constructor(
    value: Double,
    deriv: Double,
    secondDeriv: Double = 0.0
) : ValueMotionState<Double>(value, deriv, secondDeriv) {

    /** Returns the new state after time [t], assuming constant acceleration. */
    fun afterTime(t: Double): LinearMotionState =
        LinearMotionState(value + deriv * t + secondDeriv * t.pow(2) / 2, deriv + secondDeriv * t, secondDeriv)

    /**
     * Returns the new state after moving a displacement of [s] relative to the current motion, assuming constant
     * acceleration, giving the solution with positive velocity.
     *
     * This may return a velocity of NaN if this state will never reach a displacement of [s].
     */
    fun afterForwardDist(s: Double): LinearMotionState =
        LinearMotionState(value + s, sqrt(deriv.pow(2) + 2 * secondDeriv * s), secondDeriv)

    /**
     * Returns the new state when this state reaches a position of [s], assuming constant acceleration,
     * and giving the solution with positive velocity.
     *
     * This may return a velocity of NaN if this state will never reach a position of [s].
     */
    fun atDist(s: Double): LinearMotionState =
        LinearMotionState(s, sqrt(deriv.pow(2) + 2 * secondDeriv * (s - value)), secondDeriv)

    /** Adds component-wise. */
    operator fun plus(other: LinearMotionState): LinearMotionState =
        LinearMotionState(value + other.value, deriv + other.deriv, secondDeriv + other.secondDeriv)

    /** Subtracts component-wise. */
    operator fun minus(other: LinearMotionState): LinearMotionState =
        LinearMotionState(value - other.value, deriv - other.deriv, secondDeriv - other.secondDeriv)

    companion object {
        private const val serialVersionUID: Long = 6199743470386206427
    }
}