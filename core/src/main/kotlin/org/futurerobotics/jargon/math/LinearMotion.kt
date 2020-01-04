package org.futurerobotics.jargon.math

import kotlin.math.pow
import kotlin.math.sqrt

/**
 * An [MotionOnly] with double values for velocity ([v]) and acceleration ([a]),
 * also containing some useful calculations for one-dimensional motion.
 *
 * @property v velocity
 * @property a acceleration
 */
@Suppress("OVERRIDE_BY_INLINE")
class LinearMotionOnly
@JvmOverloads constructor(
    @JvmField val v: Double,
    @JvmField val a: Double = 0.0
) : AnyMotionOnly<Double> {

    override val deriv: Double inline get() = v
    override val secondDeriv: Double inline get() = a

    /**
     * Returns the new motion after time [t], assuming constant acceleration.
     */
    fun afterTime(t: Double): LinearMotionOnly =
        LinearMotionOnly(v + a * t, a)

    /**
     * Returns the new state after moving a displacement of [s] relative to the current motion, assuming constant
     * acceleration, giving the solution with positive velocity.
     *
     * This may return a velocity of NaN if this state will never reach a displacement of [s].
     */
    fun afterForwardDist(s: Double): LinearMotionOnly =
        LinearMotionOnly(sqrt(v.pow(2) + 2 * a * s), a)

    /** Adds component-wise. */
    operator fun plus(other: LinearMotionOnly): LinearMotionOnly =
        LinearMotionOnly(v + other.v, a + other.a)

    /** Subtracts component-wise. */
    operator fun minus(other: LinearMotionOnly): LinearMotionOnly =
        LinearMotionOnly(v - other.v, a - other.a)
}

/**
 * [MotionState] with double values for value([s]), velocity ([v]), and acceleration ([a]),
 * also containing some useful calculations for one-dimensional motion.
 *
 * @property s value
 * @property v velocity
 * @property a acceleration
 */
@Suppress("OVERRIDE_BY_INLINE")
class LinearMotionState
@JvmOverloads constructor(
    @JvmField val s: Double,
    @JvmField val v: Double,
    @JvmField val a: Double = 0.0
) : AnyMotionState<Double> {

    override val value: Double inline get() = s
    override val deriv: Double inline get() = v
    override val secondDeriv: Double inline get() = a

    /** Returns the new state after time [t], assuming constant acceleration. */
    fun afterTime(t: Double): LinearMotionState =
        LinearMotionState(s + v * t + a * t.pow(2) / 2, v + a * t, a)

    /**
     * Returns the new state after moving a displacement of [s] relative to the current motion, assuming constant
     * acceleration, giving the solution with positive velocity.
     *
     * This may return a velocity of NaN if this state will never reach a displacement of [s].
     */
    fun afterForwardDistance(s: Double): LinearMotionState =
        LinearMotionState(this.s + s, sqrt(v.pow(2) + 2 * a * s), a)

    /**
     * Returns the new state when this state reaches a position of [s], assuming constant acceleration,
     * and giving the solution with positive velocity.
     *
     * This may return a velocity of NaN if this state will never reach a position of [s].
     */
    fun atDistance(s: Double): LinearMotionState =
        LinearMotionState(s, sqrt(v.pow(2) + 2 * a * (s - this.s)), a)

    /**
     * Returns the time elapsed to get to the specified [s] distance, favoring the solution in the direction
     * of velocity. This may return NaN if it will never reach that distance.
     */
    fun timeElapsedAtDistance(s: Double): Double =
        if (a epsEq 0.0) (s - this.s) / v else
            (-v + sqrt(v.pow(2) + 2 * a * (s - this.s))) / a

    /** Adds component-wise. */
    operator fun plus(other: LinearMotionState): LinearMotionState =
        LinearMotionState(s + other.s, v + other.v, a + other.a)

    /** Subtracts component-wise. */
    operator fun minus(other: LinearMotionState): LinearMotionState =
        LinearMotionState(s - other.s, v - other.v, a - other.a)
}
