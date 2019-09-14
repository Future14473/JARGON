package org.futurerobotics.temporaryname.mechanics

import org.futurerobotics.temporaryname.math.squared
import kotlin.math.sqrt

/**
 * Represents a motion in [Double], with only velocity [v], and acceleration [a].
 * @param v velocity of this motion
 * @param a acceleration of this motion
 */
open class LinearMotion constructor(
    override val v: Double,
    override val a: Double = 0.0
) : Motion<Double> {

    /** Returns the new motion after time [t], assuming constant acceleration. */
    open fun afterTime(t: Double): LinearMotion =
        LinearMotion(v + a * t, a)

    /**
     * Returns the motion, the case with positive velocity, after moving a displacement
     * of [s] relative to the current motion.
     *
     * This may return a velocity of NaN if this state will never reach a displacement of [s]
     */
    open fun afterForwardDist(s: Double): LinearMotion =
        LinearMotion(sqrt(v.squared() + 2 * a * s), a)
}

/**
 * Represents a [State] in [Double], with position [s], velocity [v], and acceleration [a].
 * @param s position of this state
 * @param v velocity of this state
 * @param a acceleration of this state
 */
class LinearState constructor(override val s: Double, v: Double, a: Double = 0.0) : LinearMotion(v, a),
    State<Double> {

    /** Returns the new state after time [t], assuming constant acceleration. */
    override fun afterTime(t: Double): LinearState =
        LinearState(s + v * t + a * t.squared() / 2, v + a * t, a)

    /**
     * Returns the state, the case with positive velocity, after moving a displacement
     * of [s] relative to the current state.
     *
     * This may return a velocity of NaN if this state will never reach a displacement of [s]
     */
    override fun afterForwardDist(s: Double): LinearState =
        LinearState(this.s + s, sqrt(v.squared() + 2 * a * s), a)

    /**
     * Returns the state, the case with positive velocity, when this state reaches a displacement of [s] relative
     * to the global frame.
     *
     * This may return a velocity of NaN if this state will never reach a displacement of [s]
     */
    fun atDist(s: Double): LinearState =
        LinearState(s, sqrt(v.squared() + 2 * a * (s - this.s)), a)
}
