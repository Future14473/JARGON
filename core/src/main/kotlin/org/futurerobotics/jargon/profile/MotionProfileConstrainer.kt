package org.futurerobotics.jargon.profile

import org.futurerobotics.jargon.math.Interval
import org.futurerobotics.jargon.util.Stepper

/**
 * Represents the constraints used to generate a dynamic [ForwardMotionProfile] using [generateDynamicProfile].
 *
 * This operates by returning constraints on velocity and acceleration at specific points along a path, where
 * the points, velocity, and acceleration are given by some notion of distance.
 *
 * @see PointConstraint
 */
interface MotionProfileConstrainer {

    /**
     * Returns a [Stepper] that steps on distances along a profiled path, and returns a [PointConstraint]
     * representing the constraints on both velocity and acceleration at that point.
     * @see MotionProfileConstrainer
     */
    fun stepper(): Stepper<PointConstraint>

    /**
     * Gets a set of required points that must be considered in the motion profile.
     */
    val requiredPoints: Set<Double>
        get() = emptySet()
}

/**
 * Represents the motion profile constraints at a specific point along a profiled path.
 *
 * This includes maximum velocity, and an [Interval] of allowable accelerations at this point.
 *
 * @see MotionProfileConstrainer
 */
interface PointConstraint {

    /** The maximum allowed velocity at this point on the profiled path. */
    val maxVelocity: Double

    /**
     * A [Interval] of allowable accelerations at this point on the profile, given the [currentVelocity].
     *
     * This may be called multiple times with different [currentVelocity]s.
     */
    fun accelRange(currentVelocity: Double): Interval
}

