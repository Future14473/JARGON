package org.futurerobotics.jargon.profile

import org.futurerobotics.jargon.math.Interval
import org.futurerobotics.jargon.util.Steppable
import org.futurerobotics.jargon.util.Stepper

/**
 * Represents the constraints used to generate a dynamic [ForwardMotionProfile], using [generateDynamicProfile].
 *
 * This operates by returning constraints on velocity and acceleration at specific points along a path, given
 * by some notion of distance.
 *
 * @see PointConstraint
 */
interface MotionProfileConstrainer : Steppable<PointConstraint> {

    /**
     * Returns a [Stepper] that steps on distances along a profiled path, and returns a [PointConstraint]
     * representing the constraints at that point.
     * @see MotionProfileConstrainer
     */
    override fun stepper(): Stepper<PointConstraint>

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

/**
 * A partial implementation of a ProfileConstraint that simply divides up velocity and acceleration
 * components of the constraint into [getMaxVelocity] and [getMaxAccel] functions, without stepper optimizations.
 */
abstract class ComponentsMotionProfileConstrainer : MotionProfileConstrainer {

    /** Gets the maximum allowable velocity at the point [x] units from the start. */
    abstract fun getMaxVelocity(x: Double): Double

    /** Gets a range of allowable acceleration at the point [x] units from the start, given the [curVelocity]. */
    abstract fun getMaxAccel(x: Double, curVelocity: Double): Interval

    override fun stepper(): Stepper<PointConstraint> =
        Stepper {
            object : PointConstraint {
                override val maxVelocity: Double = getMaxVelocity(it)
                override fun accelRange(currentVelocity: Double): Interval = getMaxAccel(it, currentVelocity)
            }
        }
}
