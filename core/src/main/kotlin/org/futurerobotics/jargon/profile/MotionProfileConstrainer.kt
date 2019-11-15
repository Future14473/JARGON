package org.futurerobotics.jargon.profile

import org.futurerobotics.jargon.math.Interval
import org.futurerobotics.jargon.util.Steppable
import org.futurerobotics.jargon.util.Stepper

/**
 * Represents the constraints used to generate a dynamic [MotionProfile], using [generateDynamicProfile].
 *
 *
 * Distance values along a _profiled path_ can be length or angle, or something else.
 *
 * This operates by returns constraints on velocity and acceleration (w/ respect to time) at specific points along
 *  the profiled path.
 * @see PointConstraint
 */
interface MotionProfileConstrainer : Steppable<Double, PointConstraint> {

    /**
     * Returns a [Stepper] that steps on distances along a profiled object and returns a [PointConstraint]
     * representing the constraints at that point.
     * @see [MotionProfileConstrainer]
     */
    override fun stepper(): Stepper<Double, PointConstraint>

    /**
     * Gets a list of required points that must be considered in the motion profile.
     */
    @JvmDefault
    val requiredPoints: Set<Double>
        get() = emptySet()
}

/**
 * Represents the motion profile constraints at a specific point along the
 *
 * This includes maximum velocity, and an [Interval] of allowable accelerations going along
 * a _profiled object_
 * @see [MotionProfileConstrainer]
 */
interface PointConstraint {

    /** The maximum velocity at this point on the profile */
    val maxVelocity: Double

    /**
     * A [Interval] of allowable accelerations at this point on the profile, both positive and negative, given
     * the [currentVelocity].
     * */
    fun accelRange(currentVelocity: Double): Interval
}

/**
 * A quick way to create a [PointConstraint], using the given [maxVelocity] value and [accelRange] function
 */
@Suppress("FunctionName")
inline fun PointConstraint(
    crossinline maxVelocity: () -> Double,
    crossinline accelRange: (currentVelocity: Double) -> Interval
): PointConstraint = object : PointConstraint {
    override val maxVelocity: Double = maxVelocity()
    override fun accelRange(currentVelocity: Double): Interval = accelRange(currentVelocity)
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

    override fun stepper(): Stepper<Double, PointConstraint> =
        Stepper {
            object : PointConstraint {
                override val maxVelocity: Double = getMaxVelocity(it)
                override fun accelRange(currentVelocity: Double): Interval = getMaxAccel(it, currentVelocity)
            }
        }
}
