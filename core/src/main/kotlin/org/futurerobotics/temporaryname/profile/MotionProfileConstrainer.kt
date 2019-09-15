package org.futurerobotics.temporaryname.profile

import org.futurerobotics.temporaryname.math.Interval
import org.futurerobotics.temporaryname.util.Steppable
import org.futurerobotics.temporaryname.util.Stepper

/**
 * Represents the constrainer to use to generate a dynamic [MotionProfile] using [MotionProfileGenerator].
 * @see PointConstraint
 */
interface MotionProfileConstrainer : Steppable<Double, PointConstraint> {

    /**
     * Returns a [Stepper] that steps on distances along the profiled object and returns a [PointConstraint]
     * representing the constraints at that point.
     */
    override fun stepper(): Stepper<Double, PointConstraint>
}

/**
 * Represents the constraints at a specific point to be motion profiled.
 *
 * This includes maximum Velocity, and a range of allowable accelerations.
 */
interface PointConstraint {

    /** The maximum velocity at this point on the profile */
    val maxVelocity: Double

    /** The range of allowable accelerations at this point on the profile */
    fun accelRange(curVelocity: Double): Interval
}

/**
 * A quick way to create a [PointConstraint], using the given [maxVelocity] value and [accelRange] function
 */
@Suppress("FunctionName")
inline fun PointConstraint(
    maxVelocity: Double,
    crossinline accelRange: (curVelocity: Double) -> Interval
): PointConstraint = object : PointConstraint {
    override val maxVelocity: Double = maxVelocity
    override fun accelRange(curVelocity: Double): Interval = accelRange(curVelocity)
}

/**
 * A partial implementation of a ProfileConstraint that simply divides up velocity and acceleration
 * components of the constraint into [getMaxVelocity] and [getMaxAccel] functions, without stepper optimizations.
 */
abstract class ComponentsMotionProfileConstrainer : MotionProfileConstrainer {

    /** Gets the maximum allowable velocity at the point [x] units from the start. */
    abstract fun getMaxVelocity(x: Double): Double

    /**
     * Gets a range of allowable acceleration at the point [x] units from the start, given [curVelocity].
     */
    abstract fun getMaxAccel(x: Double, curVelocity: Double): Interval

    override fun stepper(): Stepper<Double, PointConstraint> =
        Stepper {
            PointConstraint(getMaxVelocity(it)) { vel -> getMaxAccel(it, vel) }
        }
}
