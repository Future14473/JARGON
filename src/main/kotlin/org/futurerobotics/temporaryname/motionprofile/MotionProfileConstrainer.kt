package org.futurerobotics.temporaryname.motionprofile

import org.futurerobotics.temporaryname.math.Interval
import org.futurerobotics.temporaryname.util.Steppable
import org.futurerobotics.temporaryname.util.Stepper

/**
 * Represents the constraints to be used to generate a dynamic [MotionProfiled] using [MotionProfileGenerator].
 * @see PointConstraint
 */
interface MotionProfileConstrainer : Steppable<Double, PointConstraint> {
    /**
     * Returns a stepper that steps along distances from the start along the profiled object and returns a [PointConstraint]
     * representing the constraints at that point.
     */
    override fun stepper(): Stepper<Double, PointConstraint>
}

/**
 * Represents the constraints at a specific point along a motion to be profiled.
 */
interface PointConstraint {
    /** The maximum velocity at this point on the profile */
    val maxVelocity: Double

    /** The range of allowable accelerations at this point on the profile */
    fun accelRange(curVelocity: Double): Interval
}

/**
 * A quick way to create a [PointConstraint], using the given [maxVelocity] and [accelRange]
 */
@Suppress("FunctionName")
inline fun PointConstraint(
    maxVelocity: Double,
    crossinline accelRange: (curVelocity: Double) -> Interval
): PointConstraint =
    object : PointConstraint {
        override val maxVelocity: Double = maxVelocity

        override fun accelRange(curVelocity: Double): Interval = accelRange(curVelocity)
    }

/**
 * A partial implementation of a ProfileConstraint that simply divides up into [getMaxVelocity] and [getMaxAccel],
 * without using stepper optimizations.
 */
abstract class ComponentsMotionProfileConstrainer : MotionProfileConstrainer {
    /** Gets the maximum allowable velocity at the point [x] units from the start. */
    abstract fun getMaxVelocity(x: Double): Double

    /**
     * Gets a range of allowable acceleration at the point [x] units from the start, given [curVelocity].
     */
    abstract fun getMaxAccel(x: Double, curVelocity: Double): Interval

    final override fun stepper(): Stepper<Double, PointConstraint> = Stepper {
        object : PointConstraint {
            override val maxVelocity: Double = getMaxVelocity(it)
            override fun accelRange(curVelocity: Double): Interval = getMaxAccel(it, curVelocity)
        }
    }
}
