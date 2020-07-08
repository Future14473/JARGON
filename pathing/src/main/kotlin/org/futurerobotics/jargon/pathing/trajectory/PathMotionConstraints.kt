package org.futurerobotics.jargon.pathing.trajectory

import org.futurerobotics.jargon.math.Interval
import org.futurerobotics.jargon.math.ifNan
import org.futurerobotics.jargon.pathing.PathPoint
import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.sqrt

/**
 * A constraint on [max] _tangential_ velocity on a path.
 */
class MaxTangentVelocity(max: Double) : MaxBasedConstraint(max), VelocityConstraint {

    override fun maxVelocity(point: PathPoint): Double = max
}

/**
 * A constraint on [max] _tangential_ acceleration on a path.
 * @see MaxCentripetalAccel
 * @see MaxTotalAcceleration
 */
class MaxTangentAcceleration(max: Double) : MaxBasedConstraint(max), AccelerationConstraint {

    private val interval = Interval.symmetric(max)

    override fun accelRange(point: PathPoint, curVelocity: Double): Interval = interval
}

/**
 * A constraint on [max] _centripetal_ acceleration on a path.
 *
 * Centripetal acceleration is always perpendicular to the direction of motion, and tangential acceleration.
 *
 * Since centripetal acceleration is only dependent on velocity and curvature, this is actually a [VelocityConstraint].
 * @see MaxTangentAcceleration
 * @see MaxTotalAcceleration
 */
class MaxCentripetalAccel(max: Double) : MaxBasedConstraint(max), VelocityConstraint {

    //a_c = v^2*c <= max; v <= sqrt(max/c)
    override fun maxVelocity(point: PathPoint): Double = sqrt(abs(max / point.tanAngleDeriv))
}

/**
 * A constraint on [max] _total_ acceleration on a path (tangential and centripetal combined).
 * @see MaxTangentAcceleration
 * @see MaxCentripetalAccel
 */
class MaxTotalAcceleration(max: Double) : MultipleConstraint {

    override val velocityConstraints: Collection<VelocityConstraint> = listOf(MaxCentripetalAccel(max))
    override val accelerationConstraints: Collection<AccelerationConstraint> = listOf(TheAccelerationConstraint(max))

    private class TheAccelerationConstraint(max: Double) : MaxBasedConstraint(max), AccelerationConstraint {
        //since |dp/dt| = 1 = const, tan accel is always perpendicular to tangental
        //== sqrt(max^2-center^2)
        override fun accelRange(point: PathPoint, curVelocity: Double): Interval = Interval.symmetric(
            sqrt(
                max.pow(2) -
                        (point.tanAngleDeriv * curVelocity.pow(2)).pow(2)
            ).ifNan { 0.0 }
        )
    }
}

/**
 * A constraint on [max] angular velocity of the _heading_.
 */
class MaxAngularVelocity(max: Double) : MaxBasedConstraint(max), VelocityConstraint {

    override fun maxVelocity(point: PathPoint): Double = abs(max / point.headingDeriv)
}

/**
 * A constraint on [max] angular acceleration of the _heading_.
 */
class MaxAngularAcceleration(max: Double) : MaxBasedConstraint(max), AccelerationConstraint {

    override fun accelRange(point: PathPoint, curVelocity: Double): Interval {
        val deriv = point.headingDeriv
        return if (deriv == 0.0) Interval.REAL else
            Interval.symmetricBetween(
                max / deriv,
                -point.headingSecondDeriv * curVelocity.pow(2) / deriv
            )
    }
}

/**
 * A constraint on [max] **path tangent angle** velocity.
 */
class MaxPathAngularVelocity(max: Double) : MaxBasedConstraint(max), VelocityConstraint {

    override fun maxVelocity(point: PathPoint): Double = abs(max / point.tanAngleDeriv)
}

/**
 * A constraint on [max] **path tangent angle** acceleration.
 */
class MaxPathAngularAcceleration(max: Double) : MaxBasedConstraint(max), AccelerationConstraint {

    override fun accelRange(point: PathPoint, curVelocity: Double): Interval {
        //second derivative chain rule, solve for s''(t)
        val deriv = point.tanAngleDeriv
        return if (deriv == 0.0) Interval.REAL else
            Interval.symmetricBetween(
                max / deriv,
                -point.tanAngleSecondDeriv * curVelocity.pow(2) / deriv
            )
    }
}
