package org.futurerobotics.jargon.pathing.trajectory

import org.futurerobotics.jargon.math.EPSILON
import org.futurerobotics.jargon.math.Interval
import org.futurerobotics.jargon.math.notNaNOrElse
import org.futurerobotics.jargon.pathing.PathPoint
import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.sqrt

/**
 * A constraint on _path_ tangent velocity, not allowing a motion that travels faster than [max] along the curve.
 */
class MaxVelConstraint(max: Double) : MaxBasedConstraint(max), VelConstraint {

    override fun maxVelocity(point: PathPoint): Double = max
}

/**
 * A constraint on _path_ tangent acceleration, not allowing a motion that accelerates more than [max]
 * _in the direction tangent to the curve_.
 * @see MaxCentripetalAccelConstraint
 * @see MaxTotalAccelConstraint
 */
class MaxTangentAccelConstraint(max: Double) : MaxBasedConstraint(max), AccelConstraint {

    private val interval = Interval.symmetric(max)

    init {
        require(max >= EPSILON) { "Impossible constraint." }
    }

    override fun accelRange(point: PathPoint, curVelocity: Double): Interval = interval
    override fun toString(): String = "TangentAccelConstraint(max=${interval.b})"
}

/**
 * Represents a constraint on centripetal acceleration, not allowing a motion that has a _centripetal_ acceleration
 * more than [max].
 *
 * Centripetal acceleration is always perpendicular to motion and tangential acceleration.
 *
 * Since centripetal acceleration is only dependent on velocity, this is actually a [VelConstraint]
 *
 * @see MaxTangentAccelConstraint
 * @see MaxTotalAccelConstraint
 */
class MaxCentripetalAccelConstraint(max: Double) : MaxBasedConstraint(max), VelConstraint {

    override fun maxVelocity(point: PathPoint): Double =//a_c = v^2*c <= max
        //v <= sqrt(max/c)
        sqrt(abs(max / point.tanAngleDeriv))
}

/**
 * Represents a constraint on total acceleration, not allowing a motion that has a total acceleration in any direction
 * more than [max]. This is centripetal and tangential combined.
 */
class MaxTotalAccelConstraint(max: Double) : MultipleConstraint {

    override val velConstraints: Collection<VelConstraint> = listOf(MaxCentripetalAccelConstraint(max))
    override val accelConstraints: Collection<AccelConstraint> = listOf(TotalAccelConstraint(max))

    init {
        require(max >= EPSILON) { "Impossible constraint." }
    }

    private class TotalAccelConstraint(max: Double) : MaxBasedConstraint(max), AccelConstraint {
        //since |dp/dt| = 1 = const, tan accel is always perpendicular to tangental
        //== sqrt(max^2-center^2)
        //super: unambiguous
        override fun accelRange(point: PathPoint, curVelocity: Double): Interval = Interval.symmetric(
            sqrt(super.max.pow(2) - (point.tanAngleDeriv * curVelocity.pow(2)).pow(2))
                .notNaNOrElse { 0.0 }
        )
    }
}

/**
 * Represents a constraint on angular velocity _by the path's tangent_, not allowing a motion that has an angular velocity of a magnitude
 * greater than [max].
 */
class MaxPathAngularVelConstraint(max: Double) : MaxBasedConstraint(max), VelConstraint {

    override fun maxVelocity(point: PathPoint): Double = abs(max / point.tanAngleDeriv)
}

/**
 * Represents a constraint on angular velocity _by the path's tangent_, not allowing a motion that has an angular acceleration of a magnitude
 * greater than [max].
 */
class MaxPathAngularAccelConstraint(max: Double) : MaxBasedConstraint(max), AccelConstraint {

    override fun accelRange(point: PathPoint, curVelocity: Double): Interval {
        //second derivative chain rule, solve for s''(t)
        val deriv = point.tanAngleDeriv
        return if (deriv == 0.0) Interval.REAL else
            Interval.symmetricRegular(max / deriv, -point.tanAngleSecondDeriv * curVelocity.pow(2) / deriv)
    }
}

/**
 * Represents a constraint on angular velocity _by the heading (of the actual robot)_, not allowing a motion that has an angular velocity of a magnitude
 * greater than [max].
 */
class MaxAngularVelConstraint(max: Double) : MaxBasedConstraint(max), VelConstraint {

    override fun maxVelocity(point: PathPoint): Double = abs(max / point.headingDeriv)
}

/**
 * Represents a constraint on angular acceleration _by the heading (of the actual robot)_, not allowing a motion that has an angular acceleration of a magnitude
 * greater than [max].
 */
class MaxAngularAccelConstraint(max: Double) : MaxBasedConstraint(max), AccelConstraint {

    override fun accelRange(point: PathPoint, curVelocity: Double): Interval {
        val deriv = point.headingDeriv
        return if (deriv == 0.0) Interval.REAL else
            Interval.symmetricRegular(
                max / deriv, -point.headingSecondDeriv * curVelocity.pow(2) / deriv
            )
    }

    override fun toString(): String = "RobotAngularAccelConstraint(max=$max)"
}
