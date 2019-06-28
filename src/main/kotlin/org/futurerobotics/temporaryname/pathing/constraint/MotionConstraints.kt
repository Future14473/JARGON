package org.futurerobotics.temporaryname.pathing.constraint

import org.futurerobotics.temporaryname.math.EPSILON
import org.futurerobotics.temporaryname.math.Interval
import org.futurerobotics.temporaryname.math.notNaNOrElse
import org.futurerobotics.temporaryname.math.squared
import org.futurerobotics.temporaryname.pathing.path.PathPointInfo
import kotlin.math.abs
import kotlin.math.sqrt


/**
 * A constraint on _path_ tangent velocity, not allowing a motion that travels faster than [max] along the curve.
 */
class TangentVelocityConstraint(private val max: Double) : VelocityConstraint() {
    init {
        require(max >= EPSILON) { "Impossible constraint with given max $max." }
    }

    override fun maxVelocity(point: PathPointInfo): Double = max
    override fun toString(): String = "TangentVelocityConstraint(max=$max)"
}

/**
 * A constraint on _path_ tangent acceleration, not allowing a motion that accelerates more than [max]
 * _in the direction tangent to the curve_.
 * @see CentripetalAccelConstraint
 * @see TotalAccelConstraint
 */
class TangentAccelConstraint(max: Double) : AccelConstraint() {
    init {
        require(max >= EPSILON) { "Impossible constraint." }
    }

    private val interval = Interval.symmetric(max)

    override fun maxAccelRange(point: PathPointInfo, curVelocity: Double, reversed: Boolean): Interval = interval

    override fun toString(): String = "TangentAccelConstraint(max=${interval.b})"
}

/**
 * Represents a constraint on centripetal acceleration, not allowing a motion that has a _centripetal_ acceleration
 * more than [max].
 *
 * Centripetal acceleration is always perpendicular to motion and TangentAcceleration
 *
 * Since centripetal acceleration is only dependent on velocity, this is actually a [VelocityConstraint]
 *
 * @see TangentAccelConstraint
 * @see TotalAccelConstraint
 */
class CentripetalAccelConstraint(private val max: Double) : VelocityConstraint() {
    init {
        require(max >= EPSILON) { "Impossible constraint." }
    }

    override fun maxVelocity(point: PathPointInfo): Double {
        //a_c = v^2*c
        //v <= sqrt(a_c/c)
        return sqrt(abs(max / point.tanAngleDeriv))
    }

    override fun toString(): String = "CentripetalAccelConstraint(max=$max)"
}

/**
 * Represents a constraint on total acceleration, not allowing a motion that has a total acceleration in any direction
 * more than [max]. This is centripetal and tangential combined.
 */
class TotalAccelConstraint(max: Double) : MultipleConstraint() {
    init {
        require(max >= EPSILON) { "Impossible constraint." }
    }

    override val velocityConstraints: Collection<VelocityConstraint> = listOf(CentripetalAccelConstraint(max))
    override val accelConstraints: Collection<AccelConstraint> = listOf(object : AccelConstraint() {
        private val maxSquared = max.squared()
        override fun maxAccelRange(point: PathPointInfo, curVelocity: Double, reversed: Boolean): Interval =
            Interval.symmetric(
                //since |dp/dt| = 1 = const, second accel is always perpendicular.
                //== sqrt(max^2-centr^2)
                sqrt(maxSquared - (point.tanAngleDeriv * curVelocity.squared()).squared()).notNaNOrElse { 0.0 })
    })
}

/**
 * Represents a constraint on angular velocity _by the path's tangent_, not allowing a motion that has an angular velocity of a magnitude
 * greater than [max].
 */
class PathAngularVelocityConstraint(private val max: Double) : VelocityConstraint() {
    override fun maxVelocity(point: PathPointInfo): Double = abs(max / point.tanAngleDeriv)
    override fun toString(): String = "PathAngularVelocityConstraint(max=$max)"
}

/**
 * Represents a constraint on angular velocity _by the path's tangent_, not allowing a motion that has an angular acceleration of a magnitude
 * greater than [max].
 */
class PathAngularAccelConstraint(private val max: Double) : AccelConstraint() {
    override fun maxAccelRange(point: PathPointInfo, curVelocity: Double, reversed: Boolean): Interval {
        //second derivative chain rule, solve for s''(t)
        val deriv = point.tanAngleDeriv
        return Interval.symmetricRegular(
            max / deriv,
            -point.tanAngleSecondDeriv * curVelocity.squared() / deriv
        )
    }

    override fun toString(): String = "PathAngularAccelConstraint(max=$max)"
}

/**
 * Represents a constraint on angular velocity _by the heading (of the actual robot)_, not allowing a motion that has an angular velocity of a magnitude
 * greater than [max].
 */
class RobotAngularVelocityConstraint(private val max: Double) : VelocityConstraint() {
    override fun maxVelocity(point: PathPointInfo): Double = abs(max / point.headingDeriv)
    override fun toString(): String = "PathAngularVelocityConstraint(max=$max)"
}

/**
 * Represents a constraint on angular acceleration _by the heading (of the actual robot)_, not allowing a motion that has an angular acceleration of a magnitude
 * greater than [max].
 */
class RobotAngularAccelConstraint(private val max: Double) : AccelConstraint() {
    override fun maxAccelRange(point: PathPointInfo, curVelocity: Double, reversed: Boolean): Interval {
        val deriv = point.headingDeriv
        return Interval.symmetricRegular(
            max / deriv,
            -point.headingSecondDeriv * curVelocity.squared() / deriv
        )
    }

    override fun toString(): String = "RobotAngularAccelConstraint(max=$max)"
}
//Fallback constraints

private const val FALLBACK_MAX_VEL = 10000.0
private const val FALLBACK_MAX_ACCEL = 10000.0
internal val FALLBACK_VELOCITY_CONSTRAINTS: List<VelocityConstraint> = listOf(FallbackVelocityConstraint)
internal val FALLBACK_ACCEL_CONSTRAINTS: List<AccelConstraint> = listOf(FallbackAccelConstraint)

private object FallbackVelocityConstraint : VelocityConstraint() {
    override fun maxVelocity(point: PathPointInfo): Double = FALLBACK_MAX_VEL
    override fun toString(): String = "FallbackVelocityConstraint(max=$FALLBACK_MAX_VEL)"
}


private object FallbackAccelConstraint : AccelConstraint() {
    private val interval = Interval.symmetric(FALLBACK_MAX_ACCEL)
    override fun maxAccelRange(point: PathPointInfo, curVelocity: Double, reversed: Boolean) = interval
    override fun toString(): String = "FallbackAccelConstraint(max=$FALLBACK_MAX_ACCEL)"
}

