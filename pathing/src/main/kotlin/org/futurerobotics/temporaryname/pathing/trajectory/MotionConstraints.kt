package org.futurerobotics.temporaryname.pathing.trajectory

import org.futurerobotics.temporaryname.math.EPSILON
import org.futurerobotics.temporaryname.math.Interval
import org.futurerobotics.temporaryname.math.notNaNOrElse
import org.futurerobotics.temporaryname.math.squared
import org.futurerobotics.temporaryname.pathing.PathPoint
import kotlin.math.abs
import kotlin.math.sqrt

/**
 * A constraint on _path_ tangent velocity, not allowing a motion that travels faster than [max] along the curve.
 */
class MaxVelocityConstraint(max: Double) : MaxBasedVelocityConstraint(max) {

    override fun maxVelocity(point: PathPoint): Double = max
}

/**
 * A constraint on _path_ tangent acceleration, not allowing a motion that accelerates more than [max]
 * _in the direction tangent to the curve_.
 * @see MaxCentripetalAccelConstraint
 * @see MaxTotalAccelerationConstraint
 */
class MaxTangentAccelConstraint(max: Double) : MaxBasedAccelConstraint(max) {

    private val interval = Interval.symmetric(max)

    init {
        require(max >= EPSILON) { "Impossible constraint." }
    }

    override fun maxAccelRange(point: PathPoint, curVelocity: Double): Interval = interval
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
 * @see MaxTangentAccelConstraint
 * @see MaxTotalAccelerationConstraint
 */
class MaxCentripetalAccelConstraint(max: Double) : MaxBasedVelocityConstraint(max) {

    override fun maxVelocity(point: PathPoint): Double {
        //a_c = v^2*c <= max
        //v <= sqrt(max/c)
        return sqrt(abs(max / point.tanAngleDeriv))
    }
}

/**
 * Represents a constraint on total acceleration, not allowing a motion that has a total acceleration in any direction
 * more than [max]. This is centripetal and tangential combined.
 */
class MaxTotalAccelerationConstraint(max: Double) : MultipleConstraint() {

    override val velocityConstraints: Collection<VelocityConstraint> = listOf(
        MaxCentripetalAccelConstraint(max)
    )
    override val accelConstraints: Collection<AccelConstraint> = listOf(
        DynamicTangentalAccelConstraint(
            max
        )
    )

    init {
        require(max >= EPSILON) { "Impossible constraint." }
    }

    private class DynamicTangentalAccelConstraint(max: Double) : MaxBasedAccelConstraint(max) {
        override fun maxAccelRange(point: PathPoint, curVelocity: Double): Interval =
            Interval.symmetric(
                //since |dp/dt| = 1 = const, tan accel is always perpendicular to tangental
                //== sqrt(max^2-center^2)
                //super: unambiguous
                sqrt(super.max.squared() - (point.tanAngleDeriv * curVelocity.squared()).squared()).notNaNOrElse { 0.0 })
    }
}

/**
 * Represents a constraint on angular velocity _by the path's tangent_, not allowing a motion that has an angular velocity of a magnitude
 * greater than [max].
 */
class MaxPathAngularVelocityConstraint(max: Double) : MaxBasedVelocityConstraint(max) {

    override fun maxVelocity(point: PathPoint): Double = abs(max / point.tanAngleDeriv)
}

/**
 * Represents a constraint on angular velocity _by the path's tangent_, not allowing a motion that has an angular acceleration of a magnitude
 * greater than [max].
 */
class MaxPathAngularAccelConstraint(max: Double) : MaxBasedAccelConstraint(max) {

    override fun maxAccelRange(point: PathPoint, curVelocity: Double): Interval {
        //second derivative chain rule, solve for s''(t)
        val deriv = point.tanAngleDeriv
        return Interval.symmetricRegular(
            max / deriv, -point.tanAngleSecondDeriv * curVelocity.squared() / deriv
        )
    }
}

/**
 * Represents a constraint on angular velocity _by the heading (of the actual robot)_, not allowing a motion that has an angular velocity of a magnitude
 * greater than [max].
 */
class MaxAngularVelocityConstraint(max: Double) : MaxBasedVelocityConstraint(max) {

    override fun maxVelocity(point: PathPoint): Double = abs(max / point.headingDeriv)
}

/**
 * Represents a constraint on angular acceleration _by the heading (of the actual robot)_, not allowing a motion that has an angular acceleration of a magnitude
 * greater than [max].
 */
class MaxAngularAccelerationConstraint(max: Double) : MaxBasedAccelConstraint(max) {

    override fun maxAccelRange(point: PathPoint, curVelocity: Double): Interval {
        val deriv = point.headingDeriv
        return Interval.symmetricRegular(
            max / deriv, -point.headingSecondDeriv * curVelocity.squared() / deriv
        )
    }

    override fun toString(): String = "RobotAngularAccelConstraint(max=$max)"
}

//Fallback constraints
private const val FALLBACK_MAX_VEL = 10000.0
private const val FALLBACK_MAX_ACCEL = 10000.0
internal val FALLBACK_VELOCITY_CONSTRAINTS = listOf<VelocityConstraint>(object : VelocityConstraint() {
    override fun maxVelocity(point: PathPoint): Double =
        FALLBACK_MAX_VEL

    override fun toString(): String = "FallbackVelocityConstraint(max=$FALLBACK_MAX_VEL)"
})
internal val FALLBACK_ACCEL_CONSTRAINTS = listOf<AccelConstraint>(object : AccelConstraint() {
    private val interval = Interval.symmetric(FALLBACK_MAX_ACCEL)
    override fun maxAccelRange(point: PathPoint, curVelocity: Double) = interval
    override fun toString(): String = "FallbackAccelConstraint(max=$FALLBACK_MAX_ACCEL)"
})





