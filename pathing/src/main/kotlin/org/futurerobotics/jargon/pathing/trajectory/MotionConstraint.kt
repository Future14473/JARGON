package org.futurerobotics.jargon.pathing.trajectory

import org.futurerobotics.jargon.math.Interval
import org.futurerobotics.jargon.pathing.PathPoint

/**
 * Common interface of all motion constraints, used in a [MotionConstraintSet] for dynamic motion profile generation.
 *
 * This includes:
 * - [VelocityConstraint], a constraint on the maximum velocity at given points along a path
 * - [AccelerationConstraint], a constraint on the allowed accelerations at points along a path
 * - [MultipleConstraint], a combination of one or more of the above.
 *
 * Interfaces of [MotionConstraint] that are not one of the above will be ignored.
 */
interface MotionConstraint

/**
 * Common interface of both [VelocityConstraint] and [AccelerationConstraint], but not MultipleConstraint.
 */
interface SingleConstraint : MotionConstraint {

    /**
     * Compares this constraint to another constraint, and if it is known that this constraint is always
     * more or equally restrictive compared to [other] constraint (other constraint is redundant).
     *
     * If not known or don't care, return false.
     */
    fun otherIsRedundant(other: SingleConstraint): Boolean = false
}

/**
 * A constraint on the maximum _tangential_ velocity for points on a path.
 *
 * @see MotionConstraint
 * @see SingleConstraint
 * @see PathPoint
 */
interface VelocityConstraint : SingleConstraint {

    /**
     * Returns the maximum allowed velocity given by this constraint at the given [point].
     * Maximum velocity must be >= 0.
     */
    fun maxVelocity(point: PathPoint): Double
}

/**
 * A constraint on the range of allowable _tangential_ accelerations for points on a path.
 *
 * @see MotionConstraint
 * @see SingleConstraint
 * @see PathPoint
 * @see Interval
 */
interface AccelerationConstraint : SingleConstraint {

    /**
     * Returns an [Interval] of allowable accelerations (both positive and negative) at the given [point],
     * also given the [curVelocity] at that point.
     *
     * It is assumed that if the [curVelocity] is lower, this constraint is more lenient, or contains values closer
     * to 0.
     *
     * This _may_ be called multiple times with the same [point] but different [curVelocity].
     */
    fun accelRange(point: PathPoint, curVelocity: Double): Interval
}
//
//interface JerkConstraint {
//   Algorithm does not yet support this, and theoretically it would be slow.
//   Instead, non-dynamic profiles or CONSTANT jerk constraint used instead; in the future?
//}
/**
 * Represents a constraint that needs to be represented by both one or more of [VelocityConstraint] and [AccelerationConstraint] together.
 */
interface MultipleConstraint : MotionConstraint {

    /** The [VelocityConstraint] components of this multiple constraint */
    val velocityConstraints: Collection<VelocityConstraint>

    /** The [AccelerationConstraint] components of this multiple constraint */
    val accelerationConstraints: Collection<AccelerationConstraint>
}

/**
 * A constraint that is based off of some [max] value.
 *
 * A lower max value implies a more restrictive constraint (see [SingleConstraint.otherIsRedundant]).
 *
 * @property max the maximum allowed value of something.
 */
abstract class MaxBasedConstraint(@JvmField protected val max: Double) : SingleConstraint {

    init {
        require(max > 0) { "Max value $max gives impossible constraint" }
    }

    override fun otherIsRedundant(other: SingleConstraint): Boolean {
        if (other !is MaxBasedConstraint) return false

        var commonSupertype: Class<*> = this.javaClass
        while (!commonSupertype.isAssignableFrom(other.javaClass))
            commonSupertype = commonSupertype.superclass

        return commonSupertype != MaxBasedConstraint::class.java && max <= other.max
    }

    override fun toString(): String {
        val name = javaClass.simpleName.ifEmpty {
            when (this) {
                is VelocityConstraint -> "anonymous VelConstraint"
                is AccelerationConstraint -> "anonymous AccelConstraint"
                else -> "anonymous SingleConstraint"
            }
        }
        return "$name(max=$max)"
    }
}
