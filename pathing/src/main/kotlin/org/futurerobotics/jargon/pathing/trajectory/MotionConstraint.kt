package org.futurerobotics.jargon.pathing.trajectory

import org.futurerobotics.jargon.math.Interval
import org.futurerobotics.jargon.pathing.PathPoint

/**
 * Common superclass of all Motion Constraints, used in a [MotionConstraintSet] for dynamic motion profile generation.
 * This includes:
 * - [VelocityConstraint], a constraint on velocities at points on a path,
 * - [AccelConstraint], a constraint on acceleration at a point given current velocity,
 * - [MultipleConstraint], a combination of one or more of the above.
 *
 * Interfaces of [MotionConstraint] that are not one of the above will be ignored.
 */
interface MotionConstraint

/**
 * Common superclass of both VelocityConstraint and AccelConstraint, but not MultipleConstraint.
 */
interface SingleConstraint : MotionConstraint {

    /**
     * Compares this constraint to another constraint; and returns true if it is known that this constraint is always
     * more or equally restrictive compared to [other] constraint (other constraint is redundant).
     *
     * If not known, return false.
     */
    fun otherIsRedundant(other: SingleConstraint): Boolean = false
}

/**
 * Represents a constraint on velocity (in arc length) for any point on a path, given the data provided in [PathPoint].
 */
interface VelocityConstraint : SingleConstraint {

    /**
     * Returns the maximum possible velocity given by this constraint, given the current path [point] info.
     * Maximum velocity must be >= 0.
     */
    fun maxVelocity(point: PathPoint): Double
}

/**
 * Represents a constraint on acceleration (in arc length) for any point along a path, give the data provided in [PathPoint] and
 * the current velocity (of arc length) along this path.
 */
interface AccelConstraint : SingleConstraint {

    /**
     * Returns an [Interval] of position accelerations (both positive and negative) given by this constraint,
     * given the current path [point] info and [curVelocity].
     *
     * It is assumed (in profile generation) that if the velocity is slower, this constraint is somewhat more lenient
     * and/or has values closer to 0
     */
    fun maxAccelRange(point: PathPoint, curVelocity: Double): Interval
}
//
//interface JerkConstraint {
//   Algorithm does not yet support this, and theoretically it would be slow.
//   Instead, possible non-dynamic profiles or CONSTANT jerk constraint used instead; in the future?
//}
/**
 * Represents a constraint that needs to be represented by both one or more of [VelocityConstraint] and [AccelConstraint] together.
 */
interface MultipleConstraint : MotionConstraint {

    /** The [VelocityConstraint] components of this multiple constraint */
    val velocityConstraints: Collection<VelocityConstraint>
    /** The [AccelConstraint] components of this multiple constraint */
    val accelConstraints: Collection<AccelConstraint>
}

/**
 * A base implementation of a [VelocityConstraint] based upon some positive max value, where if the maximum value is lower, it
 * implies more restrictive constraint.
 *
 * @property max the maximum value of something
 */
abstract class MaxBasedVelocityConstraint(protected val max: Double) : VelocityConstraint {

    init {
        require(max > 0) { "Max value $max gives impossible constraint" }
    }

    override fun otherIsRedundant(other: SingleConstraint): Boolean =
        other is MaxBasedVelocityConstraint && this.javaClass == other.javaClass && this.max <= other.max

    override fun toString(): String = "${(javaClass.simpleName ?: "anonymous AccelConstraint")}(max=$max)"
}

/**
 * A base implementation of a [AccelConstraint] based upon some maximum value, where if the maximum value is lower, it
 * implies more restrictive constraint.
 *
 * @property max the maximum value of something
 */
abstract class MaxBasedAccelConstraint(protected val max: Double) : AccelConstraint {

    init {
        require(max > 0) { "Max value $max gives impossible constraint" }
    }

    override fun otherIsRedundant(other: SingleConstraint): Boolean =
        other is MaxBasedAccelConstraint && this.javaClass == other.javaClass && this.max <= other.max

    override fun toString(): String = "${(javaClass.simpleName ?: "anonymous AccelConstraint")}(max=$max)"
}
