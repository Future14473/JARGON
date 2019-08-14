package org.futurerobotics.temporaryname.pathing.constraint

import org.futurerobotics.temporaryname.math.Interval
import org.futurerobotics.temporaryname.pathing.PathPoint

/**
 * Common superclass of all Motion Constraints, used in a [MotionConstraintSet] for dynamic motion profile generation.
 * This includes:
 * - [VelocityConstraint], a constraint on velocities at points on a path,
 * - [AccelConstraint], a constraint on acceleration at a point given current velocity,
 * - [MultipleConstraint], a combination of one or more of the above.
 */
sealed class MotionConstraint

/**
 * Common superclass of both VelocityConstraint and AccelConstraint, but not MultipleConstraint.
 */
sealed class SingleConstraint : MotionConstraint() {
    /**
     * Compares motion constraints, to remove redundant ones if multiple are added.
     *
     * Similar to compare to,
     *
     * If this returns a negative number, then this constraint is less than (always more restrictive) than the other,
     * and so only this constraint will be used.
     *
     * If returns positive, vice versa.
     *
     * If returns 0, no constraints will be removed. Return 0 if not of the same type.
     */
    open fun compareConstraints(other: MotionConstraint): Int = 0
}

/**
 * Represents a constraint on velocity (in arc length) for any point on a path, given the data provided in [PathPoint].
 */
abstract class VelocityConstraint : SingleConstraint() {
    /**
     * Returns the maximum possible velocity given by this constraint, given the current path [point] info.
     * Maximum velocity must be >= 0.
     */
    abstract fun maxVelocity(point: PathPoint): Double
}

/**
 * Represents a constraint on acceleration (in arc length) for any point along a path, give the data provided in [PathPoint] and
 * the current velocity (of arc length) along this path.
 */
abstract class AccelConstraint : SingleConstraint() {
    /**
     * Returns an [Interval] of position accelerations (both positive and negative) given by this constraint,
     * given the current path [point] info and [curVelocity].
     *
     * It is assumed (in profile generation) that if the velocity is slower, this constraint is somewhat more lenient
     * and/or has values closer to 0
     */
    abstract fun maxAccelRange(point: PathPoint, curVelocity: Double): Interval
}

/**
 * Represents a constraint that needs to be represented by both one or more of [VelocityConstraint] and [AccelConstraint] together.
 */
abstract class MultipleConstraint : MotionConstraint() {
    /** The [VelocityConstraint] components of this multiple constraint */
    abstract val velocityConstraints: Collection<VelocityConstraint>
    /** Th [AccelConstraint] components of this multiple constraint */
    abstract val accelConstraints: Collection<AccelConstraint>
}
//
//interface JerkConstraint {
//   Algorithm does not yet support this, and theoretically it would be slow.
//}
/**
 * A base implementation of a [VelocityConstraint] based upon some positive max value, where if the maximum value is lower, it
 * implies more restrictive constraint.
 *
 * @property max the maximum value of something
 */
abstract class MaxBasedVelocityConstraint(protected val max: Double) : VelocityConstraint() {

    init {
        require(max > 0) { "Max value $max gives impossible constraint" }
    }

    override fun compareConstraints(other: MotionConstraint): Int {
        if (other !is MaxBasedVelocityConstraint || this::class != other::class) return 0
        return max.compareTo(other.max)
    }

    override fun toString(): String = "${(javaClass.simpleName ?: "anonymous AccelConstraint")}(max=$max)"
}

/**
 * A base implementation of a [AccelConstraint] based upon some maximum value, where if the maximum value is lower, it
 * implies more restrictive constraint.
 *
 * @property max the maximum value of something
 */
abstract class MaxBasedAccelConstraint(protected val max: Double) : AccelConstraint() {

    init {
        require(max > 0) { "Max value $max gives impossible constraint" }
    }

    override fun compareConstraints(other: MotionConstraint): Int {
        if (other !is MaxBasedAccelConstraint || this::class != other::class) return 0
        return max.compareTo(other.max)
    }

    override fun toString(): String = "${(javaClass.simpleName ?: "anonymous AccelConstraint")}(max=$max)"
}
