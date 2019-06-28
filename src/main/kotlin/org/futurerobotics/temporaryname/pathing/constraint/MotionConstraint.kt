package org.futurerobotics.temporaryname.pathing.constraint

import org.futurerobotics.temporaryname.math.Interval
import org.futurerobotics.temporaryname.pathing.path.PathPointInfo

/**
 * Common superclass of all Motion Constraints.
 * This includes:
 * - [VelocityConstraint], a constraint on velocities at points on a path,
 * - [AccelConstraint], a constraint on acceleration at a point given current velocity,
 * - [MultipleConstraint], a combination of one or more of the above.
 */
sealed class MotionConstraint

/**
 * Represents a constraint on velocity (in arc length) for any point on a path, given the data provided in [PathPointInfo].
 */
abstract class VelocityConstraint : MotionConstraint() {
    /**
     * Returns the maximum possible velocity given by this constraint, given the current path [point] info.
     * Maximum velocity must be >= 0.
     */
    abstract fun maxVelocity(point: PathPointInfo): Double
}

/**
 * Represents a constraint on acceleration (in arc length) for any point along a path, give the data provided in [PathPointInfo] and
 * the current velocity (of arc length) along this path.
 */
abstract class AccelConstraint : MotionConstraint() {
    /**
     * Returns an [Interval] of position accelerations (both positive and negative) given by this constraint,
     * given the current path [point] info and [curVelocity].
     *
     * if [reversed], the path is currently being analyzed in the _reverse_ direction, and so the ***minimum** value of acceleration(s) is important will be used.
     *
     * It is assumed (in profile generation) that if the velocity is slower, this constraint is somewhat more lenient.
     */
    abstract fun maxAccelRange(
        point: PathPointInfo, curVelocity: Double, reversed: Boolean
    ): Interval
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