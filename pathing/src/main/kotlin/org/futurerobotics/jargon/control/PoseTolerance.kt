package org.futurerobotics.jargon.control

import org.futurerobotics.jargon.math.Pose2d
import org.futurerobotics.jargon.math.angleNorm
import kotlin.math.absoluteValue

/**
 * Represents tolerances to use for when to dictate finishing moving on this motion profiled.
 *
 * @param positionalTolerance the maximum allowed difference position
 * @param angularTolerance the maximum allowed difference in angle
 */
data class PoseTolerance(val positionalTolerance: Double, val angularTolerance: Double) {

    /**
     * Returns true if the differences in [pose1] and [pose2] are small enough to be within this tolerance.
     */
    fun areSatisfied(pose1: Pose2d, pose2: Pose2d): Boolean =
        (pose1.vec distTo pose2.vec) <= positionalTolerance &&
                angleNorm(pose1.heading - pose2.heading).absoluteValue <= angularTolerance

    companion object {
        /**
         * A [PoseTolerance] with no tolerance whatsoever -- the control can only be ended manually
         */
        @JvmField
        val NONE: PoseTolerance = PoseTolerance(0.0, 0.0)
    }
}