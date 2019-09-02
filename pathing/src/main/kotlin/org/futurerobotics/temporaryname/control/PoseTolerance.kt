package org.futurerobotics.temporaryname.control

import org.futurerobotics.temporaryname.math.Pose2d
import org.futurerobotics.temporaryname.math.angleNorm
import kotlin.math.absoluteValue

/**
 * Represents tolerances to use for when to dictate finishing moving on this motion profiled.
 */
data class PoseTolerance(val positionalTolerance: Double, val angularTolerance: Double) {

    /**
     * Returns true if the differences in [pose1] and [pose2] are small enough to be within this tolerance.
     */
    fun areSatisifed(pose1: Pose2d, pose2: Pose2d): Boolean =
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