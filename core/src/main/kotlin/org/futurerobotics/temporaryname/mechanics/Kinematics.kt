package org.futurerobotics.temporaryname.mechanics

import org.futurerobotics.temporaryname.math.Pose2d

/**
 * Calculations for kinematics; (math with motion and acceleration, not regarding forces)
 */
object Kinematics {

    /**
     * Converts [globalVelocity] (field velocity) to relative velocity (bot velocity), given the [globalHeading].
     */
    fun globalToRelativeVelocity(globalVelocity: Pose2d, globalHeading: Double): Pose2d =
        Pose2d(globalVelocity.vec.rotated(-globalHeading), globalVelocity.heading)

    /**
     * Converts [globalVelocity] (field velocity) to relative velocity (bot velocity), given the [globalPose].
     */
    fun globalToRelativeVelocity(globalVelocity: Pose2d, globalPose: Pose2d): Pose2d =
        Pose2d(globalVelocity.vec.rotated(-globalPose.heading), globalVelocity.heading)

    /**
     * Gets a new pose given the [pastGlobalPose], and [relativeDeltaPose] (non-linear update)
     */
    fun odometryUpdate(pastGlobalPose: Pose2d, relativeDeltaPose: Pose2d): Pose2d {
    }
}