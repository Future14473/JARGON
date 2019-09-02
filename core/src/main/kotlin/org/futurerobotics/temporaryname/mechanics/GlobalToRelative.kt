package org.futurerobotics.temporaryname.mechanics

import org.futurerobotics.temporaryname.math.Pose2d
import kotlin.math.PI

/**
 * Calculations for kinematics; (math with motion and acceleration, not regarding forces)
 */
object GlobalToRelative {

    /**
     * Converts [globalVelocity] (field velocity) to relative velocity (bot velocity), given the [globalHeading].
     */
    fun velocity(globalVelocity: Pose2d, globalHeading: Double): Pose2d =
        globalVelocity.vecRotated(-globalHeading)

    /**
     * Converts a [globalError] in pose to relative error given the [globalHeading]
     */
    fun error(globalError: Pose2d, globalHeading: Double): Pose2d =
        globalError.vecRotated(-globalHeading)

    /**
     * Converts a _expected_ [globalMotion] in poses to relative [globalMotion], given the true [globalHeading]
     */
    fun motion(globalMotion: PoseMotion, globalHeading: Double): PoseMotion {
        val (v, a) = globalMotion
        val rv = v.vecRotated(-globalHeading)
        val ra = Pose2d(
            a.vec.rotated(-globalHeading) +
                    v.vec.rotated(-globalHeading + PI / 2) * -v.heading,
            a.heading
        )
        return PoseMotion(rv, ra)
    }

    /**
     * Converts a global [reference] PoseMotionState to a relative reference (to where the relative current state is
     * Pose2d.Zero), given the current [globalPose]
     */
    fun reference(reference: PoseMotionState, globalPose: Pose2d): PoseMotionState {
        val (s, v, a) = reference
        val rs = (s - globalPose).vecRotated(-globalPose.heading)
        val rv = v.vecRotated(-globalPose.heading)
        val ra = Pose2d(
            a.vec.rotated(-globalPose.heading) +
                    v.vec.rotated(-globalPose.heading + PI / 2) * -v.heading,
            a.heading
        )
        return PoseMotionState(rs, rv, ra)
    }
}