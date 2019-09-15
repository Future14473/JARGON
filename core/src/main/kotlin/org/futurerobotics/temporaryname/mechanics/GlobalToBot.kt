package org.futurerobotics.temporaryname.mechanics

import org.futurerobotics.temporaryname.math.Pose2d
import kotlin.math.PI

/**
 * Calculations for mapping global pose information to bot pose information.
 */
object GlobalToBot {

    /**
     * Converts [globalVelocity] (field velocity) to relative velocity (bot velocity), given the [globalHeading].
     */
    @JvmStatic
    fun velocity(globalVelocity: Pose2d, globalHeading: Double): Pose2d =
        globalVelocity.vecRotated(-globalHeading)

    /**
     * Converts a [globalError] in pose to bot error given the [globalHeading]
     */
    @JvmStatic
    fun error(globalError: Pose2d, globalHeading: Double): Pose2d =
        globalError.vecRotated(-globalHeading)

    /**
     * Converts a _expected_ [globalMotion] in poses to bot [globalMotion], given the true [globalHeading]
     */
    @JvmStatic
    fun motion(globalMotion: Motion<Pose2d>, globalHeading: Double): Motion<Pose2d> {
        val (v, a) = globalMotion
        val rv = v.vecRotated(-globalHeading)
        val ra = Pose2d(
            a.vec.rotated(-globalHeading) +
                    v.vec.rotated(-globalHeading + PI / 2) * -v.heading,
            a.heading
        )
        return ValueMotion(rv, ra)
    }

    /**
     * Converts a global [reference] PoseMotionState to a bot reference (to where the bot's "current state" is
     * Pose2d.Zero), given the current [globalPose]
     */
    @JvmStatic
    fun reference(reference: State<Pose2d>, globalPose: Pose2d): State<Pose2d> {
        val (s, v, a) = reference
        val rs = (s - globalPose).vecRotated(-globalPose.heading)
        val rv = v.vecRotated(-globalPose.heading)
        val ra = Pose2d(
            a.vec.rotated(-globalPose.heading) +
                    v.vec.rotated(-globalPose.heading + PI / 2) * -v.heading,
            a.heading
        )
        return ValueState(rs, rv, ra)
    }
}