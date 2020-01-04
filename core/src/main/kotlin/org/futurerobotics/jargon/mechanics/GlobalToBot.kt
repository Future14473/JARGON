package org.futurerobotics.jargon.mechanics

import org.futurerobotics.jargon.math.*
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
     * Converts [globalMotion] in poses to bot [motion], given the true [globalHeading]
     */
    @JvmStatic
    fun motion(globalMotion: MotionOnly<Pose2d>, globalHeading: Double): MotionOnly<Pose2d> {
        val (v, a) = globalMotion
        val rv = v.vecRotated(-globalHeading)
        val ra = Pose2d(
            a.vec.rotated(-globalHeading) +
                    v.vec.rotated(-globalHeading + PI / 2) * -v.heading,
            a.heading
        )
        return MotionOnly(rv, ra)
    }

    /**
     * Converts a global [reference] PoseMotionState to a bot reference (to where the bot's "current state" is
     * [Pose2d.ZERO], given the current [globalPose]
     */
    @JvmStatic
    fun referenceMotion(reference: MotionState<Pose2d>, globalPose: Pose2d): MotionState<Pose2d> {
        val (s, v, a) = reference
        val globalHeading = globalPose.heading
        val rs = (s - globalPose).vecRotated(-globalHeading)
        val rv = v.vecRotated(-globalHeading)
        val ra = Pose2d(
            a.vec.rotated(-globalHeading) -
                    v.vec.rotated(-globalHeading + PI / 2) * v.heading,
            a.heading
        )
        return MotionState(rs, rv, ra)
    }

    /**
     * Non-linearly updates the current global pose, given the change in pose from the bot's perspective, [botPoseDelta].
     * and the current global pose.
     */
    @JvmStatic
    fun trackGlobalPose(
        botPoseDelta: Pose2d,
        currentGlobalPose: Pose2d
    ): Pose2d {
        val (x, y, dTheta) = botPoseDelta
        val sinc = sinc(dTheta)
        val cosc = cosc(dTheta)
        val relativeDiff = Vector2d(sinc * x - cosc * y, cosc * x + sinc * y)
        val dPose = Pose2d(relativeDiff.rotated(currentGlobalPose.heading), dTheta)
        return (currentGlobalPose + dPose).angleNormalized()
    }
}
