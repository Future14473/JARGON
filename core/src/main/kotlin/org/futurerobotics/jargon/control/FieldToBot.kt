package org.futurerobotics.jargon.control

import org.futurerobotics.jargon.math.*
import kotlin.math.PI

/**
 * Kinematics calculations for mapping global/field values to/from local/bot values.
 */
//TODO: consider name
object FieldToBot {

    /**
     * Converts a [globalVelocity] a velocity _relative to the bot_, given the [globalHeading].
     *
     * @see motion
     */
    @JvmStatic
    fun velocity(globalVelocity: Pose2d, globalHeading: Double): Pose2d =
        globalVelocity.vecRotated(-globalHeading)

    /**
     * Given a [targetGlobalPose] and the bot's [currentGlobalPose], gets the difference in position (error) _relative
     * to the bot_.
     */
    @JvmStatic
    fun position(targetGlobalPose: Pose2d, currentGlobalPose: Pose2d): Pose2d =
        (targetGlobalPose - currentGlobalPose).vecRotated(-currentGlobalPose.heading)

    /**
     * Converts a [globalMotion] to the motion _relative to the bot_, given the current [globalHeading].
     *
     * This includes centripetal acceleration.
     *
     * @see velocity
     */
    @JvmStatic
    fun motion(globalMotion: MotionOnly<Pose2d>, globalHeading: Double): MotionOnly<Pose2d> {
        val (v, a) = globalMotion
        val rv = v.vecRotated(-globalHeading)
        val ra = Pose2d(
            a.vector2d.rotated(-globalHeading) +
                v.vector2d.rotated(-globalHeading + PI / 2) * -v.heading,
            a.heading
        )
        return MotionOnly(rv, ra)
    }

    /**
     * Given a [targetGlobalState], and the bot's current [currentGlobalPose], converts the state to
     * the state _relative to the bot_.
     *
     * This is a combination of [position] and [motion].
     */
    @JvmStatic
    fun state(targetGlobalState: MotionState<Pose2d>, currentGlobalPose: Pose2d): MotionState<Pose2d> {
        val (s, v, a) = targetGlobalState

        val globalHeading = currentGlobalPose.heading

        val rs = position(s, currentGlobalPose)
        val rv = v.vecRotated(-globalHeading)
        val ra = Pose2d(
            a.vector2d.rotated(-globalHeading) -
                v.vector2d.rotated(-globalHeading + PI / 2) * v.heading,
            a.heading
        )
        return MotionState(rs, rv, ra)
    }

    /**
     * Tracks a bot's global pose given the [pastGlobalPose], and a calculated estimated change in pose relative to the
     * bot ([botPoseDelta]).
     */
    @JvmStatic
    fun trackGlobalPose(botPoseDelta: Pose2d, pastGlobalPose: Pose2d): Pose2d {
        val (x, y, dTheta) = botPoseDelta
        val sinc = sinc(dTheta)
        val cosc = cosc(dTheta)
        val relativeDiff = Vector2d(sinc * x - cosc * y, cosc * x + sinc * y)
        val dPose = Pose2d(relativeDiff.rotated(pastGlobalPose.heading), dTheta)
        return (pastGlobalPose + dPose).angleNormalized()
    }
}
