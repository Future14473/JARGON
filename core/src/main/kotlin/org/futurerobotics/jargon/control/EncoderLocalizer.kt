package org.futurerobotics.jargon.control

import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.math.Pose2d
import org.futurerobotics.jargon.math.angleNorm
import org.futurerobotics.jargon.math.toPose
import org.futurerobotics.jargon.model.MotorBotGyroInteraction
import org.futurerobotics.jargon.model.MotorBotInteraction

/** Common components of [EncoderLocalizer] and [EncoderGyroLocalizer]. */
abstract class BaseEncoderBasedLocalizer(protected val numMotors: Int, initialPose: Pose2d = Pose2d.ZERO) {

    /**
     * The currently tracked pose.
     */
    var pose: Pose2d = initialPose
        protected set
    /**
     * The previously tracked motor positions.
     */
    protected var pastPositions: Vec = zeroVec(numMotors)

    /**
     * Resets this localizer:
     * Resets the [pose] and the previous [motorPositions], as a vector.
     */
    fun reset(pose: Pose2d, motorPositions: Vec) {
        resetPose(pose)
        resetMotorPositions(motorPositions)
    }

    /** Rests the tracked pose. */
    fun resetPose(pose: Pose2d) {
        this.pose = pose
    }

    /**
     * Resets this previously tracked [motorPositions], as a vector.
     */
    fun resetMotorPositions(motorPositions: Vec) {
        require(motorPositions.size == numMotors)
        { "Motor positions given (${motorPositions.size}) does not reflect model (${numMotors})" }
    }
}

/**
 * A localizer to track global pose based only on encoder measurements.
 *
 * A [MotorBotInteraction] must be supplied.
 *
 * @see EncoderGyroLocalizer
 */
class EncoderLocalizer
@JvmOverloads constructor(
    private val motorBotInteraction: MotorBotInteraction,
    initialPose: Pose2d = Pose2d.ZERO
) : BaseEncoderBasedLocalizer(motorBotInteraction.numMotors, initialPose) {

    /**
     * Given new [motorPositions], uses the delta in motor position to update the currently tracked global [pose]
     * and return it.
     */
    fun update(motorPositions: Vec): Pose2d {
        val diff = motorPositions - pastPositions
        val delta = (motorBotInteraction.botVelFromMotorVel * diff).toPose()
        pastPositions = motorPositions
        return GlobalToBot.trackGlobalPose(delta, pose)
            .also { pose = it }
    }
}

/**
 * A localizer to track global pose based only on encoder measurements.
 *
 * A [MotorBotInteraction] must be supplied.
 *
 * @param useAbsoluteHeading If true, the heading is treated as _the_ global heading, otherwise relative
 *                           differences in heading between [heading] is used.
 * @see EncoderLocalizer
 */
class EncoderGyroLocalizer
@JvmOverloads constructor(
    private val motorBotGyroInteraction: MotorBotGyroInteraction,
    initialPose: Pose2d = Pose2d.ZERO,
    private val useAbsoluteHeading: Boolean = true
) : BaseEncoderBasedLocalizer(motorBotGyroInteraction.numMotors, initialPose) {

    private var lastHeading: Double = 0.0

    /**
     * Resets the heading measurement. This has no effect if [useAbsoluteHeading] is false.
     */
    fun resetHeadingMeasurement(heading: Double) {
        lastHeading = angleNorm(heading)
    }

    /**
     * Given new [motorPositions] and new [heading] measurement, uses the delta in motor position to update the
     * currently tracked global [pose] and return it.
     *
     * @see [useAbsoluteHeading]
     */
    fun update(motorPositions: Vec, heading: Double): Pose2d {
        val normHeading = angleNorm(heading)
        val motorDelta = motorPositions - pastPositions
        val gyroDelta = if (useAbsoluteHeading) {
            angleNorm(normHeading - pose.heading)
        } else {
            angleNorm(normHeading - lastHeading)
                .also { lastHeading = normHeading }
        }
        val botPoseDelta = (motorBotGyroInteraction.botVelFromMotorAndGyroVel * motorDelta.append(gyroDelta)).toPose()
        lastHeading = normHeading
        pastPositions = motorPositions
        return GlobalToBot.trackGlobalPose(botPoseDelta, pose).also { pose = it }
    }
}
