package org.futurerobotics.jargon.control

import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.math.Pose2d
import org.futurerobotics.jargon.math.angleNorm
import org.futurerobotics.jargon.math.toPose
import org.futurerobotics.jargon.model.MotorBotInteraction
import org.futurerobotics.jargon.util.replaceIf

/** Common components of [EncoderLocalizer] and [EncoderGyroLocalizer]. */
abstract class BaseEncoderBasedLocalizer(private val numMotors: Int, initialPose: Pose2d = Pose2d.ZERO) {

    /**
     * The current localized pose.
     */
    var pose: Pose2d = initialPose
        protected set
    /** The previously given motor positions. */
    protected var pastPositions: Vec = Vec(numMotors)

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
     * Resets the [motorPositions], as a vector.
     */
    fun resetMotorPositions(motorPositions: Vec) {
        require(motorPositions.size == numMotors)
        { "Motor positions given (${motorPositions.size}) does not reflect model (${numMotors})" }
        this.pastPositions = motorPositions
    }
}

/**
 * A localizer to track global pose based only on encoder measurements on wheels.
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
     * Given new [motorPositions], uses the delta in motor position to update the currently tracked global [pose],
     * and returns it.
     */
    fun update(motorPositions: Vec): Pose2d {
        val motorDelta = motorPositions - pastPositions
        pastPositions = motorPositions
        val botDelta = (motorBotInteraction.botVelFromMotorVel * motorDelta).toPose()
        return GlobalToBot.trackGlobalPose(botDelta, pose)
            .also { pose = it }
    }
}

/**
 * A localizer to track global pose based on both encoder measurements on wheels and gyroscope readings.
 *
 * A [MotorBotInteraction] must be supplied.
 *
 * @param useRelativeHeading If true, the heading is treated as _the_ global heading, otherwise relative
 *                           differences in heading between [heading] is used.
 * @param treatGyroAsActualHeading If true, the gyroscope's values will always be used as the heading measurement,
 *                          as opposed to also using the encoder measurements to estimate heading.
 * @see EncoderLocalizer
 */
class EncoderGyroLocalizer
@JvmOverloads constructor(
    motorBotInteraction: MotorBotInteraction,
    initialPose: Pose2d = Pose2d.ZERO,
    private val useRelativeHeading: Boolean = false,
    private val treatGyroAsActualHeading: Boolean = true
) : BaseEncoderBasedLocalizer(motorBotInteraction.numMotors, initialPose) {

    private var lastHeading: Double = 0.0

    private val botVelFromMotorAndGyroVel = kotlin.run {
        val motorAndGyroVelFromBotVel = concatCol(
            motorBotInteraction.motorVelFromBotVel,
            matOf(0, 0, 1)
        )
        motorAndGyroVelFromBotVel.pinv()
    }

    /**
     * Resets the heading measurement. This has no effect if [useRelativeHeading] is false.
     */
    fun resetHeadingMeasurement(heading: Double) {
        lastHeading = angleNorm(heading)
    }

    /**
     * Given new [motorPositions] and new [heading] measurement, uses the delta in motor position to update the
     * currently tracked global [pose] and return it.
     *
     * @see [useRelativeHeading]
     */
    fun update(motorPositions: Vec, heading: Double): Pose2d {
        val normHeading = angleNorm(heading)
        val motorDelta = motorPositions - pastPositions
        pastPositions = motorPositions
        val gyroDelta = if (useRelativeHeading) {
            angleNorm(normHeading - lastHeading)
                .also { lastHeading = normHeading }
        } else {
            angleNorm(normHeading - pose.heading)
        }
        val botPoseDelta = (botVelFromMotorAndGyroVel * motorDelta.append(gyroDelta)).toPose()
        return GlobalToBot.trackGlobalPose(botPoseDelta, pose).replaceIf(treatGyroAsActualHeading) {
            val actualHeading = angleNorm(pose.heading + gyroDelta)
            it.copy(heading = actualHeading)
        }.also { pose = it }
    }
}
