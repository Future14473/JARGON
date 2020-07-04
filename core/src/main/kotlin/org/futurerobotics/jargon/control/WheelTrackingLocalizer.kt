package org.futurerobotics.jargon.control

import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.math.Pose2d
import org.futurerobotics.jargon.math.angleNorm
import org.futurerobotics.jargon.math.toPose
import org.futurerobotics.jargon.model.MotorBotInteraction

/** Common components of [WheelTrackingLocalizer] and [WheelGyroTrackingLocalizer]. */
abstract class BaseWheelTrackingLocalizer(
    initialPose: Pose2d = Pose2d.ZERO
) {

    /**
     * The current localized pose.
     *   Can be manually reset.
     */
    var pose: Pose2d = initialPose

    /** The previously given motor positions. */
    protected var pastPositions: Vec? = null

    /**
     * Resets this localizer.
     *
     * @param pose the initial pose.
     */
    @JvmOverloads
    open fun reset(pose: Pose2d = Pose2d.ZERO) {
        this.pose = pose
        pastPositions = null
    }
}

/**
 * A localizer to track global pose based only on encoder measurements on wheels.
 *
 * A [MotorBotInteraction] must be supplied.
 *
 * @see WheelGyroTrackingLocalizer
 */
class WheelTrackingLocalizer
@JvmOverloads constructor(
    motorBotInteraction: MotorBotInteraction,
    initialPose: Pose2d = Pose2d.ZERO
) : BaseWheelTrackingLocalizer(initialPose) {

    private val botVelFromMotorVel = motorBotInteraction.botVelFromMotorVel

    /**
     * Given new [motorPositions], uses the delta in motor position to update the currently tracked global [pose],
     * and returns it.
     */
    fun update(motorPositions: Vec): Pose2d {
        val pastPositions = pastPositions
        this.pastPositions = motorPositions
        return if (pastPositions == null) pose else {
            val motorDelta = motorPositions - pastPositions
            val botPoseDelta = (botVelFromMotorVel * motorDelta).toPose()
            FieldToBot.trackGlobalPose(botPoseDelta, pose)
                .also { pose = it }
        }
    }
}

/**
 * A localizer to track global pose based on both encoder measurements on wheels and (gyroscope) heading measurements.
 *
 * Note that only the relative difference between heading measurements are used, and the initial pose used to set
 * the actual initial heading (the gyroscope measurement can be a constant offset from the actual heading).
 *
 * A [MotorBotInteraction] must be supplied.
 *
 * @param treatGyroAsActualHeading If true, the heading will always be set to be (a possible constant offset) of the
 *        gyroscope readings, otherwise the wheel position measurements may also be used to determine heading. This only
 *        has a real effect if there are more than 2 wheels.
 * @see WheelTrackingLocalizer
 */
class WheelGyroTrackingLocalizer
@JvmOverloads constructor(
    motorBotInteraction: MotorBotInteraction,
    initialPose: Pose2d = Pose2d.ZERO,
    private val treatGyroAsActualHeading: Boolean = true
) : BaseWheelTrackingLocalizer(initialPose) {

    private var lastHeadingMeasurement: Double = Double.NaN

    private val botVelFromMotorAndGyroVel = kotlin.run {
        val motorAndGyroVelFromBotVel = concatCol(
            motorBotInteraction.motorVelFromBotVel,
            matOf(0, 0, 1)
        )
        motorAndGyroVelFromBotVel.pinv()
    }

    override fun reset(pose: Pose2d) {
        super.reset(pose)
        lastHeadingMeasurement = Double.NaN
    }

    /**
     * Given new [motorPositions] and new [heading] measurement, uses the delta in motor position to update the
     * currently tracked global [pose] and returns it.
     */
    fun update(motorPositions: Vec, heading: Double): Pose2d {
        val normHeading = angleNorm(heading)
        val pastPositions = pastPositions
        val lastHeadingMeasurement = lastHeadingMeasurement
        this.lastHeadingMeasurement = normHeading
        this.pastPositions = pastPositions

        return if (pastPositions == null) pose else {
            val motorDelta = motorPositions - pastPositions
            val headingDelta = angleNorm(normHeading - lastHeadingMeasurement)
            val totalDelta = motorDelta.append(headingDelta)
            val botPoseDelta = (botVelFromMotorAndGyroVel * totalDelta).toPose()
            return FieldToBot.trackGlobalPose(botPoseDelta, pose)
                .let {
                    if (treatGyroAsActualHeading) {
                        val actualHeading = angleNorm(pose.heading + headingDelta)
                        it.copy(heading = actualHeading)
                    } else it
                }
                .also { pose = it }
        }
    }
}
