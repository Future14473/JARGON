package org.futurerobotics.jargon.blocks.control

import org.futurerobotics.jargon.blocks.Block.Processing.LAZY
import org.futurerobotics.jargon.blocks.PipeBlock
import org.futurerobotics.jargon.blocks.SingleOutputBlock
import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.math.Pose2d
import org.futurerobotics.jargon.mechanics.MotorBotVelInteraction

/**
 * A [PipeBlock] that takes in motor _positions_, calculates the difference, and then estimate _bot_ pose delta using
 * a drive [interaction].
 *
 * Maybe pass through a filter first.
 */
class FixedDriveMotorToBotDelta(private val interaction: MotorBotVelInteraction) :
    PipeBlock<List<Double>, Pose2d>(LAZY) {

    private var pastPositions: Vec? = null
    override fun Context.pipe(
        input: List<Double>
    ): Pose2d {
        val pastPositions = pastPositions
        val curPositions = input.toVec()
        this@FixedDriveMotorToBotDelta.pastPositions = curPositions
        return if (pastPositions == null) Pose2d.ZERO else {
            Pose2d(interaction.botVelFromMotorVel(curPositions - pastPositions))
        }
    }
}

/**
 * A [SingleOutputBlock] block that takes in motor _positions_ and gyro readings to estimate _bot_ pose delta,
 * using a drive [interaction].
 *
 * Maybe pass through a filter first.
 */
class FixedDriveMotorAndGyroToBotDelta(private val interaction: MotorBotVelInteraction) :
    SingleOutputBlock<Pose2d>(LAZY) {

    /** Motor positions input */
    val motorPositions: Input<List<Double>> = newInput()
    /** Gyroscope input */
    val gyroAngle: Input<Double> = newInput()

    private var pastPositions: Vec? = null
    private var pastAngle: Double = Double.NaN

    override fun Context.getOutput(): Pose2d {
        val a = motorPositions.get
        val b = gyroAngle.get
        val pastPositions = pastPositions
        val pastAngle = pastAngle
        val curPositions = a.toVec()
        this@FixedDriveMotorAndGyroToBotDelta.pastPositions = curPositions
        this@FixedDriveMotorAndGyroToBotDelta.pastAngle = b
        return if (pastPositions == null) Pose2d.ZERO else {
            Pose2d(interaction.botVelFromMotorVel * (curPositions - pastPositions)).copy(heading = pastAngle - b)
        }
    }
}

/**
 * A component that takes in motor _velocities_ to estimate _bot pose velocity_, using a drive [interaction].
 *
 * Maybe pass through a filter first.
 */
class FixedDriveMotorToBotVel(private val interaction: MotorBotVelInteraction) :
    PipeBlock<List<Double>, Pose2d>(LAZY) {

    override fun Context.pipe(
        input: List<Double>
    ): Pose2d =
        Pose2d((interaction.botVelFromMotorVel * input.toVec()))
}
