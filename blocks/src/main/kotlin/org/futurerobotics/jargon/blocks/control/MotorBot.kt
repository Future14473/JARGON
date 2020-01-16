package org.futurerobotics.jargon.blocks.control

import org.futurerobotics.jargon.blocks.Block
import org.futurerobotics.jargon.blocks.PipeBlock
import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.math.Pose2d
import org.futurerobotics.jargon.math.angleNorm
import org.futurerobotics.jargon.math.toPose
import org.futurerobotics.jargon.model.MotorBotInteraction

/**
 * A [PipeBlock] that takes in motor _positions_, calculates the difference, and then estimate _bot_ pose delta using
 * a drive [interaction].
 *
 * Maybe pass through a filter first.
 */
class MotorToBotDelta(private val interaction: MotorBotInteraction) : PipeBlock<Vec, Pose2d>(Processing.ALWAYS) {

    /** Motor positions input. */
    val motorPositions: Input<Vec> get() = super.input
    /** Bot delta output. */
    val botDelta: Output<Pose2d> get() = super.output

    private var pastPositions: Vec? = null
    override fun Context.pipe(input: Vec): Pose2d {
        val curPositions = input.copy()

        val pastPositions = pastPositions
        this@MotorToBotDelta.pastPositions = curPositions
        return if (pastPositions == null) Pose2d.ZERO else {
            interaction.botVelFromMotorVel(curPositions - pastPositions).toPose()
        }
    }

    override fun stop() {
        pastPositions = null
    }
}

/**
 * A block that takes in motor _positions_, and gyroscope measurements as the `true` gyro reading,
 * and estimates _bot_ pose delta using a drive [interaction].
 *
 * This will produce the same values if the values the gyro produces is offset by a constant value.
 */
class MotorAndGyroToBotDelta(private val interaction: MotorBotInteraction) : Block(Processing.ALWAYS) {

    /** Motor positions input. */
    val motorPositions: Input<Vec> = newInput()
    /** Gyroscope heading measurement input */
    val headingMeasurement: Input<Double> = newInput()
    /** Bot delta output. */
    val botDelta: Output<Pose2d> = newOutput()

    private var pastPositions: Vec? = null
    private var pastAngle: Double = Double.NaN

    override fun Context.process() {
        val curPositions = motorPositions.get.copy()
        val curAngle = headingMeasurement.get

        val pastPositions = pastPositions
        val pastAngle = pastAngle
        this@MotorAndGyroToBotDelta.pastPositions = curPositions
        this@MotorAndGyroToBotDelta.pastAngle = curAngle

        botDelta.set = if (pastPositions == null) Pose2d.ZERO else {
            interaction.botVelFromMotorVel(curPositions - pastPositions)
                .also { it[2] = angleNorm(curAngle - pastAngle) }.toPose()
        }
    }

    override fun stop() {
        pastPositions = null
        pastAngle = Double.NaN
    }
}

/**
 * A block that takes in motor _velocities_ to estimate _bot pose velocity_, using a drive [interaction].
 *
 * Maybe pass through a filter first.
 */
class MotorToBotVel(private val interaction: MotorBotInteraction) : PipeBlock<Vec, Pose2d>(Processing.LAZY) {

    override fun Context.pipe(input: Vec): Pose2d =
        (interaction.botVelFromMotorVel * input).toPose()
}

/**
 * A [PipeBlock] that converts bot velocities (pose) into wheel velocities (vec), using the given [interaction]
 */
class BotToMotorVel(private val interaction: MotorBotInteraction) : PipeBlock<Pose2d, Vec>() {

    override fun Context.pipe(input: Pose2d): Vec = interaction.motorVelFromBotVel * input.toVec()
}
