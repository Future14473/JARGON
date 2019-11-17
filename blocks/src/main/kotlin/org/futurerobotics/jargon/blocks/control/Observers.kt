package org.futurerobotics.jargon.blocks.control

import org.futurerobotics.jargon.blocks.Block.Processing.LAZY
import org.futurerobotics.jargon.blocks.PipeBlock
import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.math.Pose2d
import org.futurerobotics.jargon.mechanics.MotorBotVelInteraction

/**
 * A [PipeBlock] that takes in motor _positions_, calculates the difference, and then estimate _bot_ pose delta using
 * a drive [interaction].
 *
 * Maybe pass through a filter first.
 */
class MotorToBotDelta(private val interaction: MotorBotVelInteraction) : PipeBlock<Vec, Pose2d>(LAZY) {

    private var pastPositions: Vec? = null
    override fun Context.pipe(input: Vec): Pose2d {
        val pastPositions = pastPositions
        val curPositions = input.copy()
        this@MotorToBotDelta.pastPositions = curPositions
        return if (pastPositions == null) Pose2d.ZERO else {
            Pose2d(interaction.botVelFromMotorVel(curPositions - pastPositions))
        }
    }
}

/**
 * A component that takes in motor _velocities_ to estimate _bot pose velocity_, using a drive [interaction].
 *
 * Maybe pass through a filter first.
 */
class MotorToBotVel(private val interaction: MotorBotVelInteraction) : PipeBlock<Vec, Pose2d>(LAZY) {

    override fun Context.pipe(input: Vec): Pose2d =
        Pose2d((interaction.botVelFromMotorVel * input))
}
