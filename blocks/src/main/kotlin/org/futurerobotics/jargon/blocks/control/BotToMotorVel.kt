package org.futurerobotics.jargon.blocks.control

import org.futurerobotics.jargon.blocks.PipeBlock
import org.futurerobotics.jargon.blocks.functional.MapMotionOnly
import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.math.Pose2d
import org.futurerobotics.jargon.mechanics.MotionOnly
import org.futurerobotics.jargon.mechanics.MotorBotVelInteraction

/**
 * A [PipeBlock] that converts bot velocities (pose) into wheel velocities (vec), using the given [interaction]
 */
class BotToMotorVel(private val interaction: MotorBotVelInteraction) : PipeBlock<Pose2d, Vec>() {

    override fun pipe(input: Pose2d): Vec = interaction.motorVelFromBotVel * input.toVec()
}

/**
 * A [PipeBlock] that converts bot pose [MotionOnly]  into wheel velocities (vec) [MotionOnly], using the given
 * [interaction].
 */
class BotToMotorMotion(private val interaction: MotorBotVelInteraction) : MapMotionOnly<Pose2d, Vec>() {

    override fun map(value: Pose2d): Vec = interaction.motorVelFromBotVel * value.toVec()
}
