package org.futurerobotics.jargon.blocks.control

import org.futurerobotics.jargon.blocks.Block
import org.futurerobotics.jargon.blocks.BlockArrangementBuilder
import org.futurerobotics.jargon.blocks.functional.Pass
import org.futurerobotics.jargon.math.MotionOnly
import org.futurerobotics.jargon.math.MotionState
import org.futurerobotics.jargon.math.Pose2d

/**
 * A [BotPoseController] that uses internally a [PosePidController] for the desired motion of the bot, plus
 * feed-forward, and then mapping from _global_ motion to _bot_ motion.
 */
class HolonomicPidBotPoseController(
    builder: BlockArrangementBuilder,
    xCoeff: PidCoefficients,
    yCoeff: PidCoefficients,
    headingCoeff: PidCoefficients
) : BotPoseController {

    override val reference: Block.Input<MotionState<Pose2d>>
    override val state: Block.Input<Pose2d>
    override val signal: Block.Output<MotionOnly<Pose2d>>

    init {
        with(builder) {
            val pass = Pass<Pose2d>()
            state = pass.input
            val theState = pass.output

            val poseController = FeedForwardWrapper.withAdder(
                PosePidController(xCoeff, yCoeff, headingCoeff),
                Pose2d::plus
            ).apply {
                state from theState
            }
            reference = poseController.reference

            val globalSignal = poseController.signal
            val globalToBot = GlobalToBotMotion().apply {
                globalMotion from globalSignal
                globalPose from theState
            }
            signal = globalToBot.botMotion
        }
    }
}
