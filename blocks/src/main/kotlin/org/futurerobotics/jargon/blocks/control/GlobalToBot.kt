package org.futurerobotics.jargon.blocks.control

import org.futurerobotics.jargon.blocks.Block
import org.futurerobotics.jargon.blocks.Block.Processing.LAZY
import org.futurerobotics.jargon.blocks.PrincipalOutputBlock
import org.futurerobotics.jargon.control.GlobalToBot
import org.futurerobotics.jargon.math.MotionOnly
import org.futurerobotics.jargon.math.MotionState
import org.futurerobotics.jargon.math.Pose2d

/**
 * Converts the [MotionState] of global pose (interpreted as a **reference**) into
 * an equivalent [MotionState] from the bot's perspective (where the bot's current "position" is always Pose2d.ZERO),
 * using the bot's current pose.
 *
 * This is how global reference is translated into bot reference.
 *
 * @see GlobalToBotMotion
 */
class GlobalToBotReference : Block(LAZY) {

    /** The pose reference input */
    val globalState: Input<MotionState<Pose2d>> = newInput()
    /** The actual global pose input */
    val globalPose: Input<Pose2d> = newInput()
    /** The bot reference output */
    val botReference: Output<MotionState<Pose2d>> = newOutput()

    override fun Context.process() {
        botReference.set = GlobalToBot.referenceMotion(
            globalState.get,
            globalPose.get
        )
    }
}

/**
 * Converts the [MotionOnly] of global pose into an equivalent [MotionState] from the bot's perspective using the
 * current global pose,
 *
 * This is how global motion is translated into bot motion.
 * @see GlobalToBotReference
 */
class GlobalToBotMotion : PrincipalOutputBlock<MotionOnly<Pose2d>>(LAZY) {

    /** The pose reference input */
    val globalMotion: Input<MotionOnly<Pose2d>> = newInput()
    /** The global pose input */
    val globalPose: Input<Pose2d> = newInput()

    /** The bot reference output */
    val botMotion: Output<MotionOnly<Pose2d>> get() = output

    override fun Context.getOutput(): MotionOnly<Pose2d> =
        GlobalToBot.motion(globalMotion.get, globalPose.get.heading)
}
