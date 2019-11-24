package org.futurerobotics.jargon.blocks.control

import org.futurerobotics.jargon.blocks.Block
import org.futurerobotics.jargon.blocks.BlockIndicator
import org.futurerobotics.jargon.math.MotionOnly
import org.futurerobotics.jargon.math.MotionState
import org.futurerobotics.jargon.math.Pose2d

/**
 * A block that represents a controller:
 * taking in a [Reference], comparing it with [State], and producing a [Signal].
 *
 * This interface exists so that it can be recognized by other things.
 *
 * @see BotPoseController
 */
interface Controller<Reference, State, Signal> : BlockIndicator {

    /** The reference input. */
    val reference: Block.Input<Reference>
    /** The state input. */
    val state: Block.Input<State>
    /** The signal output. */
    val signal: Block.Output<Signal>
}

/**
 * An interface to represent a bot pose [Controller]:
 *
 * given a [MotionState] of [Pose2d], in the **global** reference
 * frame as [reference], and the current (measured/estimated) global pose as [state], produces a target **bot**
 * [MotionOnly] of [Pose2d] (bot velocity and acceleration).
 *
 * @see HolonomicPidBotPoseController
 */
interface BotPoseController : Controller<MotionState<Pose2d>, Pose2d, MotionOnly<Pose2d>>
