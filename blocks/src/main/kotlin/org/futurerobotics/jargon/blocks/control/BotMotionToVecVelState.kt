package org.futurerobotics.jargon.blocks.control

import org.futurerobotics.jargon.blocks.PipeBlock
import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.math.MotionOnly
import org.futurerobotics.jargon.math.MotionState
import org.futurerobotics.jargon.math.Pose2d
import org.futurerobotics.jargon.math.ValueMotionState

/**
 * Converts a bot's _pose motion_ signal into a _vector velocity state_.
 *
 * This is a surprisingly common operation.
 */
class BotMotionToVecVelState : PipeBlock<MotionOnly<Pose2d>, MotionState<Vec>>() {

    private val zero = zeroVec(3)
    override fun Context.pipe(input: MotionOnly<Pose2d>): MotionState<Vec> = ValueMotionState(
        input.vel.toVec(),
        input.accel.toVec(),
        zero
    )
}
