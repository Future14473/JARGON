@file:Suppress("UNCHECKED_CAST")

package org.futurerobotics.jargon.control

import org.futurerobotics.jargon.control.Block.InOutOrder.IN_FIRST
import org.futurerobotics.jargon.control.Block.Processing.ALWAYS
import org.futurerobotics.jargon.math.Pose2d
import org.futurerobotics.jargon.math.Vector2d
import org.futurerobotics.jargon.math.cosc
import org.futurerobotics.jargon.math.sinc
import org.futurerobotics.jargon.mechanics.GlobalToBot
import org.futurerobotics.jargon.mechanics.MotionState3

/**
 * Non-linearly tracks the _global_ pose, given _bot_ pose velocities.
 *
 * The [currentPose] can also be set manually.
 *
 * Inputs:
 * 1. the _bot's_ velocity in [Pose2d]
 * 2. the loop time.
 * 3. If connected and not null, manual override of global Pose.
 *
 * Outputs:
 * 1. the currently estimated global pose.
 */
class GlobalPoseTracker(initialPose: Pose2d = Pose2d.ZERO) : AbstractBlock(3, 1, IN_FIRST, ALWAYS) {
    private var currentPose: Pose2d = initialPose


    override fun process(inputs: List<Any?>, outputs: MutableList<Any?>) {
        val maybeOverride = inputs[2]
        if (maybeOverride != null) {
            currentPose = maybeOverride as Pose2d
            return
        }
        val velocity = inputs[0] as Pose2d
        val elapsedSeconds = inputs[1] as Double
        val (v, dTheta) = velocity * elapsedSeconds
        val (x, y) = v
        val sinc = sinc(dTheta)
        val cosc = cosc(dTheta)
        val relativeDiff = Vector2d(sinc * x - cosc * y, cosc * x + sinc * y)
        val dPose = Pose2d(relativeDiff.rotated(currentPose.heading), dTheta)
        currentPose = (currentPose + dPose).normalizeAngle()
        outputs[0] = currentPose
    }

    override fun init(outputs: MutableList<Any?>) {
    }
}


/**
 * Converts the [MotionState3] of global pose (interpreted as a reference) using the current global pose, into
 * an equivalent [MotionState3] from the bot's perspective (where the bot's current "position" is always Pose2d.ZERO).
 * This is how global motion is translated into bot motion.
 *
 * Inputs:
 * 1. the current pose [MotionState3]/[MotionState2] of the pose, or the current pose.
 * 2. the global pose
 *
 * Outputs:
 * 1. [MotionState3] from the bot's perspective
 */
class GlobalToBotReference : CombineBlock<Any, Pose2d, MotionState3<Pose2d>>() {
    override fun combine(a: Any, b: Pose2d): MotionState3<Pose2d> {
        return GlobalToBot.referenceMotion(a.toMotionState3(Pose2d.ZERO), b)
    }
}