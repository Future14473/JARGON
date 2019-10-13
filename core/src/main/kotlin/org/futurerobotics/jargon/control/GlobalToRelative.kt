@file:Suppress("UNCHECKED_CAST")

package org.futurerobotics.jargon.control

import org.futurerobotics.jargon.control.Block.Processing.IN_FIRST_ALWAYS
import org.futurerobotics.jargon.math.Pose2d
import org.futurerobotics.jargon.math.Vector2d
import org.futurerobotics.jargon.math.cosc
import org.futurerobotics.jargon.math.sinc
import org.futurerobotics.jargon.mechanics.GlobalToBot
import org.futurerobotics.jargon.mechanics.MotionState

/**
 * Non-linearly tracks the _global_ pose, given _bot_ pose velocities.
 *
 * The [currentPose] can also be set manually.
 *
 * Inputs:
 * 1. the _bot's_ velocity in [Pose2d]
 * 2. If connected and not null, manual override of global Pose.
 * 3. the loop time (auto connected)
 *
 * Outputs:
 * 1. the currently estimated global pose.
 */
class GlobalPoseTracker(initialPose: Pose2d = Pose2d.ZERO) : SingleOutputBlock<Pose2d>(2, IN_FIRST_ALWAYS) {
    private var currentPose: Pose2d = initialPose

    override fun doInit(): Pose2d? = null

    override fun getOutput(inputs: List<Any?>): Pose2d {
        val maybeOverride = inputs[1]
        currentPose = if (maybeOverride != null) maybeOverride as Pose2d else {
            val velocity = inputs[0] as Pose2d
            //TODO
            val (v, dTheta) = velocity * elapsedSeconds
            val (x, y) = v
            val sinc = sinc(dTheta)
            val cosc = cosc(dTheta)
            val relativeDiff = Vector2d(sinc * x - cosc * y, cosc * x + sinc * y)
            val dPose = Pose2d(relativeDiff.rotated(currentPose.heading), dTheta)
            (currentPose + dPose).normalizeAngle()
        }
        return currentPose
    }

    override fun prepareAndVerify(config: BlocksConfig): Unit = config.run {
        repeat(2) {
            if (!inputIndex<Any>(0).isConnected()) throw IllegalBlockConfigurationException()
        }
    }

    /** The velocity [BlockInput] */
    val velocityIn: BlockInput<Pose2d> get() = inputIndex(0)
    /** The pose override [BlockInput] */
    val poseOverride: BlockInput<Pose2d?> get() = inputIndex(1)
}


/**
 * Converts the [MotionState] of global pose (interpreted as a **reference**) using the current global pose, into
 * an equivalent [MotionState] from the bot's perspective (where the bot's current "position" is always Pose2d.ZERO).
 * This is how global motion is translated into bot motion.
 *
 * Inputs:
 * 1. the global pose [MotionState] _reference_.
 * 2. the actual global pose
 *
 * Outputs:
 * 1. [MotionState] from the bot's perspective
 */
class GlobalToBotReference : Combine<MotionState<Pose2d>, Pose2d, MotionState<Pose2d>>() {
    override fun combine(a: MotionState<Pose2d>, b: Pose2d): MotionState<Pose2d> = GlobalToBot.referenceMotion(a, b)

    /** The pose reference [BlockInput] */
    val referenceIn: BlockInput<MotionState<Pose2d>> get() = first
    /** The actual global pose [BlockInput] */
    val globalPoseIn: BlockInput<Pose2d> get() = second
}