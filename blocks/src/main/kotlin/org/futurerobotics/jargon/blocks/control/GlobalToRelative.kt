package org.futurerobotics.jargon.blocks.control

import org.futurerobotics.jargon.blocks.BaseBlock
import org.futurerobotics.jargon.blocks.Block.Processing.ALWAYS
import org.futurerobotics.jargon.blocks.Block.Processing.LAZY
import org.futurerobotics.jargon.blocks.SingleOutputBlock
import org.futurerobotics.jargon.math.Pose2d
import org.futurerobotics.jargon.math.notNaNOrElse
import org.futurerobotics.jargon.mechanics.GlobalToBot
import org.futurerobotics.jargon.mechanics.MotionOnly
import org.futurerobotics.jargon.mechanics.MotionState

/**
 * Common parts of [GlobalPoseTrackerFromDelta] and [GlobalPoseTrackerFromVel]
 */
abstract class AbstractGlobalPoseTracker(initialPose: Pose2d) : SingleOutputBlock<Pose2d>(ALWAYS) {

    /** The pose override input */
    val poseOverride: Input<Pose2d?> = newInput(true)
    /** The currently tracked global pose. */
    val globalPose: Output<Pose2d> get() = super.output

    /** The current pose tracked by this pose tracker */
    protected var currentPose: Pose2d = initialPose

    override fun Context.getOutput(): Pose2d {
        val maybeOverride = poseOverride.get
        val newPose = if (maybeOverride != null) maybeOverride else {
            val poseDelta = getPoseDelta()
            mapPose(GlobalToBot.trackGlobalPose(poseDelta, currentPose))
        }
        currentPose = newPose
        return newPose
    }

    /** Converts input to pose delta. */
    protected abstract fun Context.getPoseDelta(): Pose2d

    /** Possibly further processes calculated pose to actual estimated pose. */
    protected open fun Context.mapPose(pose: Pose2d): Pose2d = pose
}

/**
 * Non-linearly tracks the _global_ pose, given _bot_ pose **velocities** (for example from [FixedDriveMotorToBotVel])
 *
 * The [currentPose] can also be overridden using input.
 * @see [GlobalPoseTrackerFromDelta]
 */
class GlobalPoseTrackerFromVel(initialPose: Pose2d = Pose2d.ZERO) : AbstractGlobalPoseTracker(initialPose) {

    /** The velocity input */
    val velocityIn: Input<Pose2d> = newInput()

    override fun Context.getPoseDelta(): Pose2d =
        velocityIn.get * loopTime.notNaNOrElse { 0.0 }
}

/**
 * Non-linearly tracks the **global** pose, given **bot** pose **delta** (for example from [FixedDriveMotorToBotDelta]).
 *
 * The [currentPose] can also overridden using input.
 *
 * @see GlobalPoseTrackerFromVel
 */
open class GlobalPoseTrackerFromDelta(initialPose: Pose2d = Pose2d.ZERO) : AbstractGlobalPoseTracker(initialPose) {

    /** The velocity input */
    val deltaIn: Input<Pose2d> = newInput()

    override fun Context.getPoseDelta(): Pose2d = deltaIn.get
}

/**
 * Non-linearly tracks the _global__ pose, given _bot_ pose **delta** (for example from [FixedDriveMotorToBotDelta]),
 * _and uses gyroscope for heading_.
 *
 * The [currentPose] can also overridden using input.
 * @see GlobalPoseTrackerFromVel
 */
class GlobalPoseTrackerFromDeltaAndGyro(initialPose: Pose2d = Pose2d.ZERO) : GlobalPoseTrackerFromDelta(initialPose) {

    /** The gyro angle measurement input */
    val gyroIn: Input<Double> = newInput()

    override fun Context.getPoseDelta(): Pose2d = deltaIn.get
    override fun Context.mapPose(
        pose: Pose2d
    ): Pose2d = pose.copy(heading = gyroIn.get)
}

/**
 * Converts the [MotionState] of global pose (interpreted as a **reference**) into
 * an equivalent [MotionState] from the bot's perspective (where the bot's current "position" is always Pose2d.ZERO),
 * using the bot's current pose.
 *
 * This is how global reference is translated into bot reference.
 *
 * @see GlobalToBotMotion
 */
class GlobalToBotReference : BaseBlock(LAZY) {

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
class GlobalToBotMotion : SingleOutputBlock<MotionOnly<Pose2d>>(LAZY) {

    /** The pose reference input */
    val globalMotion: Input<MotionOnly<Pose2d>> = newInput()
    /** The global pose input */
    val globalPose: Input<Pose2d> = newInput()

    /** The bot reference output */
    val botMotion: Output<MotionOnly<Pose2d>> get() = output

    override fun Context.getOutput(): MotionOnly<Pose2d> =
        GlobalToBot.motion(globalMotion.get, globalPose.get.heading)
}
