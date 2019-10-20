@file:Suppress("UNCHECKED_CAST")

package org.futurerobotics.jargon.blocks.motion

import org.futurerobotics.jargon.blocks.*
import org.futurerobotics.jargon.blocks.Block.Processing.IN_FIRST_ALWAYS
import org.futurerobotics.jargon.blocks.control.FixedDriveMotorToBotDelta
import org.futurerobotics.jargon.blocks.control.FixedDriveMotorToBotVel
import org.futurerobotics.jargon.math.Pose2d
import org.futurerobotics.jargon.math.notNaNOrElse
import org.futurerobotics.jargon.mechanics.GlobalToBot
import org.futurerobotics.jargon.mechanics.MotionOnly
import org.futurerobotics.jargon.mechanics.MotionState

/**
 * Common parts of [GlobalPoseTrackerFromDelta] and [GlobalPoseTrackerFromVel]
 */
abstract class AbstractGlobalPoseTracker(initialPose: Pose2d, numInputs: Int) : SingleOutputBlock<Pose2d>(
    numInputs, IN_FIRST_ALWAYS
) {
    /** The current pose tracked by this pose tracker */
    protected var currentPose: Pose2d = initialPose
    /** The pose override [BlocksConfig.Input] */
    val poseOverride: BlocksConfig.Input<Pose2d?> get() = configInput(1)

    override fun doInit(): Pose2d? = null
    override fun processOutput(inputs: List<Any?>, systemValues: SystemValues): Pose2d {
        val maybeOverride = inputs[1]
        val newPose = if (maybeOverride != null) maybeOverride as Pose2d else {
            val velocity = inputs[0] as Pose2d
            val poseDelta = toPoseDelta(velocity, systemValues)
            GlobalToBot.trackGlobalPose(poseDelta, currentPose).map(inputs)
        }
        return newPose.also { currentPose = it }
    }

    /** Converts input to pose delta. */
    protected abstract fun toPoseDelta(input: Pose2d, systemValues: SystemValues): Pose2d

    /** Possibly further processes calculated pose to actual estimated pose. */
    protected open fun Pose2d.map(inputs: List<Any?>): Pose2d = this
    override fun prepareAndVerify(config: BlocksConfig): Unit = config.run {
        if (!inputIsConnected(0))
            throw IllegalBlockConfigurationException(
                "Input at ${this@AbstractGlobalPoseTracker.javaClass.simpleName} not connected"
            )
    }
}

/**
 * Non-linearly tracks the **global** pose, given **bot** pose **velocities** (for example from [FixedDriveMotorToBotVel])
 *
 * The [currentPose] can also be overridden using input.
 *
 * Inputs:
 * 1. the _bot's_ velocity in [Pose2d]
 * 2. If connected and not null, manual override of global Pose.
 *
 * Outputs:
 * 1. the currently estimated global pose.
 *
 * @see [GlobalPoseTrackerFromDelta]
 */
class GlobalPoseTrackerFromVel(initialPose: Pose2d = Pose2d.ZERO) : AbstractGlobalPoseTracker(initialPose, 2) {

    /** The velocity [BlocksConfig.Input] */
    val velocityIn: BlocksConfig.Input<Pose2d> get() = configInput(0)

    override fun toPoseDelta(input: Pose2d, systemValues: SystemValues): Pose2d =
        input * systemValues.loopTime.notNaNOrElse { 0.0 }
}

/**
 * Non-linearly tracks the **global** pose, given **bot** pose **delta** (for example from [FixedDriveMotorToBotDelta]).
 *
 * The [currentPose] can also overridden using input.
 *
 * Inputs:
 * 1. the _bot's_ velocity in [Pose2d]
 * 2. If connected and not null, manual override of global Pose.
 *
 * Outputs:
 * 1. the currently estimated global pose.
 *
 * @see GlobalPoseTrackerFromVel
 */
class GlobalPoseTrackerFromDelta(initialPose: Pose2d = Pose2d.ZERO) : AbstractGlobalPoseTracker(initialPose, 2) {
    /** The velocity [BlocksConfig.Input] */
    val deltaIn: BlocksConfig.Input<Pose2d> get() = configInput(0)

    override fun toPoseDelta(input: Pose2d, systemValues: SystemValues): Pose2d = input
}

/**
 * Non-linearly tracks the **global** pose, given **bot** pose **delta** (for example from [FixedDriveMotorToBotDelta]),
 * _and uses gyroscope for heading_.
 *
 * The [currentPose] can also overridden using input.
 *
 * Inputs:
 * 1. the _bot's_ velocity in [Pose2d]
 * 2. If connected and not null, manual override of global Pose.
 *
 * Outputs:
 * 1. the currently estimated global pose.
 *
 * @see GlobalPoseTrackerFromVel
 */
class GlobalPoseTrackerFromDeltaAndGyro(initialPose: Pose2d = Pose2d.ZERO) : AbstractGlobalPoseTracker(initialPose, 3) {
    /** The velocity input */
    val deltaIn: BlocksConfig.Input<Pose2d> get() = configInput(0)
    /** The gyro angle measurement input */
    val gyroIn: BlocksConfig.Input<Double> get() = configInput(2)

    override fun toPoseDelta(input: Pose2d, systemValues: SystemValues): Pose2d = input
    override fun Pose2d.map(inputs: List<Any?>): Pose2d = copy(heading = inputs[2] as Double)
}

/**
 * Converts the [MotionState] of global pose (interpreted as a **reference**) into
 * an equivalent [MotionState] from the bot's perspective (where the bot's current "position" is always Pose2d.ZERO),
 * using the bot's current pose.
 *
 * This is how global reference is translated into bot reference.
 *
 * Inputs:
 * 1. the global pose [MotionState] _reference_.
 * 2. the actual global pose
 *
 * Outputs:
 * 1. [MotionState] from the bot's perspective
 *
 * @see GlobalToBotMotion
 */
class GlobalToBotReference : Combine<MotionState<Pose2d>, Pose2d, MotionState<Pose2d>>(Block.Processing.IN_FIRST_LAZY) {
    override fun combine(a: MotionState<Pose2d>, b: Pose2d): MotionState<Pose2d> = GlobalToBot.referenceMotion(a, b)

    /** The pose reference [BlocksConfig.Input] */
    val referenceIn: BlocksConfig.Input<MotionState<Pose2d>> get() = firstInput
    /** The actual global pose [BlocksConfig.Input] */
    val globalPoseIn: BlocksConfig.Input<Pose2d> get() = secondInput
}

/**
 * Converts the [MotionOnly] of global pose into an equivalent [MotionState] from the bot's perspective using the
 * current global pose,
 *
 * This is how global motion is translated into bot motion.
 *
 * Inputs:
 * 1. the global pose [MotionState] _reference_.
 * 2. the actual global pose
 *
 * Outputs:
 * 1. [MotionState] from the bot's perspective
 *
 * @see GlobalToBotReference
 */
class GlobalToBotMotion :
    Combine<MotionOnly<Pose2d>, Pose2d, MotionOnly<Pose2d>>(Block.Processing.IN_FIRST_LAZY) {
    override fun combine(a: MotionOnly<Pose2d>, b: Pose2d): MotionOnly<Pose2d> = GlobalToBot.motion(a, b.heading)

    /** The pose reference [BlocksConfig.Input] */
    val reference: BlocksConfig.Input<MotionOnly<Pose2d>> get() = firstInput
    /** The actual global pose [BlocksConfig.Input] */
    val globalPose: BlocksConfig.Input<Pose2d> get() = secondInput
}