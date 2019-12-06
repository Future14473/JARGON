package org.futurerobotics.jargon.blocks.control

import org.futurerobotics.jargon.blocks.Block
import org.futurerobotics.jargon.blocks.BlockArrangementBuilder
import org.futurerobotics.jargon.blocks.PrincipalOutputBlock
import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.math.Pose2d
import org.futurerobotics.jargon.math.ifNan
import org.futurerobotics.jargon.mechanics.GlobalToBot
import org.futurerobotics.jargon.mechanics.MotorBotVelInteraction

/**
 * A [PoseLocalizer] represents a block with some method of tracking the [globalPose].
 *
 * @see BotDeltaLocalizer
 * @see EncoderOnlyLocalizer
 * @see EncoderAndStrictGyroLocalizer
 */
interface PoseLocalizer {

    /** The current tracked/estimated [globalPose]. */
    val globalPose: Block.Output<Pose2d>

    /**
     * When given a non-null value, overrides the current pose.
     */
    val poseOverride: Block.Input<Pose2d?>
}

/** Common parts of a localizer based off of bot delta only */
abstract class DeltaBasedLocalizer(initialPose: Pose2d) : PrincipalOutputBlock<Pose2d>(Processing.ALWAYS),
                                                          PoseLocalizer {

    /** The currently tracked global pose. */
    override val globalPose: Output<Pose2d> get() = super.output
    /** The pose override input */
    override val poseOverride: Input<Pose2d?> = newOptionalInput()

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
 * Non-linearly tracks the **global** pose, given **bot** pose **velocities** (for example from
 * [MotorToBotVel])
 *
 * The [currentPose] can also be overridden using input.
 * @see [BotDeltaLocalizer]
 */
class BotVelLocalizer(initialPose: Pose2d = Pose2d.ZERO) : DeltaBasedLocalizer(initialPose) {

    /** The velocity input */
    val botVelocity: Input<Pose2d> = newInput()

    override fun Context.getPoseDelta(): Pose2d = botVelocity.get * loopTime.ifNan { 0.0 }
}

/**
 * Non-linearly tracks the **global** pose, given **bot** pose **delta** (for example from [MotorToBotDelta]).
 *
 * The [currentPose] can also overridden using input.
 *
 * @see BotVelLocalizer
 * @see BotDeltaAndGyroLocalizer
 */
open class BotDeltaLocalizer(initialPose: Pose2d = Pose2d.ZERO) : DeltaBasedLocalizer(initialPose) {

    /** The velocity input */
    val botDelta: Input<Pose2d> = newInput()

    override fun Context.getPoseDelta(): Pose2d = botDelta.get
}

/**
 * Non-linearly tracks the **global** pose, given _bot_ pose **delta** (for example from [MotorToBotDelta]),
 * and uses gyroscope directly for heading. Note that this is prone to gyroscope drift.
 *
 * @see BotVelLocalizer
 * @see BotDeltaLocalizer
 * @see BotDeltaAndGyroLocalizer
 */
class BotDeltaAndGyroLocalizer(initialPose: Pose2d = Pose2d.ZERO) : BotDeltaLocalizer(initialPose) {

    /** The gyro angle measurement input */
    val gyroReading: Input<Double> = newInput()

    override fun Context.getPoseDelta(): Pose2d = botDelta.get
    override fun Context.mapPose(pose: Pose2d): Pose2d = pose.copy(heading = gyroReading.get)
}

/**
 * A [PoseLocalizer] block group that only uses bot encoder measurements to estimate pose.
 */
class EncoderOnlyLocalizer(
    builder: BlockArrangementBuilder,
    interaction: MotorBotVelInteraction
) : PoseLocalizer {

    /** Constructor for [EncoderOnlyLocalizer] that also connects now. */
    constructor(
        builder: BlockArrangementBuilder,
        interaction: MotorBotVelInteraction,
        motors: MotorsBlock
    ) : this(builder, interaction) {
        builder.connect(motorPositions, motors.motorPositions)
    }

    override val poseOverride: Block.Input<Pose2d?>
    override val globalPose: Block.Output<Pose2d>
    /** Motor positions input (from encoders). */
    val motorPositions: Block.Input<Vec>

    init {
        with(builder) {
            val deltaGetter = MotorToBotDelta(interaction)
            motorPositions = deltaGetter.motorPositions

            val delta = deltaGetter.botDelta
            val tracker = BotDeltaLocalizer().apply {
                botDelta from delta
            }
            poseOverride = tracker.poseOverride
            globalPose = tracker.globalPose
        }
    }
}

/**
 * A [PoseLocalizer] that uses encoders to estimate translational velocities, but uses a gyroscope as
 * _THE_ heading. This might cause problems if your gyro starts to drift.
 */
class EncoderAndStrictGyroLocalizer(
    builder: BlockArrangementBuilder, interaction: MotorBotVelInteraction
) : PoseLocalizer {

    constructor(
        builder: BlockArrangementBuilder,
        interaction: MotorBotVelInteraction,
        motors: MotorsBlock,
        gyroBlock: GyroBlock
    ) : this(builder, interaction) {
        builder.connect(motorPositions, motors.motorPositions)
        builder.connect(headingMeasurement, gyroBlock.headingMeasurement)
    }

    override val poseOverride: Block.Input<Pose2d?>
    override val globalPose: Block.Output<Pose2d>
    /** Motor positions input (from encoders). */
    val motorPositions: Block.Input<Vec>

    /** Heading measurement input (from gyro). */
    val headingMeasurement: Block.Input<Double>

    init {
        with(builder) {
            val deltaGetter = MotorAndGyroToBotDelta(interaction)
            motorPositions = deltaGetter.motorPositions
            headingMeasurement = deltaGetter.headingMeasurement

            val delta = deltaGetter.botDelta
            val tracker = BotDeltaLocalizer().apply {
                botDelta from delta
            }
            poseOverride = tracker.poseOverride
            globalPose = tracker.globalPose
        }
    }
}
