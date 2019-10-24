@file:Suppress("UNCHECKED_CAST")

package org.futurerobotics.jargon.blocks.motion

import org.futurerobotics.jargon.blocks.AbstractBlock
import org.futurerobotics.jargon.blocks.Block.Processing.IN_FIRST_ALWAYS
import org.futurerobotics.jargon.blocks.BlocksConfig
import org.futurerobotics.jargon.blocks.IllegalBlockConfigurationException
import org.futurerobotics.jargon.blocks.SystemValues
import org.futurerobotics.jargon.profile.MotionProfiled
import org.futurerobotics.jargon.util.Stepper

/**
 * Base class for implementing a block that follows a motion profiled path.
 *
 * When a motion profile is done following, the follower will stop on the last output of the previous
 * profile.
 *
 * Inputs:
 * - [profileInput]: The [MotionProfiled] object to follow, or null to indicate to idling at the last reference given.
 *      WILL ONLY BE POLLED upon reaching end of the previous motion profile, or input #2 is pulsed:
 * - [stop] to stop following motion profile. When given `true`, will immediately cancel following the
 *      current motion profile and the next profile will be polled, if any. For example, using [Pulse].
 * Subclasses may specify other inputs, starting with #3.
 *
 * Outputs:
 * - [output] current output of the motion profile.
 * - [progress] The current progress along motion profile as a number from 0 to 1.
 * - [isFollowing] true if this follower is following a profile, false if idling.
 * Subclasses may specify other outputs, starting with #4.
 *
 * @param initialOutput the initial output if the system is idle and no motion profiled has been given
 *              yet.
 */
abstract class MotionProfileFollower<T : Any>(numInputs: Int, numOutputs: Int, private val initialOutput: T) :
    AbstractBlock(numInputs, numOutputs, IN_FIRST_ALWAYS) {

    private var outputValue: T = initialOutput
    private var currentTime: Double = 0.0
    private var endTime: Double = 1.0
    private var currentStepper: Stepper<Double, T>? = null //if null; means poll more.

    init {
        require(numInputs >= 2) { "NumInputs should be >= 2" }
        require(numOutputs >= 3) { "NumOutputs should be >= 3" }
    }

    final override fun init() {
        outputValue = initialOutput
        currentTime = 0.0
        endTime = 1.0
        currentStepper = null
    }

    final override fun process(inputs: List<Any?>, systemValues: SystemValues) {
        var currentStepper = currentStepper
        if (inputs[1] as Boolean? == true || currentStepper == null) {//always poll inputs[1]; so doesn't store up
            val newProfiledMaybe = inputs[0] ?: return
            val newProfiled = newProfiledMaybe as MotionProfiled<T>
            currentTime = 0.0
            endTime = newProfiled.duration
            currentStepper = newProfiled.stepper()
            this.currentStepper = currentStepper
        } else {
            currentTime = getNextTime(currentTime, outputValue, inputs, systemValues)
        }
        if (currentTime >= endTime) {
            currentTime = endTime
            this.currentStepper = null
        }
        outputValue = currentStepper.stepTo(currentTime)

        processFurther(inputs)
    }

    /** Performs any additional possible processing. */
    protected abstract fun processFurther(inputs: List<Any?>)

    /**
     * Gets the next time to use to get the value out of the [MotionProfiled] object;
     *
     * given the [currentTime] along the profile, the [lastOutput]ed value, and possible additional
     * [inputs]. If this component specifies it (Please do not to access inputs (0-1) or things may break)
     *
     * Following the motion profile ends if [getNextTime] returns a time longer past the current motion profiled's
     * duration.
     */
    protected abstract fun getNextTime(
        currentTime: Double, lastOutput: Any, inputs: List<Any?>, systemValues: SystemValues
    ): Double

    override fun getOutput(index: Int): Any? = when (index) {
        !in 0..numOutputs -> IndexOutOfBoundsException(index)
        0 -> outputValue
        1 -> currentTime / endTime
        2 -> currentStepper != null
        else -> getMoreOutput(index)
    }

    /** Gets any additional possible outputs, starting with index 2. */
    protected abstract fun getMoreOutput(index: Int)

    override fun prepareAndVerify(config: BlocksConfig): Unit = config.run {
        if (!profileInput.isConnected()) throw IllegalBlockConfigurationException("Motion profile input to ${this@MotionProfileFollower} must be connected.")
    }

    /** The motion profile input. See [MotionProfileFollower]*/
    val profileInput: BlocksConfig.Input<MotionProfiled<T>> get() = configInput(0)
    /** The stop input input. See [MotionProfileFollower] */
    val stop: BlocksConfig.Input<Boolean?> get() = configInput(1)
    /** The [BlocksConfig.Output] of this [MotionProfileFollower] */
    val output: BlocksConfig.Output<T> get() = configOutput(0)
    /** The progress [BlocksConfig.Output] of this [MotionProfileFollower] */
    val progress: BlocksConfig.Output<Double> get() = configOutput(1)
    /** True if this follower is following an output. */
    val isFollowing: BlocksConfig.Output<Boolean> get() = configOutput(2)
}

/**
 * A [MotionProfileFollower] that only uses time to progress along the motion profile.
 *
 * Inputs:
 * - [profileInput]: The [MotionProfiled] object to follow, or null to indicate to idling at the last reference given.
 *      WILL ONLY BE POLLED upon reaching end of the previous motion profile, or input #2 is pulsed:
 * - [stop] to stop following motion profile. When given `true`, will immediately cancel following the
 *      current motion profile and the next profile will be polled, if any. For example, using [Pulse].
 *
 * Outputs:
 * - [output] current output of the motion profile.
 * - [progress] The current progress along motion profile as a number from 0 to 1.
 * - [isFollowing] true if this follower is following a profile, false if idling.
 *
 * @param initialIdleOutput the initial value to be outputted when no motion profile has been ever given.
 */
class TimeOnlyMotionProfileFollower<T : Any>(initialIdleOutput: T) : MotionProfileFollower<T>(2, 3, initialIdleOutput) {

    override fun getNextTime(
        currentTime: Double, lastOutput: Any, inputs: List<Any?>, systemValues: SystemValues
    ): Double = currentTime + systemValues.loopTime

    override fun processFurther(inputs: List<Any?>) {
    }

    override fun getMoreOutput(index: Int) {
    }
}
