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
 * Base class for implementing a component that follows a motion profiled object.
 *
 * Inputs:
 * 1. The [MotionProfiled] object to follow, or null to indicate to idling at the last reference given.
 *      WILL ONLY BE POLLED upon reaching end of the previous motion profile, or input #2 is pulsed:
 * 2. Boolean to stop following motion profile. When given `true`, will immediately cancel following the
 *      current motion profile and the next profile will be polled. Recommended using [Pulse] to accomplish
 * Subclasses may specify other inputs, starting with #3.
 *
 * Outputs:
 * 1. The current output of the motion profiled object.
 * 2. The current progress along motion profiled object as a number from 0 to 1.
 * Subclasses may specify other outputs, starting with #3
 *
 * @param initialIdleOutput the initial output if the system is idle and no motion profiled has been given
 *              yet.
 */
abstract class MotionProfileFollower<T : Any>(numInputs: Int, numOutputs: Int, private val initialIdleOutput: T) :
    AbstractBlock(numInputs, numOutputs, IN_FIRST_ALWAYS) {
    private var profileOutput: T = initialIdleOutput

    private var currentTime: Double = 0.0
    private var endTime: Double = 1.0
    private var currentStepper: Stepper<Double, T>? = null //if null; means poll more.

    init {
        require(numInputs >= 3) { "NumInputs should be >= 2" }
        require(numOutputs >= 2) { "NumInputs should be >= 2" }
    }

    final override fun init() {
        profileOutput = initialIdleOutput
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
            currentTime = getNextTime(currentTime, profileOutput, inputs, systemValues)
        }
        if (currentTime >= endTime) {
            currentTime = endTime
            this.currentStepper = null
        }
        profileOutput = currentStepper.stepTo(currentTime)

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
        currentTime: Double,
        lastOutput: Any,
        inputs: List<Any?>,
        systemValues: SystemValues
    ): Double

    override fun getOutput(index: Int): Any? = when (index) {
        !in 0..numOutputs -> IndexOutOfBoundsException(index)
        0 -> profileOutput
        1 -> currentTime / endTime
        else -> getMoreOutput(index)
    }

    /** Gets any additional possible outputs, starting with index 2. */
    protected abstract fun getMoreOutput(index: Int)

    override fun prepareAndVerify(config: BlocksConfig) = config.run {
        if (!profileInput.isConnected())
            throw IllegalBlockConfigurationException("Motion profile input to ${this@MotionProfileFollower} must be connected.")
    }

    /** The motion profile [BlocksConfig.Input]. See [MotionProfileFollower]*/
    val profileInput: BlocksConfig.Input<MotionProfiled<T>> get() = configInput(0)

    /** The stop input [BlocksConfig.Input]. See [MotionProfileFollower] */
    val stop: BlocksConfig.Input<Boolean?> get() = configInput(1)

    /** The [BlocksConfig.Output] of this [MotionProfileFollower] */
    val output: BlocksConfig.Output<T> get() = configOutput(0)

    /** The progress [BlocksConfig.Output] of this [MotionProfileFollower] */
    val progress: BlocksConfig.Output<Double> get() = configOutput(1)


}

/**
 * A [MotionProfileFollower] that only uses time to progress along the motion profile.
 *
 * Inputs:
 * 1. The [MotionProfiled] object to follow, or null to indicate to idling at the last reference given.
 *      WILL ONLY BE POLLED upon reaching end of the previous motion profile, or input #2 is pulsed:
 * 2. Boolean to stop following motion profile. When given `true`, will immediately cancel following the
 *      current motion profile and the next profile will be polled. Recommended using [Pulse] to accomplish
 *
 * Outputs:
 * 1. The current output of the motion profiled object.
 * 2. The current progress along motion profiled object as a number from 0 to 1.
 *
 * @param initialIdleOutput the initial value to be outputted when no motion profile has been ever given.
 */
class TimeOnlyMotionProfileFollower<T : Any>(initialIdleOutput: T) :
    MotionProfileFollower<T>(2, 2, initialIdleOutput) {

    override fun getNextTime(
        currentTime: Double,
        lastOutput: Any,
        inputs: List<Any?>,
        systemValues: SystemValues
    ): Double = currentTime + systemValues.loopTime

    override fun processFurther(inputs: List<Any?>) {
    }

    override fun getMoreOutput(index: Int) {
    }
}