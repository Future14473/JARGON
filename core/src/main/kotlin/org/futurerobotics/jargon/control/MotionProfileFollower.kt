@file:Suppress("UNCHECKED_CAST")

package org.futurerobotics.jargon.control

import org.futurerobotics.jargon.control.Block.InOutOrder.IN_FIRST
import org.futurerobotics.jargon.control.Block.Processing.ALWAYS
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
abstract class MotionProfileFollower(numInputs: Int, numOutputs: Int, private val initialIdleOutput: Any) :
    AbstractBlock(numInputs, numOutputs, IN_FIRST, ALWAYS) {
    private var profileOutput: Any = initialIdleOutput

    private var currentTime: Double = 0.0
    private var endTime: Double = 1.0
    private var currentStepper: Stepper<Double, Any>? = null //if null; means poll more.

    init {
        require(numInputs >= 3) { "NumInputs should be >= 2" }
        require(numOutputs >= 2) { "NumInputs should be >= 2" }
    }

    override fun init(outputs: MutableList<Any?>) {
        profileOutput = initialIdleOutput
        currentTime = 0.0
        endTime = 1.0
        currentStepper = null
    }

    final override fun process(inputs: List<Any?>, outputs: MutableList<Any?>) {
        var currentStepper = currentStepper
        if (inputs[1] as Boolean? == true || currentStepper == null) {//always poll inputs[2]; so doesn't store up
            val newProfiledMaybe = inputs[0] ?: return
            val newProfiled = newProfiledMaybe as MotionProfiled<*>
            currentTime = 0.0
            endTime = newProfiled.duration
            currentStepper = newProfiled.stepper()
            this.currentStepper = currentStepper
        } else {
            currentTime = getNextTime(currentTime, profileOutput, inputs)
        }
        if (currentTime >= endTime) {
            currentTime = endTime
            this.currentStepper = null
        }
        profileOutput = currentStepper.stepTo(currentTime)

        outputs[0] = profileOutput
        outputs[1] = currentTime / endTime
        processFurther(inputs, outputs)
    }

    /**
     * Gets the next time to use to get the value out of the [MotionProfiled] object;
     *
     * given the [currentTime] along the profile, the [lastOutput]ed value, and possible additional
     * [inputs]. If this component specifies it (Please do not to access inputs (0-1) or things may break)
     *
     * Following the motion profile ends if [getNextTime] returns a time longer past the current motion profiled's
     * duration.
     */
    protected abstract fun getNextTime(currentTime: Double, lastOutput: Any, inputs: List<Any?>): Double

    /**
     * Performs any additional possible processing, possibly using more outputs.
     */
    abstract fun processFurther(inputs: List<Any?>, outputs: MutableList<Any?>)
}

/**
 * A [MotionProfileFollower] that only uses time to progress along the motion profile.
 *
 * Inputs:
 * 1. The [MotionProfiled] object to follow, or null to indicate to idling at the last reference given.
 *      WILL ONLY BE POLLED upon reaching end of the previous motion profile, or input #2 is pulsed:
 * 2. Boolean to stop following motion profile. When given `true`, will immediately cancel following the
 *      current motion profile and the next profile will be polled. Recommended using [Pulse] to accomplish
 * 3. The loop time.
 *
 * Outputs:
 * 1. The current output of the motion profiled object.
 * 2. The current progress along motion profiled object as a number from 0 to 1.
 *
 * @param initialIdleOutput the initial value to be outputted when no motion profile has been ever given.
 */
class TimeOnlyMotionProfileFollower(initialIdleOutput: Any) : MotionProfileFollower(3, 2, initialIdleOutput) {

    override fun getNextTime(currentTime: Double, lastOutput: Any, inputs: List<Any?>): Double {
        return currentTime + inputs[2] as Double
    }

    override fun processFurther(inputs: List<Any?>, outputs: MutableList<Any?>) {
    }
}