package org.futurerobotics.jargon.blocks.control

import org.futurerobotics.jargon.blocks.Block
import org.futurerobotics.jargon.blocks.Block.Processing.ALWAYS
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
abstract class MotionProfileFollower<T : Any>(private val initialOutput: T) : Block(ALWAYS) {

    /** The motion profile input. See [MotionProfileFollower]*/
    val profileInput: Input<MotionProfiled<T>?> = newInput()
    /** The stop input. See [MotionProfileFollower] */
    val stop: Input<Boolean?> = newOptionalInput(isOptional = true)
    /** The [Block.Output] of this [MotionProfileFollower] */
    val output: Output<T> = newOutput()
    /** The progress [Block.Output] of this [MotionProfileFollower] */
    val progress: Output<Double> = newOutput()
    /** True if this follower is following an output. */
    val isFollowing: Output<Boolean> = newOutput()

    private var outputValue: T = initialOutput
    private var currentTime: Double = 0.0
    private var endTime: Double = 1.0
    private var currentStepper: Stepper<Double, T>? = null //if null; means poll more.

    final override fun init() {
        outputValue = initialOutput
        currentTime = 0.0
        endTime = 1.0
        currentStepper = null
    }

    final override fun Context.process() {
        update()
        output.set = outputValue
        progress.set = currentTime / endTime
        isFollowing.set = currentStepper != null
        processFurther(this)
    }

    private fun Context.update() {
        var currentStepper = currentStepper
        if (stop.get == true || currentStepper == null) {//always poll stop
            val newProfiled = profileInput.get ?: return
            currentTime = 0.0
            endTime = newProfiled.duration
            currentStepper = newProfiled.stepper()
            this@MotionProfileFollower.currentStepper = currentStepper
        } else {
            currentTime = getNextTime(
                currentTime,
                outputValue
            )
        }
        if (currentTime >= endTime) {
            currentTime = endTime
            this@MotionProfileFollower.currentStepper = null
        }
        outputValue = currentStepper.stepTo(currentTime)
    }

    /** Performs any additional possible processing. */
    protected open fun processFurther(inputs: Context) {}

    /**
     * Gets the next time to use to get the value out of the [MotionProfiled] object;
     *
     * given the [currentTime] along the profile, the [lastOutput]ed value, and possible additional
     * [this@getNextTime]. If this component specifies it (Please do not to access inputs (0-1) or things may break)
     *
     * Following the motion profile ends if [getNextTime] returns a time longer past the current motion profiled's
     * duration.
     */
    protected abstract fun Context.getNextTime(currentTime: Double, lastOutput: Any): Double
}

/**
 * A [MotionProfileFollower] that only uses time to progress along the motion profile.
 *
 * @param initialIdleOutput the initial value to be outputted when no motion profile has been ever given.
 */
class TimeOnlyMotionProfileFollower<T : Any>(initialIdleOutput: T) : MotionProfileFollower<T>(initialIdleOutput) {

    override fun Context.getNextTime(currentTime: Double, lastOutput: Any): Double = currentTime + loopTime
}
