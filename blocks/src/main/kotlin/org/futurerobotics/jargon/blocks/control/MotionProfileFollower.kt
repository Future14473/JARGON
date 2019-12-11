package org.futurerobotics.jargon.blocks.control

import org.futurerobotics.jargon.blocks.Block
import org.futurerobotics.jargon.blocks.Block.Processing.ALWAYS
import org.futurerobotics.jargon.profile.MotionProfiled
import org.futurerobotics.jargon.running.CompletionCallback
import org.futurerobotics.jargon.util.Stepper

/**
 * Base class for implementing a block that follows a motion profile.
 *
 * When a motion profile is done following, the follower will stop on the last output of the previous
 * profile.
 *
 * Inputs:
 * - [profileInput]: The [MotionProfiled] object to follow, or null to indicate to idling at the last reference given.
 *      WILL ONLY BE POLLED upon reaching end of the previous motion profile, or input #2 is pulsed:
 * - [profileWithCallback]: Profiles can also be given with a [CompletionCallback] which will be called
 *   when the profile is done being traversed. This will be prioritized over [profileInput]
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
    val profileInput: Input<MotionProfiled<T>?> = newOptionalInput()
    /** Motion profile input with callbacks. See [MotionProfileFollower] */
    val profileWithCallback: Input<Pair<MotionProfiled<T>, CompletionCallback>?> = newOptionalInput()
    /** The stop input. See [MotionProfileFollower] */
    val stop: Input<Boolean?> = newOptionalInput()
    /** The output from the motion profile this [MotionProfileFollower] is at. */
    val output: Output<T> = newOutput()
    /**
     * The progress along the current motion profile, as a number from 0 to 1. If not following any profile,
     * the progress is 1.0.
     */
    val progress: Output<Double> = newOutput()
    /** If this follower is currently following a motion profile. */
    val isFollowing: Output<Boolean> = newOutput()

    private var outputValue: T = initialOutput
    private var currentTime: Double = 0.0
    private var currentProfile: MotionProfiled<T>? = null
    private var currentCallback: CompletionCallback? = null
    private var currentStepper: Stepper<T>? = null //if null; means poll more.

    final override fun init() {
        outputValue = initialOutput
        currentTime = 0.0
        currentProfile = null
        currentCallback = null
        currentStepper = null
    }

    final override fun Context.process() {
        update()
        output.set = outputValue
        progress.set = currentProfile?.let { currentTime / it.duration } ?: 1.0
        isFollowing.set = currentStepper != null
        this.processFurther()
    }

    private fun Context.update() {
        var stepper = currentStepper
        val stop = stop.get == true
        if (stop || stepper === null) {//always poll stop
            currentCallback?.let {
                if (stop) it.cancel() else it.complete()
            }
            currentCallback = null
            val newProfile = profileWithCallback.get?.let {
                currentCallback = it.second
                it.first
            } ?: profileInput.get ?: return
            currentTime = 0.0
            stepper = newProfile.stepper()
            currentProfile = newProfile
            currentStepper = stepper
        } else {
            currentTime = getNextTime(currentTime, outputValue)
        }
        val endTime = currentProfile!!.duration
        if (currentTime >= endTime) {
            currentTime = endTime
            currentStepper = null
            currentProfile = null
        }
        outputValue = stepper.stepTo(currentTime)
    }

    /** Performs any additional possible processing. */
    protected open fun Context.processFurther() {}

    /**
     * Gets the next time to use to get the value out of the [MotionProfiled].
     *
     * @param currentTime along the motion profile.
     * @param lastOutput of the motion profile.
     */
    protected abstract fun Context.getNextTime(currentTime: Double, lastOutput: T): Double
}

/**
 * A [MotionProfileFollower] that only uses time to progress along the motion profile.
 *
 * @param initialIdleOutput the initial value to be outputted when no motion profile has been ever given.
 */
class TimeOnlyMotionProfileFollower<T : Any>(initialIdleOutput: T) : MotionProfileFollower<T>(initialIdleOutput) {

    override fun Context.getNextTime(currentTime: Double, lastOutput: T): Double = currentTime + loopTime
}
