package org.futurerobotics.jargon.blocks.control

import org.futurerobotics.jargon.blocks.Block
import org.futurerobotics.jargon.blocks.Block.Processing.ALWAYS
import org.futurerobotics.jargon.control.TimeOnlyMotionProfileFollower
import org.futurerobotics.jargon.profile.TimeProfiled

/**
 * Base class for implementing a block that follows a motion profile.
 *
 * When a motion profile is done following, the follower will stop on the last output of the previous
 * profile.
 *
 * Inputs:
 * - [profileInput]: The [TimeProfiled] object to follow, or null to indicate to idling at the last reference given.
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
abstract class MotionProfileFollower<T : Any, P : TimeProfiled<T>>(private val initialOutput: T) : Block(ALWAYS) {

    /** The motion profile input. See [MotionProfileFollower]*/
    val profileInput: Input<P?> = newInput()
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

    protected abstract val follower: org.futurerobotics.jargon.control.MotionProfileFollower<T, P>

    final override fun init() {
        follower.reset(initialOutput)
    }

    final override fun Context.process() {
        if (stop.get == true) {
            follower.reset(follower.outputOrNull ?: initialOutput)
        } else {
            update()
        }
        output.set = follower.output
        val following = follower.isFollowing
        isFollowing.set = following
        progress.set = if (following) follower.currentTime / follower.currentProfile!!.duration else 1.0
    }

    /** Updates the [follower]. */
    protected abstract fun Context.update()
}

/**
 * A [MotionProfileFollower] that only uses time to progress along the motion profile.
 *
 * @param initialIdleOutput the initial value to be outputted when no motion profile has been ever given.
 */
class TimeOnlyMotionProfileFollower<T : Any, P : TimeProfiled<T>>(initialIdleOutput: T) :
    MotionProfileFollower<T, P>(initialIdleOutput) {

    override val follower: org.futurerobotics.jargon.control.MotionProfileFollower<T, P> =
        TimeOnlyMotionProfileFollower()

    override fun Context.update() {
        (follower as TimeOnlyMotionProfileFollower<*, *>).update(loopTimeInNanos)
    }
}
