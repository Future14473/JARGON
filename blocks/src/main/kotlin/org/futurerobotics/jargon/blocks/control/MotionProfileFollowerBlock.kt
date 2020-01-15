package org.futurerobotics.jargon.blocks.control

import org.futurerobotics.jargon.blocks.Block
import org.futurerobotics.jargon.blocks.Block.Processing.ALWAYS
import org.futurerobotics.jargon.control.MotionProfileFollower
import org.futurerobotics.jargon.control.TimeOnlyMotionProfileFollower
import org.futurerobotics.jargon.profile.TimeProfiled

/**
 * A base implementation of a block that wraps around a [MotionProfileFollower].
 *
 * An [update] function needs to be overriden for the specific follower's updating.
 */
abstract class MotionProfileFollowerBlock<T : Any, P : TimeProfiled<T>>(private val initialOutput: T) : Block(ALWAYS) {

    /** The motion profile input */
    val profileInput: Input<P?> = newInput()
    /** The stop input. If inputted true, will cancel following the current motion profile.*/
    val stop: Input<Boolean?> = newOptionalInput()
    /** The output from the motion profile this [MotionProfileFollowerBlock] is at. */
    val output: Output<T> = newOutput()

    /** If this follower is currently following a motion profile. */
    val isFollowing: Output<Boolean> = newOutput()

    /**
     * The motion profile follower used.
     */
    protected abstract val follower: MotionProfileFollower<T, P>

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
    }

    /** Updates the [follower]. */
    protected abstract fun Context.update()
}

/**
 * A [MotionProfileFollowerBlock] that uses a [TimeOnlyMotionProfileFollower].
 */
class TimeOnlyMotionProfileFollowerBlock<T : Any, P : TimeProfiled<T>>(initialIdleOutput: T) :
    MotionProfileFollowerBlock<T, P>(initialIdleOutput) {

    override val follower: TimeOnlyMotionProfileFollower<T, P> = TimeOnlyMotionProfileFollower()

    override fun Context.update() {
        follower.update(elapsedTimeInNanos)
    }
}
