package org.futurerobotics.jargon.control

import org.futurerobotics.jargon.profile.TimeProfiled
import org.futurerobotics.jargon.util.Stepper
import org.futurerobotics.jargon.util.replaceIf
import java.util.concurrent.ConcurrentLinkedQueue

/**
 * Base class for implementing a construct that follows a motion profile.
 *
 * One needs to subclass and define their own update method/mechanism. This is done
 * by calling [needUpdate], and if it returns true, later calling [moveToTime].
 *
 * Only [output] and [queueProfile] can be called from different threads without issues.
 * This may change in the future.
 *
 * for example:
 *
 * kotlin:
 * ```kotlin
 * fun update(elapsedNanos: Long){
 *    if(needUpdate()){
 *       moveToTime(currentTime + elapsedNanos/1e9)
 *    }
 * }
 * ```
 * java:
 * ```java
 * void update(long elapsedNanos){
 *    if(needUpdate()){
 *       moveToTime(currentTime + elapsedNanos/1e9);
 *    }
 * }
 * ```
 * @see TimeOnlyMotionProfileFollower
 */
abstract class MotionProfileFollower<T : Any, P : TimeProfiled<T>> {

    @Volatile
    private var _output: T? = null
    /**
     * Gets the current time traversed along the profile.
     */
    protected var currentTime = 0.0
        private set

    private val profileQueue = ConcurrentLinkedQueue<P>()
    private var curStepper: Stepper<T>? = null

    /** The currently traversed profile. */
    protected var currentProfile: P? = null
        private set

    /**
     * The current output of this [MotionProfileFollower].
     *
     * May throw exception on get if this has never been [reset].
     */
    val output: T get() = _output ?: error("Output not initialized.")

    /** If this motion profile follower is currently following any profile. */
    val isFollowing: Boolean = currentProfile != null

    /**
     * Queues a profile to be followed.
     */
    fun queueProfile(profile: P) {
        profileQueue += profile
    }

    /**
     * Resets this [MotionProfileFollower]. An [initialOutput] must be given
     * for when this is run but still without any motion profile output.
     */
    fun reset(initialOutput: T) {
        _output = initialOutput
        curStepper = null
        currentProfile = null
        profileQueue.clear()
    }

    /**
     * Queues if an update via [moveToTime] is needed (checks the queue).
     * If this returns true, one should calculate the next time in [moveToTime].
     * Otherwise, not.
     *
     * Note that when a profile is first selected, it automatically moves to time 0.0
     */
    protected fun needUpdate(): Boolean {
        val stepper = curStepper
        if (stepper === null) {
            val newProfile = profileQueue.poll() ?: return false
            currentTime = 0.0
            currentProfile = newProfile
            curStepper = newProfile.stepper()
            moveToTime(0.0)
            return false
        }
        return true
    }

    /**
     * Only if [needUpdate] previously returned true; moves to the given [time] along
     * the motion profile and returns the profile's output.
     *
     * A time >= to the duration of the [currentProfile] indicates this profile is done
     * being traversed.
     */
    protected fun moveToTime(time: Double): T {

        val stepper = curStepper ?: error("Not traversing any profile!")
        val endTime = currentProfile!!.duration
        val realTime = time.replaceIf({ it >= endTime }) {
            currentProfile = null
            endTime
        }
        currentTime = realTime
        return stepper.stepTo(realTime).also { _output = it }
    }
}

/**
 * A [MotionProfileFollower] that only traversed based on time.
 */
class TimeOnlyMotionProfileFollower<T : Any, P : TimeProfiled<T>> : MotionProfileFollower<T, P>() {

    /**
     * Updates traversing along the profile, given the time in [elapsedNanos] since the
     * last update.
     */
    fun update(elapsedNanos: Long) {
        if (needUpdate()) {
            moveToTime(currentTime + elapsedNanos / 1e9)
        }
    }
}
