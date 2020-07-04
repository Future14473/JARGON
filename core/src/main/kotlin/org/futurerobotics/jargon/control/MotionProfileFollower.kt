package org.futurerobotics.jargon.control

import org.futurerobotics.jargon.profile.TimeProfiled
import org.futurerobotics.jargon.util.Stepper
import java.util.concurrent.ConcurrentLinkedQueue

/**
 * Base class for implementing a mechanism that follows a motion profile ([TimeProfiled]).
 *
 *
 * The default behavior when a profile has finished being followed is to keep outputting the very last output
 * of the motion profile.
 *
 *
 * You can add motion profiles to be followed using [queueProfile]. When subclassed and used appropriately,
 * one can then get appropriate [output]s from it.
 *
 * To use, one needs to subclass and define a update method.
 * In side the update function, one should first call [needUpdate], and if it returns true, then calling
 * [moveToTime] to move to an appropriate time ([needUpdate] handles motion profile queueing and completion).
 *
 * This provides [output], [currentProfile], [isFollowing], and [currentTime] as data that can be polled.
 *
 * If you are dealing with concurrency, only [output] and [queueProfile] are safe to be called from different threads
 * than the updating thread.
 *
 * Example implementation for a follower that simply tracks time ([TimeOnlyMotionProfileFollower]):
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
 * public void update(long elapsedNanos){
 *    if(needUpdate()){
 *       moveToTime(currentTime + elapsedNanos/1e9);
 *    }
 * }
 * ```
 * @see TimeOnlyMotionProfileFollower
 */
abstract class MotionProfileFollower<T>
@JvmOverloads constructor(initialOutput: T? = null) {

    private val profileQueue = ConcurrentLinkedQueue<TimeProfiled<T>>()

    /** The currently traversed profile, or null if not currently traversing a profile. */
    var currentProfile: TimeProfiled<T>? = null
        private set

    /**
     * If this motion profile follower is currently following any profile. Will be
     * false if queue is empty current profile is done.
     */
    val isFollowing: Boolean = currentProfile != null

    private var currentStepper: Stepper<T>? = null

    /**
     * The current time on traversing a [TimeProfiled].
     *
     * If not currently following a profile, the meaning of this value is undefined.
     */
    var currentTime: Double = 0.0
        private set

    /**
     * The current output of this [MotionProfileFollower], or null if no output exists.
     *
     * This can also be manually set by subclasses.
     */
    @Volatile
    var output: T? = initialOutput
        protected set

    /**
     * Queues up a profile to be followed.
     */
    fun queueProfile(profile: TimeProfiled<T>) {
        profileQueue += profile
    }

    /**
     * Resets this [MotionProfileFollower]. An [initialOutput] must be given for what output
     * to give out when no profiles are being followed.
     */
    fun reset(initialOutput: T) {
        output = initialOutput
        currentStepper = null
        currentProfile = null
        profileQueue.clear()
    }

    /**
     * Queues if an update via [moveToTime] is needed (checks the queue).
     * If this returns true, [currentProfile] will not be null and
     * one should calculate the next time needed and call [moveToTime], otherwise don't.
     *
     * Note that when a profile is first selected, it automatically moves to time 0.0.
     */
    protected fun needUpdate(): Boolean {
        val stepper = currentStepper
        if (stepper == null) {
            val newProfile = profileQueue.poll() ?: return false
            currentTime = 0.0
            currentProfile = newProfile
            currentStepper = newProfile.timeStepper()
            moveToTime(0.0)
            return false
        }
        return true
    }

    /**
     * Only if [needUpdate] previously returned true; moves to the given [time] along
     * the motion profile and returns the profile's output.
     *
     * Inputting time greater than or equal to to the duration of the [currentProfile] indicates this profile is done
     * being traversed.
     */
    protected fun moveToTime(time: Double): T {
        val stepper = currentStepper ?: error("Not traversing any profile!")
        val endTime = currentProfile!!.duration
        val realTime = if (time >= endTime) {
            currentProfile = null
            currentStepper = null
            endTime
        } else time
        currentTime = realTime
        return stepper.stepTo(realTime).also { output = it }
    }
}

/**
 * A [MotionProfileFollower] that works only on traversing based off of time.
 */
class TimeOnlyMotionProfileFollower<T> : MotionProfileFollower<T>() {

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
