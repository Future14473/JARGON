package org.futurerobotics.temporaryname.control

import org.futurerobotics.temporaryname.profile.MotionProfiled
import org.futurerobotics.temporaryname.util.Stepper
import org.futurerobotics.temporaryname.util.replaceIf
import org.futurerobotics.temporaryname.util.value
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicReference

/**
 * A base implementation of a [ReferenceTracker] that follows a [MotionProfiled] object.
 *
 * The abstract function [getNextTime] dictates what time is polled on the [MotionProfiled] object.
 *
 * The base implementation is also thread safe; if the thread that runs the loop is separate than the thread
 * that supplies [MotionProfiled]s
 */
abstract class MotionProfileFollower<State : Any, Reference : Any> :
    ReferenceTracker<State, Reference> {

    private var _reference: Reference? = null
    final override val reference: Reference
        get() = _reference ?: throw IllegalStateException("reference was accessed before function update was called.")

    /** Set to not null when command to follow a newProfiled reference. */
    private val newProfiled = AtomicReference<MotionProfiled<Reference>?>()
    /** If is currently following a profiled. */
    private val following = AtomicBoolean(false)
    //command only thread things:
    private var currentStepper: Stepper<Double, Reference>? = null
    private var currentDuration = 0.0
    private var currentTime = Double.NaN
    /** If idle reference needs to be polled. */
    private var needIdleRef = true
    /**
     * The past state given on the last call to [update]
     */
    protected var pastState: State? = null
        private set

    final override fun update(currentState: State, elapsedSeconds: Double) {
        _reference = run getRef@{
            processCommand()
            val pastState = pastState
            this.pastState = currentState
            val currentStepper = currentStepper
                ?: return@getRef if (needIdleRef || _reference == null) {
                    needIdleRef = false
                    getIdleReference(_reference, currentState)
                } else _reference

            if (currentTime.isNaN()) { //just started
                currentTime = 0.0
            } else if (!elapsedSeconds.isNaN() && pastState != null) {
                currentTime = getNextTime(
                    pastState,
                    currentState,
                    _reference
                        ?: throw AssertionError("If just started, currentTime should be NaN at least once."),
                    currentTime,
                    elapsedSeconds
                ).replaceIf({ it >= currentDuration || it.isNaN() }) {
                    following.value = false
                    this.currentStepper = null
                    needIdleRef = true
                    currentDuration
                }
            } //else keep as is
            return@getRef currentStepper.stepTo(currentTime)
        }
    }

    private fun processCommand() {
        val command = newProfiled.value
        if (command != null) {
            newProfiled.compareAndSet(command, null)
            following.value = true
            currentDuration = command.duration
            currentTime = Double.NaN
            currentStepper = command.stepper()
        }
    }

    /**
     * Sets this reference provider to follow the current [profiled] object.
     */
    fun follow(profiled: MotionProfiled<Reference>) {
        beforeFollow()
        newProfiled.set(profiled)
    }

    /**
     * If this reference is currently still following a [MotionProfiled]
     */
    fun isFollowing(): Boolean {
        return following.value
    }

    override fun stop() {
        currentStepper = null
        needIdleRef = true
    }

    override fun start() {
        stop()
        pastState = null
        _reference = null
    }

    /**
     * What is run before [follow] is called; do any reset-before-following-profile here.
     */
    protected open fun beforeFollow() {}

    /**
     * Gets the "virtual" next time to go to on a [MotionProfiled] object, based on the supplied
     * [pastOutput], [pastState], [currentState], [pastVirtualTime], and the real [elapsedSeconds].
     *
     * This time will be used to progress through [MotionProfiled] object.
     * A returned time of Double.NAN or greater than the current [MotionProfiled]'s profile duration
     * signals for this to either halt or hold the reference at the final state.
     *
     * For example, if the bot does not move as fast as expected, this can account for it by not stepping time up as
     * fast.
     */
    protected abstract fun getNextTime(
        pastState: State,
        currentState: State,
        pastOutput: Reference,
        pastVirtualTime: Double,
        elapsedSeconds: Double
    ): Double

    /**
     * Gets a reference to output when no motion profile is being followed.
     *
     * If [pastReference] is null, [update] had been called on the very first iteration of the control loop, and
     * [currentState] will be the very first estimate of the state.
     *
     * Otherwise, [pastReference] is the last reference obtained at the end of the last motion profiled followed,
     * and currentState is the measured state right after.
     */
    protected abstract fun getIdleReference(
        pastReference: Reference?,
        currentState: State
    ): Reference
}