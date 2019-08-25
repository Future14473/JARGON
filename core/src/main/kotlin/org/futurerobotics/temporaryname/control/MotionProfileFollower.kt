package org.futurerobotics.temporaryname.control

import org.futurerobotics.temporaryname.control.reference.DerivRefProvider
import org.futurerobotics.temporaryname.control.reference.GenericReferenceProvider
import org.futurerobotics.temporaryname.math.MathUnits.nanoseconds
import org.futurerobotics.temporaryname.math.MathUnits.seconds
import org.futurerobotics.temporaryname.motionprofile.MotionProfiled
import org.futurerobotics.temporaryname.util.Steppable
import org.futurerobotics.temporaryname.util.Stepper
import org.futurerobotics.temporaryname.util.value
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicReference
import java.util.concurrent.locks.ReentrantLock
import kotlin.concurrent.withLock

//command states
private object IDLE

private inline val NONE get() = null
//run states
/**
 * A base implementation of a [GenericReferenceProvider] that follows a [MotionProfiled] object.
 *
 * The abstract function [getNextTime] dictates what time is fed into the [MotionProfiled] object.
 *
 * The base implementation is also thread safe.
 *
 * @param initialState is the initial reference this outputs before following any [MotionProfiled]
 * @param zeroDeriv the value for the reference deriv when not following any [MotionProfiled]
 */
abstract class MotionProfileFollower<ProfileState, State : Any, StateDeriv : Any>(
    initialState: State, private val zeroDeriv: StateDeriv? = null
) : DerivRefProvider<State, StateDeriv> {
    //  null -> nothing
    //  REQUEST_IDLE -> stop
    //  else -> new Profiled
    private val command = AtomicReference<Any?>()
    //if is holding reference; someday possibly update to state instead of only checking currentStepper.
    private val holdingReference = AtomicBoolean(false)

    private val updateLock = ReentrantLock()
    // ^ guards the following:
    private var currentStepper: Stepper<Double, ProfileState>? = null
    private var currentDuration: Double = 0.0
    private var currentTime: Double = Double.NaN
    private var pastOutput: Pair<State, StateDeriv?> = initialState to zeroDeriv
    private var pastState: State? = null

    final override fun update(currentState: State, elapsedNanos: Long): Pair<State, StateDeriv?> = updateLock.withLock {
        processCommand()

        val currentStepper = currentStepper
            ?: return pastOutput.also { pastState = currentState }
        //holdReference == false

        if (currentTime.isNaN()) { //just started
            currentTime = 0.0
        } else if (elapsedNanos != -1L) {
            currentTime = getNextTime(
                pastOutput,
                pastState!!,
                currentState,
                currentTime,
                elapsedNanos
            )
        }
        pastState = currentState
        val output = if (currentTime >= currentDuration) {
            holdingReference.value = true //vwrite
            this.currentStepper = null
            mapState(currentStepper.stepTo(currentDuration)).first to zeroDeriv
        } else {
            mapState(currentStepper.stepTo(currentTime))
        }
        return output.also { pastOutput = it }
    }

    private fun processCommand() {
        when (val command = command.value) {
            NONE -> {}
            IDLE -> {
                this.command.compareAndSet(command, NONE) //otherwise, pickup on next loop.
                if (holdingReference.compareAndSet(false, true)) {
                    currentStepper = null
                    pastOutput = pastOutput.first to zeroDeriv
                }
            }
            else -> { //follow profiled.
                this.command.compareAndSet(command, NONE)
                holdingReference.value = false
                @Suppress("UNCHECKED_CAST")
                command as MotionProfiled<ProfileState>
                currentDuration = command.duration
                currentStepper = command.stepper()
                currentTime = Double.NaN
            }
        }
    }

    /**
     * Sets this reference provider to follow the current [profiled] object.
     */
    fun follow(profiled: MotionProfiled<ProfileState>) {
        beforeFollow()
        command.set(profiled)
    }

    /**
     * Stops following any [Steppable] and instead holds the reference at the last outputted state.
     */
    fun stopFollowing() {
        command.set(IDLE)
    }

    /**
     * If this reference is currently still following a [MotionProfiled]
     */
    fun isFollowing(): Boolean {
        return !holdingReference.value
    }

    override fun stop(): Unit = updateLock.withLock {
        currentStepper = null
    }

    override fun start() {
        currentStepper = null
    }
    /**
     * What is run before [follow] is called; do any reset-before-following-profile here.
     */
    protected open fun beforeFollow(){}
    /**
     * Gets the "virtual" next time to go to on a [MotionProfiled] object, based on the supplied
     * [pastOutput], [pastState], [currentState], [pastVirtualTime],
     * and the real elapsed time in nanoseconds [elapsedNanos].
     *
     * This time will be used to progress through [MotionProfiled] object.
     * A returned time of Double.NAN or greater than the current [MotionProfiled]'s profile duration
     * signals for this to either halt or hold the reference at the final state.
     *
     * For example, if motors were under-actuated, this can account for it by not stepping time up as fast.
     */
    protected abstract fun getNextTime(
        pastOutput: Pair<State, StateDeriv?>,
        pastState: State,
        currentState: State,
        pastVirtualTime: Double,
        elapsedNanos: Long
    ): Double

    /**
     * Maps the profile's State ([profileState]) to a reference state.
     * @see update
     */
    protected abstract fun mapState(profileState: ProfileState): Pair<State, StateDeriv?>
}

/**
 * A simple implementation of [MotionProfileFollower] that only uses a Clock/timer.
 *
 * Still needs [mapState] implementation.
 */
abstract class TimeOnlyProfileFollower<ProfileState, State : Any, StateDeriv : Any>(
    initialState: State,
    zeroDeriv: StateDeriv?
) : MotionProfileFollower<ProfileState, State, StateDeriv>(initialState, zeroDeriv) {

    final override fun getNextTime(
        pastOutput: Pair<State, StateDeriv?>,
        pastState: State,
        currentState: State,
        pastVirtualTime: Double,
        elapsedNanos: Long
    ): Double = pastVirtualTime + elapsedNanos * seconds / nanoseconds
}