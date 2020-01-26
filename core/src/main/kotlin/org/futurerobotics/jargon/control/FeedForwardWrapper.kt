package org.futurerobotics.jargon.control

import org.futurerobotics.jargon.math.MotionOnly
import org.futurerobotics.jargon.math.MotionState

/**
 * A controller that wraps around another [velocityController] to add simple pass-through feed forward.
 * The inner controller is assumed to give a target _velocity_ output based on state.
 *
 * The input is now a [MotionState], and the output now a [MotionOnly].
 *
 * - The _value_ of the motion state is passed to the [velocityController]
 * - The _velocity/derivative_ of the [velocityController] is added to the signal
 * - The acceleration of the motion state is passed to the output.
 *
 * This adds simple feed-forward mechanics.
 *
 * A [plus] function still needs to be supplied in order to add values.
 *
 * @param plus a function to add two values.
 */
class FeedForwardWrapper<State>(
    private val velocityController: SimpleController<State, State, State>,
    private val plus: (State, State) -> State
) : SimpleController<MotionState<State>, State, MotionOnly<State>> {


    override var signal: MotionOnly<State> = MotionOnly(velocityController.signal, velocityController.signal)
        private set

    override fun reset() {
        velocityController.reset()
        val controllerSignal = velocityController.signal
        signal = MotionOnly(controllerSignal, controllerSignal)
    }

    private operator fun State.plus(state: State) = plus(this, state)
    override fun update(
        reference: MotionState<State>,
        currentState: State,
        elapsedTimeInNanos: Long
    ): MotionOnly<State> {
        val (ref, vel, accel) = reference
        val controllerSignal = velocityController.update(ref, currentState, elapsedTimeInNanos)
        val finalVel = controllerSignal + vel
        return MotionOnly(finalVel, accel)
    }
}
