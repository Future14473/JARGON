package org.futurerobotics.jargon.control

import org.futurerobotics.jargon.math.MotionOnly
import org.futurerobotics.jargon.math.MotionState

//TODO: better doc
//TODO: test
/**
 * A [SimpleController] that wraps around another [controller][velocityController] to add simple feed forward.
 * This operates on some value of [State] (a value, vector, pose, etc)
 *
 * The velocity controller takes a reference position and a measured position, and produces a desired velocity to
 * reach that state. This wrapper takes in an entire [MotionState] as a reference (position, velocity, and
 * acceleration), and produces a [MotionOnly] as a signal. The reference position is passed to the inner
 * [velocityController], the reference velocity is added to the controller's output, and the acceleration is
 * simply passed to the output.
 *
 * A [plus] function needs to be supplied to add two values.
 *
 * @param velocityController The inner controller this wraps around. This should take in some [State] as a position
 *                           and reference, and outputs a target _velocity_ output based on that state.
 * @param plus a function to add two values.
 */
open class FeedForwardWrapper<State>(
    private val velocityController: SimpleController<State, State, State>,
    private val plus: State.(State) -> State
) : SimpleController<MotionState<State>, State, MotionOnly<State>> {


    final override var signal: MotionOnly<State> = MotionOnly(velocityController.signal, velocityController.signal)
        private set

    override fun update(
        reference: MotionState<State>,
        currentState: State,
        elapsedTimeInNanos: Long
    ): MotionOnly<State> {
        val (ref, vel, accel) = reference
        val controllerVel = velocityController.update(ref, currentState, elapsedTimeInNanos)
        val finalVel = controllerVel.plus(vel)
        return MotionOnly(finalVel, accel)
    }
}
