package org.futurerobotics.temporaryname.control.controller

/**
 * Represents a controller.
 * It takes in measured states and a reference state and produces [Signal] to give
 * to a Plant to drive the current State to the reference.
 *
 * This does not _have to_ include linear algebra and fancy math.
 *
 * Nothing is stopping you from "chaining" controllers together to create a "control pipeline", so to speak.
 *
 * @param State the State representation
 * @param StateDeriv the [State]'s Derivative representation
 * @param Signal the output signal.
 */
interface Controller<in State, in StateDeriv : Any, out Signal> {

    /**
     * Given the [current] state, the [reference] state, if the expected [referenceDeriv] (or null if unknown/not
     * implemented), and the elapsedTime in nanoseconds [elapsedNanos],
     * produces a control [Signal] to drive the current state to the [reference].
     *
     * [elapsedNanos] of -1 indicates the first iteration of the control loop.
     *
     * A [referenceDeriv] of null indicates unknown referenceChange.
     */
    fun process(current: State, reference: State, referenceDeriv: StateDeriv? = null, elapsedNanos: Long): Signal

    /**
     * Performs any necessary reset actions to this controller, if applicable
     */
    fun reset()
}

/**
 * A base implementation of a [Controller] that ignores referenceDeriv.
 *
 * This does not _have to_ include linear algebra and fancy math.
 *
 * Nothing is stopping you from "chaining" controllers together to create a "control pipeline", so to speak.
 */
abstract class ControllerIgnoreDeriv<in State, out Signal> :
    Controller<State, Any, Signal> {
    /**
     * Given the [current] state, the [reference] state, and the elapsedTime in nanoseconds [elapsedNanos],
     * produces a control [Signal] for a Plant to drive the current state to the [reference].
     *
     * [elapsedNanos] of -1 indicates the first iteration of the control loop.
     */
    protected abstract fun process(current: State, reference: State, elapsedNanos: Long): Signal

    final override fun process(current: State, reference: State, referenceDeriv: Any?, elapsedNanos: Long): Signal {
        return process(current, reference, elapsedNanos)
    }
}

/**
 * A controller that does not take into account the current state; i.e. a open loop control system.
 */
abstract class OpenController<in State, in StateDeriv : Any, out Signal> : Controller<State, StateDeriv, Signal> {
    final override fun process(
        current: State,
        reference: State,
        referenceDeriv: StateDeriv?,
        elapsedNanos: Long
    ): Signal = process(reference, referenceDeriv, elapsedNanos)

    /**
     * Given the [reference] state, if the expected [referenceDeriv] (or null if unknown/not
     * implemented), and the elapsedTime in nanoseconds [elapsedNanos],
     * produces a control [Signal] to drive the current state to the [reference].
     *
     * [elapsedNanos] of -1 indicates the first iteration of the control loop.
     *
     * A [referenceDeriv] of null indicates unknown referenceChange.
     */
    protected abstract fun process(reference: State, referenceDeriv: StateDeriv?, elapsedNanos: Long): Signal
}

/**
 * A controller that simply feeds the referenceDeriv as it's output. Used for chaining open controllers.
 */
class OpenFeedDerivController<StateDeriv : Any>(private val defaultDeriv: StateDeriv) :
    Controller<Any, StateDeriv, StateDeriv> {
    override fun process(current: Any, reference: Any, referenceDeriv: StateDeriv?, elapsedNanos: Long): StateDeriv =
        referenceDeriv ?: defaultDeriv

    override fun reset() {}
}