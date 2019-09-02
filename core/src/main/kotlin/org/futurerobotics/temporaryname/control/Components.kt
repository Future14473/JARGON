package org.futurerobotics.temporaryname.control

import org.futurerobotics.temporaryname.system.StartStoppable

/**
 * A [ReferenceTracker] is the "root" of a ControlSystem.
 *
 * It provides the root [Reference] (desired state, fed into a [Controller]),
 * and is inputted [State]s (from a [Observer]) to update the reference, if the reference is dynamic.
 *
 * This is also [StartStoppable]; and attempting to access [reference] before calling [update] when this has
 * just started may throw an exception.
 *
 * This is also responsible for stopping the system if the state has reached the reference within tolerable levels
 * _and_ it is safe to stop the system.
 *
 * Both [SimpleControlSystem]s and [ChainedControlSystem]s only have one of these at its root.
 *
 * The [Reference] is usually the same as the [State], or some representation of the [State] along with derivative values
 * used for reference dynamic feed-forward.
 */
interface ReferenceTracker<in State : Any, out Reference : Any> : StartStoppable {

    /**
     * The current reference, possibly calculated from the last call to [update].
     *
     * @throws IllegalStateException if [update] has not been called since the last call to [start] _and_ so there is not
     *                               enough information to produce a reference
     */
    val reference: Reference
    /**
     * Polls if, by the previous call to [update], the state has reached the desired state within acceptable levels
     * and so we can stop the system, _and_ it is safe to stop the system and the state will stay by the reference.
     */
    val isDone: Boolean

    /**
     * Given the new current [State], updates the [reference]
     *
     * [elapsedSeconds] will be NaN on first iteration of the control loop.
     */
    fun update(currentState: State, elapsedSeconds: Double)
}

/**
 * Base implementation of a [ReferenceTracker] where only a [getReference] function that returns reference is needed;
 * automatically stores reference.
 *
 * Use [invalidateReference] to invalidate the reference (on start/stop).
 */
abstract class BaseReferenceTracker<State : Any, Reference : Any> : ReferenceTracker<State, Reference> {

    private var _reference: Reference? = null
    /**
     * Given the new current [State], returns the [reference]
     *
     * [elapsedSeconds] will be NaN on first iteration of the control loop.
     */
    protected abstract fun getReference(currentState: State, elapsedSeconds: Double): Reference

    /**
     * Invalidates the reference. Call on [start]/[stop]
     */
    protected fun invalidateReference() {
        _reference = null
    }

    override val reference: Reference
        get() = _reference ?: throw IllegalStateException("Attempted to get reference before calling [update]")

    final override fun update(currentState: State, elapsedSeconds: Double) {
        _reference = getReference(currentState, elapsedSeconds)
    }
}

/**
 * A [ReferenceTracker] in which the user sets reference manually,
 * and does not respond dynamically to inputted [State]s.
 */
class ManualReferenceTracker<State : Any, Reference : Any>(
    @Volatile override var reference: Reference,
    @Volatile override var isDone: Boolean = false
) : ReferenceTracker<State, Reference> {

    override fun update(currentState: State, elapsedSeconds: Double) {}
    override fun start() {}
    override fun stop() {}
}

/**
 * The [Plant] is the "end" of a control system;
 * i.e. representing the actuators (motors and such) and measurement devices (encoders, gyro, tracking, etc).
 *
 * This takes in [Signal]s (from a [Controller]) to set to the actuators,
 * and provides the [Measurement]s used for feedback and state estimation (used by [Observer]s).
 *
 * This is also usually the slowest part of the control system.
 *
 * Both [SimpleControlSystem]s and [ChainedControlSystem]s only have one of these at its end.
 */
interface Plant<in Signal : Any, out Measurement : Any> : StartStoppable {

    /**
     * Returns a [Measurement]; should be calculated on each get; or else stored based on the last call to [signal]
     */
    val measurement: Measurement

    /**
     * Signals the actuators with the given [signal].
     *
     * [elapsedSeconds] will be NaN on the first iteration of a control loop. It's really only here
     * for symmetry to [Controller]
     */
    fun signal(signal: Signal, elapsedSeconds: Double)
}

/**
 * Represents the controller component of a control system.
 *
 * This is an intermediary in signaling, in which a reference from a [ReferenceTracker] eventually turns into a signal fed into
 * a [Plant].
 *
 * It takes in a [Reference] state (given from a [ReferenceTracker] or another [Controller] (explained below)),
 * and measured/estimated [State]s (as given from a [Observer]), and produces a [Signal] (with the goal of
 * moving the current [State] to the [Reference]).
 *
 * This is also [StartStoppable]; and attempting to access [signal] before calling [update] when this has
 * just started may throw an exception.
 *
 * ***Important***:
 * See [ChainedControlSystem] for more info on how controllers/observers can be chained.
 *
 * A control system can have one or more _links_, each with a [Controller] and a [Observer], that
 * work can work with different [State]/[Reference] representations.
 * In this case, the output signal of the previous controller becomes the reference for the next link's controller.
 */
interface Controller<in Reference : Any, in State : Any, out Signal : Any> : StartStoppable {

    /**
     * The current signal, as calculated from the last call to [update]; or whatever represents "no signal" if
     * [update] has never been called at the last call to [StartStoppable.start]
     *
     * @throws IllegalStateException if trying to access before [update] had been called, when system just started.
     */
    val signal: Signal

    /**
     * Given the [currentState] state, the [reference] state, and the elapsedTime in nanoseconds [elapsedSeconds],
     * updates [signal] to produce a signal intended to drive the current state to the [reference].
     *
     * [elapsedSeconds] will be NaN on the first iteration of the control loop.
     */
    fun update(reference: Reference, currentState: State, elapsedSeconds: Double)
}

/**
 * Convenience extension function for both update and get signal at once
 */
fun <Reference : Any, State : Any, Signal : Any> Controller<Reference, State, Signal>.updateAndGetSignal(
    reference: Reference,
    currentState: State,
    elapsedSeconds: Double
): Signal {
    update(reference, currentState, elapsedSeconds)
    return signal
}

/**
 * Base implementation of a [Controller] where only a [getSignal] function that returns a signal is needed;
 * automatically stores signal.
 *
 * Use [invalidateSignal] to invalidate the signal (on start/stop).
 */
abstract class BaseController<in Reference : Any, in State : Any, Signal : Any> :
    Controller<Reference, State, Signal> {

    private var _signal: Signal? = null
    /**
     * Given the [currentState] state, the [reference] state, and the elapsedTime in nanoseconds [elapsedSeconds],
     * returns a signal intended to drive the current state to the [reference].
     *
     * [elapsedSeconds] of -1 indicates the first iteration of the control loop.
     */
    protected abstract fun getSignal(reference: Reference, currentState: State, elapsedSeconds: Double): Signal

    /**
     * Invalidates signal, call on [start]/[stop]
     */
    protected fun invalidateSignal() {
        _signal = null
    }

    final override val signal: Signal
        get() = _signal ?: throw IllegalStateException("Attempted to get signal before calling update")

    final override fun update(reference: Reference, currentState: State, elapsedSeconds: Double) {
        _signal = this.getSignal(reference, currentState, elapsedSeconds)
    }

    override fun stop() {
        invalidateSignal()
    }
}

/**
 * A [Controller] that does not take into account the current state; i.e. a open loop control system.
 *
 * This can be paired with a [DirectObserver] is state needs to be passed on.
 */
abstract class OpenController<in Reference : Any, Signal : Any> :
    BaseController<Reference, Any, Signal>() {

    final override fun getSignal(
        reference: Reference,
        currentState: Any,
        elapsedSeconds: Double
    ): Signal = getSignal(reference, elapsedSeconds)

    /**
     * Given the [reference] state, and the elapsedTime in nanoseconds [elapsedSeconds],
     * updates [signal] to produce a signal intended to drive the current state to the [reference].
     *
     * [elapsedSeconds] will be NaN on the first iteration of the control loop.
     */
    protected abstract fun getSignal(reference: Reference, elapsedSeconds: Double): Signal

    override fun stop() {
        invalidateSignal()
    }
}

/**
 * A controller that simply passes on its signal. Useful for when you need another observer chained.
 */
class DirectController<Reference : Any> : OpenController<Reference, Reference>() {

    override fun getSignal(reference: Reference, elapsedSeconds: Double): Reference = reference
    override fun start() {
        stop()
    }
}

/**
 * Represents the observer component of a control system.
 *
 * This is an intermediary in feedback, in which measurements from a [Plant] eventually estimate the current [State]
 *
 * It takes in a [Measurement] state (measured from a [Plant] or another [Observer] (explained below)),
 * and also the last [Signal] last used (as given from a [Controller], used to account for possible feed-through or
 * predictive updating), and estimates the current [State] from this data.
 *
 * This is also [StartStoppable]; and attempting to access [state] before calling [update] when this has
 * just started may throw an exception.
 *
 * Also, depending on implementation, [state] may need to be manually set on first construction.
 *
 * ***Important***:
 * See [ChainedControlSystem] for more info on how controllers/observers can be chained.
 *
 * A control system can have one or more _links_, each with a [Controller] and a [Observer], that
 * work can work with different [State]/[Measurement] representations.
 * In this case, an [Observer]'s inputted [Measurement] can actually be the [State] of the next link's observer.
 */
interface Observer<in Measurement : Any, in Signal : Any, State : Any> :
    StartStoppable {

    /**
     * Gets or manually resets the current state.
     *
     * THIS SHOULD BE BACKED BY A [Volatile] field so that concurrent access works properly.
     *
     * @throws IllegalStateException if attempted to get state before either calling [update] or manually setting,
     *                               when this component is first [start]ed.
     */
    var state: State

    /**
     * Given the current [measurement] and elapsed time in nanoseconds [elapsedSeconds], updates [state]
     *
     * An [elapsedSeconds] value of -1 indicates the first iteration of the control loop.
     */
    fun update(measurement: Measurement, lastSignal: Signal, elapsedSeconds: Double)
}

/**
 * Convenience extension function for both update and get signal at once
 */
fun <Measurement : Any, Signal : Any, State : Any> Observer<Measurement, Signal, State>.updateAndGetState(
    measurement: Measurement,
    lastSignal: Signal,
    elapsedSeconds: Double
): State {
    update(measurement, lastSignal, elapsedSeconds)
    return state
}

/**
 * Base implementation of a [Controller] where only a [getState] function that returns estimated state needed;
 * automatically stores signal.
 *
 * [state] can manually set on [start] or construction, too.
 *
 * Use [invalidateState] to invalidate the reference (on start/stop).
 */
abstract class BaseObserver<in Measurement : Any, in Signal : Any, State : Any> : Observer<Measurement, Signal, State> {

    @Volatile
    private var _state: State? = null

    /**
     * Given the current [measurement] and elapsed time in nanoseconds [elapsedSeconds], returns an estimated state.
     *
     * An [elapsedSeconds] value of -1 indicates the first iteration of the control loop.
     */
    protected abstract fun getState(measurement: Measurement, lastSignal: Any, elapsedSeconds: Double): State

    /**
     * Invalidates state; call on [start]/[stop]
     */
    protected fun invalidateState() {
        _state = null
    }

    override var state: State
        get() = _state
            ?: throw IllegalStateException("Attempted to get state before initialization")
        set(value) {
            _state = value
        }

    final override fun update(measurement: Measurement, lastSignal: Signal, elapsedSeconds: Double) {
        state = getState(measurement, lastSignal, elapsedSeconds)
    }
}

/**
 * A [Observer] for use when the measurement directly corresponds with the state.
 *
 * This can also be used with [OpenController]s, when chaining controllers (see [ChainedControlSystem])
 * so the state information can be passed on to the next [Controller]/[Observer] up the line.
 */
class DirectObserver<State : Any> :
    BaseObserver<State, Any, State>() {

    override fun start() {}
    override fun getState(measurement: State, lastSignal: Any, elapsedSeconds: Double): State {
        return measurement
    }

    override fun stop() {
        invalidateState()
    }
}
