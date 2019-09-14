package org.futurerobotics.temporaryname.control

import org.futurerobotics.temporaryname.control.ChainedControlSystemBuilder.Companion.start
import org.futurerobotics.temporaryname.system.LoopBasedSystem
import org.futurerobotics.temporaryname.system.StartStoppable
import java.util.concurrent.atomic.AtomicBoolean

/**
 * Base implementation of a ControlSystem.
 *
 * @param loopManager the [LoopRegulator] to use
 * @property reference the [ReferenceTracker] to use
 */
abstract class AbstractControlSystem<Tracker : ReferenceTracker<*, *>>(
    private val loopManager: LoopRegulator,
    val reference: Tracker
) : LoopBasedSystem {

    /**
     * The elapsed number of nanos timed by the stopwatch. Will be [Double.NaN] on
     * the first iteration of the control loop
     */
    protected var elapsedSeconds: Double = Double.NaN
        private set

    override fun tick(): Boolean {
        loopManager.start()
        feedBack()
        if (reference.isDone) {
            elapsedSeconds = Double.NaN
            return false
        }
        signalForward()
        elapsedSeconds = loopManager.syncAndTimeSeconds()
        return true
    }

    /**
     * Propagates reference/signal from the [ReferenceTracker] down to the [Plant] via the [Controller]s of the
     * system.
     */
    protected abstract fun signalForward()

    /**
     * Propagates measurement/state from the [Plant] up to the [ReferenceTracker] via the [Observer]s of the
     * system.
     */
    protected abstract fun feedBack()
}

/**
 * A basic digital control system consisting of the 4 parts shown in the constructor.
 * This is equivalent to a [ChainedControlSystem] with only 1 _link_.
 *
 * [StartStoppable.start] will be called on every component before the system starts,
 * and [StartStoppable.stop] will be called after the systems stops.
 *
 * The update order is: first the [plant] is measured, the [observer] is updated, the [reference] is updated,
 * The first update is meant for initialization.
 * then the [reference] is polled, [controller] is updated, and [plant] is signaled.
 */
class SimpleControlSystem<State : Any, Reference : Any, Signal : Any, Measurement : Any, Tracker : ReferenceTracker<State, Reference>>(
    loopManager: LoopRegulator,
    referenceTracker: Tracker,
    private val plant: Plant<Signal, Measurement>,
    private val controller: Controller<Reference, State, Signal>,
    private val observer: Observer<Measurement, Signal, State>
) : AbstractControlSystem<Tracker>(loopManager, referenceTracker) {

    /**
     * The current state monitored by this control system;
     *
     * @throws IllegalStateException if the [Observer]'s state has not been initialized
     */
    var currentState: State = observer.state

    override fun feedBack() {
        val measurement = plant.measurement
        observer.update(measurement, controller.signal, elapsedSeconds)
        observer.state.let {
            reference.update(it, elapsedSeconds)
            currentState = it
        }
    }

    override fun signalForward() {
        val reference = reference.reference
        controller.update(reference, observer.state, elapsedSeconds)
        plant.signal(controller.signal, elapsedSeconds)
    }

    /**
     * Resets the internal state using stateEstimator.resetState()
     */
    fun resetState(state: State) {
        observer.state = state
    }

    override fun start() {
        listOf(reference, controller, plant, observer).forEach { it.start() }
    }

    override fun stop() {
        listOf(reference, controller, plant, observer).forEach { it.stop() }
    }
}

@Suppress("UNCHECKED_CAST")
internal class ControlChainLink<State : Any, Reference : Any, Signal : Any, Measurement : Any>(
    private val controller: Controller<Reference, State, Signal>,
    val observer: Observer<Measurement, Signal, State>
) : StartStoppable {

    var state: State
        get() = observer.state
        set(value) {
            observer.state = value
        }

    fun getSignal(reference: Any, elapsedSeconds: Double): Signal {
        return controller.updateAndGetSignal(reference as Reference, observer.state, elapsedSeconds)
    }

    fun getState(measurement: Any, elapsedSeconds: Double): State {
        return observer.updateAndGetState(measurement as Measurement, controller.signal, elapsedSeconds)
    }

    override fun start() {
        controller.start()
        observer.start()
    }

    override fun stop() {
        controller.stop()
        observer.stop()
    }
}

/**
 * Represents a control system with multiple intermediary _links_; this is used to chain a series of controllers/state
 * representations from the [ReferenceTracker] down to the [Plant].
 *
 * Due to generics, this must be construted using a [ChainedControlSystemBuilder].
 *
 *
 * A chained control system has _one_ [ReferenceTracker] and _one_ [Plant], any number of _links_ in between;
 * Every link is independent and has one [Controller] and one [Observer], and working with their own [State]s.
 * Starting from the [ReferenceTracker], the the signal of one link's controller acts as the reference of the controller
 * of the next link, until reaching the [Plant]. Similarly, starting from the [Plant], the state estimated by one link's
 * [Observer] becomes the measurement of the previous link's [Observer] until finally reaching the [ReferenceTracker].
 *
 * If you draw a diagram of this on paper, it looks like a chain; hence the name.
 *
 * This is extremely useful when the initial [Reference] given by the supplied [ReferenceTracker] needs to be
 * processed multiple times in to different state representations before finally becoming a [Signal].
 *
 * For example, for a trajectory-following wheeled-robot control system:
 * *Tracker*: the [ReferenceTracker] follows a trajectory and provides the desired global pose motion as
 *      a _reference_, and updates via tracking the global pose, as _state(1)_.
 *
 * *Link1*: The first link has a controller can then be a Ramsete controller with pass-through acceleration feed-forward,
 *      that takes the previous _reference_ and the current global position _state(1)_, and produces translational
 *      and rotational turn signals as its _signal_.
 *
 * *Link2*: The next controller can be an [OpenController] or "mapper" that maps the previous _signal_ (it's _reference_) of the
 *      Ramsete controller into desired motor velocities and accelerations.
 *
 * *Link3*: The final controller can be and a linear state-space controller with acceleration feed-forward
 *      to control the actual motor voltages, to send to the plant.
 *
 * *Plant*: The plant takes the motor voltages, and returns motor positions as measurement.
 *
 * *Link3*: The linear state-space link contains has an [Observer] -- a kalman filter or a more naive filter, to map the
 *    motor positions into motor velocities.
 *
 * *Link2*: The "mapper" link's observer simply passes on the _measurement_ via an [DirectObserver].
 *
 * *Link1*: The Ramsete link's observer takes the motor velocities, and updates its tracking of the global _pose_.
 *
 * *Tracker*: The [ReferenceTracker] finally updates its step on the trajectory it's following based on the change in pose.
 */
class ChainedControlSystem<State : Any, Reference : Any, Signal : Any, Measurement : Any, Tracker : ReferenceTracker<State, Reference>>
internal constructor(
    loopManager: LoopRegulator,
    referenceTracker: Tracker,
    private val plant: Plant<Signal, Measurement>,
    private val links: List<ControlChainLink<*, *, *, *>>
) : AbstractControlSystem<Tracker>(loopManager, referenceTracker) {

    /**
     * Get's the current state at the corresponding [link]; or null if the system has never been run.
     * Due to generics, it is the user's responsibility to cast to the right type.
     */
    @Suppress("UNCHECKED_CAST")
    fun getState(link: Int): Any? {
        require(link in links.indices) { "link number out of range" }

        return links[link].state
    }

    @Suppress("UNCHECKED_CAST")
    override fun signalForward() {
        val signal = links.fold(reference.reference as Any) { signal, link ->
            link.getSignal(signal, elapsedSeconds)
        } as Signal
        plant.signal(signal, elapsedSeconds)
    }

    @Suppress("UNCHECKED_CAST")
    override fun feedBack() {
        val state = links.foldRight(plant.measurement as Any) { link, state ->
            link.getState(state, elapsedSeconds)
        } as State
        reference.update(state, elapsedSeconds)
    }

    override fun start() {
        reference.start()
        links.forEach {
            it.start()
        }
        plant.start()
    }

    override fun stop() {
        reference.stop()
        links.forEach {
            it.stop()
        }
        plant.stop()
    }
}

/**
 * Builds a control chain system, where compatibility of types are validated at compile time.
 *
 * Type inference is strongly recommended.
 *
 * This is created by [start]ing a chain with a stopwatch and referenceTracker,
 * appending any number of links, and closing with a plant.
 */
class ChainedControlSystemBuilder<State : Any, Reference : Any, Tracker : ReferenceTracker<State, Reference>>
private constructor(
    private val loopManager: LoopRegulator,
    private val referenceTracker: Tracker
) {

    private val links = ArrayList<ControlChainLink<*, *, *, *>>()

    /**
     * Represents an open end of a control chain.
     * Add links using [addLink], or close the chain with a plant using [end]
     */
    inner class ControlChainEnd<Signal : Any, Measurement : Any> internal constructor() {

        private val appended = AtomicBoolean(false)
        /**
         * Adds another link to this control chain; given compatible [controller] and [observer].
         *
         * a link can only be [addLink]ed or [end]ed once.
         */
        fun <NewSignal : Any, NewMeasurement : Any> addLink(
            controller: Controller<Signal, Measurement, NewSignal>,
            observer: Observer<NewMeasurement, NewSignal, Measurement>
        ): ControlChainEnd<NewSignal, NewMeasurement> {
            check(appended.compareAndSet(false, true)) { "Chain end already appended" }

            links += ControlChainLink(controller, observer)
            return ControlChainEnd()
        }

        infix fun <NewSignal : Any, NewMeasurement : Any> addLink(
            link: Pair<Controller<Signal, Measurement, NewSignal>,
                    Observer<NewMeasurement, NewSignal, Measurement>>
        ): ControlChainEnd<NewSignal, NewMeasurement> = link.let { (a, b) ->
            addLink(a, b)
        }

        /**
         * Closes off this control chain using the supplied compatible [plant].
         */
        infix fun end(plant: Plant<Signal, Measurement>):
                ChainedControlSystem<State, Reference, Signal, Measurement, Tracker> {
            check(appended.compareAndSet(false, true)) { "Chain end already appended" }
            return ChainedControlSystem(loopManager, referenceTracker, plant, links)
        }
    }

    companion object {
        /**
         * Starts building a control chain.
         */
        @Suppress("RemoveRedundantQualifierName")
        @JvmStatic
        fun <State : Any, Reference : Any, Tracker : ReferenceTracker<State, Reference>> start(
            loopManager: LoopRegulator,
            referenceTracker: Tracker
        ): ChainedControlSystemBuilder<State, Reference, Tracker>.ControlChainEnd<Reference, State> =
            ChainedControlSystemBuilder(loopManager, referenceTracker)
                .ControlChainEnd()
    }
}

/**
 * shorthand for [addLink]
 */
operator fun <State : Any, Reference : Any, Signal : Any, Measurement : Any, NewSignal : Any, NewMeasurement : Any, Tracker : ReferenceTracker<State, Reference>>
        ChainedControlSystemBuilder<State, Reference, Tracker>.ControlChainEnd<Signal, Measurement>.plus(
    link: Pair<Controller<Signal, Measurement, NewSignal>, Observer<NewMeasurement, NewSignal, Measurement>>
): ChainedControlSystemBuilder<State, Reference, Tracker>.ControlChainEnd<NewSignal, NewMeasurement> = addLink(link)

/**
 * shorthand for [end]
 */
operator fun <State : Any, Reference : Any, Signal : Any, Measurement : Any, Tracker : ReferenceTracker<State, Reference>>
        ChainedControlSystemBuilder<State, Reference, Tracker>.ControlChainEnd<Signal, Measurement>.minus(
    plant: Plant<Signal, Measurement>
): ChainedControlSystem<State, Reference, Signal, Measurement, Tracker> = end(plant)

