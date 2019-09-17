package org.futurerobotics.jargon.control

import org.futurerobotics.jargon.control.ChainedControlSystemBuilder.Companion.start
import org.futurerobotics.jargon.system.LoopBasedSystem
import org.futurerobotics.jargon.system.StartStoppable

/**
 * Represents some control system.
 *
 * @param State the state in which this control system operates on (the root state, if chained)
 * @param Tracker the [ReferenceTracker] used in this control system.
 */
interface ControlSystem<State : Any, Tracker : ReferenceTracker<State, *>> : LoopBasedSystem {
    /**
     * Gets the [ReferenceTracker] of this control system; possibly update reference here.
     */
    val reference: Tracker
    /**
     * Gets the current [State] monitored by this control system (the root state, if this is a chained system).
     *
     * @throws IllegalArgumentException if the state has not been initialized; see [Observer]
     */
    var currentState: State
}


/**
 * Base implementation of a ControlSystem. Not recommended for using directly.
 *
 * @param loopManager the [LoopRegulator] to use
 * @param reference the [ReferenceTracker] to use
 */
abstract class BaseControlSystem<State : Any, Tracker : ReferenceTracker<State, *>>(
    private val loopManager: LoopRegulator,
    override val reference: Tracker
) : ControlSystem<State, Tracker> {

    /**
     * The elapsed number of nanos timed by the stopwatch. Will be [Double.NaN] on
     * the first iteration of the control loop.
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
 * The update order is: first the [plant] is measured, the [observer] is updated, the [reference] is updated,
 * (The first update is meant for initialization).
 * then the [reference] is polled, [controller] is updated, and [plant] is signaled.
 */
class SimpleControlSystem<State : Any, Reference : Any, Signal : Any, Measurement : Any, Tracker : ReferenceTracker<State, Reference>>(
    loopManager: LoopRegulator,
    referenceTracker: Tracker,
    private val plant: Plant<Signal, Measurement>,
    private val controller: Controller<Reference, State, Signal>,
    private val observer: Observer<Measurement, Signal, State>
) : BaseControlSystem<State, Tracker>(loopManager, referenceTracker) {

    /**
     * The current state monitored by this control system; Can be manually reset.
     *
     * @throws IllegalStateException if attempted to get while the [Observer]'s state has not yet been initialized
     */
    override var currentState: State
        get() = observer.state
        set(value) {
            observer.state = value
        }


    override fun feedBack() {
        val measurement = plant.measurement
        observer.update(measurement, controller.signal, elapsedSeconds)
        reference.update(observer.state, elapsedSeconds)
    }

    override fun signalForward() {
        val reference = reference.reference
        controller.update(reference, observer.state, elapsedSeconds)
        plant.signal(controller.signal, elapsedSeconds)
    }

    override fun start() {
        listOf(reference, controller, plant, observer).forEach { it.start() }
    }

    override fun stop() {
        listOf(reference, controller, plant, observer).forEach { it.stop() }
    }
}

/**
 * Represents a yet-to-be added Control Chain Link.
 *
 * This is here instead of a pair of controller and observer to:
 * 1. Enforce Generics matching
 * 2. Allow implementations where both the controller and observer are supplied at the same time.
 *
 * @see [ChainedControlSystemBuilder]
 * @see [ValueControlLink]
 */
interface ControlLink<State : Any, Reference : Any, Signal : Any, Measurement : Any> {
    /**
     * The [Controller] of this control link.
     */
    val controller: Controller<Reference, State, Signal>
    /**
     * The [Observer] of this control ink.
     */
    val observer: Observer<Measurement, Signal, State>
}

/**
 * A implementation of [ControlLink] that simply holds values in fields; like a [Pair] but etter.
 *
 * @see [ControlLink]
 */
class ValueControlLink<State : Any, Reference : Any, Signal : Any, Measurement : Any>(
    override val controller: Controller<Reference, State, Signal>,
    override val observer: Observer<Measurement, Signal, State>
) : ControlLink<State, Reference, Signal, Measurement>


/**
 * A control chain link used in [ChainedControlSystem].
 * Internal due to unchecked casts.
 */
@Suppress("UNCHECKED_CAST")
internal class InternalControlLink<State : Any, Reference : Any, Signal : Any, Measurement : Any>(
    private val controller: Controller<Reference, State, Signal>,
    private val observer: Observer<Measurement, Signal, State>
) : StartStoppable {

    var state: Any
        get() = observer.state
        set(value) {
            observer.state = value as State
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
 * Represents a control system with multiple intermediary _links_; this is used to chain a series of controllers and or
 * observers, each working with different state representations, representations from the [ReferenceTracker] down to the
 * [Plant].
 *
 * (Due to generics, this must be constructed using a [ChainedControlSystemBuilder]).
 *
 * A chained control system has _one_ [ReferenceTracker] and _one_ [Plant], any number of _links_ in between;
 * Every link is independent and has one [Controller] and one [Observer], and working with their own representation of
 * State.
 *
 * Starting from the [ReferenceTracker], the the signal of one link's controller acts as the reference of the controller
 * of the next link, until reaching the [Plant]. Similarly, starting from the [Plant], the state estimated by one link's
 * [Observer] becomes the measurement of the previous link's [Observer] until finally reaching the [ReferenceTracker].
 *
 * If you draw a diagram of this, it looks like chain links, hence the name.
 *
 * Adding a link can loosely be interpreted as using _another_ control system instead of hardware output for a [Plant],
 * or interpreted as using _another_ control system as a [ReferenceTracker].
 *
 *
 * This is extremely useful when the initial [Reference] given by the supplied [ReferenceTracker] needs to be
 * processed multiple times in to different state representations before finally becoming a [Signal].
 *
 * For example, a common pattern for a trajectory-following wheeled-robot control system:
 *
 * **Reference**: the [ReferenceTracker] follows a trajectory and provides the desired global `Motion` as
 *      a _reference_, and updates via tracking the global pose. The first state representation is a POSE.
 *
 * **Link1**: The first link has a controller can then be a Ramsete controller with pass-through acceleration feed-forward,
 *      that takes the previous _reference_ and the current global position, and produces translational
 *      and rotational turn signals as its _signal_.
 *
 * **Link2**: The next controller can be an [OpenController] or "mapper" that maps the previous _signal_ (it's _reference_) of the
 *      Ramsete controller into desired _motor_ velocities and accelerations.
 *
 * **Link3**: The final controller can be and a linear state-space controller with motor acceleration feed-forward
 *      to control the actual motor voltages.
 *
 * **Plant**: The plant takes the motor voltages, and returns motor positions as measurement.
 *
 * **Link3**: The linear state-space link contains has an [Observer] -- a kalman filter or a simpler filter, to map
 *    the motor positions into motor velocities.
 *
 * **Link2**: The "mapper" link's takes the motor velocities and estimates the _bot's_ pose motion.
 *
 * **Link1**: The Ramsete link's observer takes the motor velocities, and updates its tracking of the global pose.
 *
 * **Tracker**: The [ReferenceTracker] finally updates its step on the trajectory it's following based on the change in
 *    pose.
 */
class ChainedControlSystem<State : Any, Reference : Any, Signal : Any, Measurement : Any, Tracker : ReferenceTracker<State, Reference>>
internal constructor(
    loopManager: LoopRegulator,
    referenceTracker: Tracker,
    private val plant: Plant<Signal, Measurement>,
    private val links: List<InternalControlLink<*, *, *, *>>
) : BaseControlSystem<State, Tracker>(loopManager, referenceTracker) {
    init {
        require(links.isNotEmpty())
        { "The control chain requires at least one link. If you REALLY mean no link, use DirectObserver/Controller." }
    }

    @Suppress("UNCHECKED_CAST")
    override var currentState: State
        get() = links.first().state as State
        set(value) {
            links.first().state = value
        }

    /**
     * Get's the current state at the corresponding [link] number, where 0 is the "root" link; or null if the system
     * has never been run. Due to generics, it is the user's responsibility to cast to the right type.
     */
    @Suppress("UNCHECKED_CAST")
    fun getState(link: Int): Any {
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
 * @see ChainedControlSystem
 */
class ChainedControlSystemBuilder<State : Any, Reference : Any, Tracker : ReferenceTracker<State, Reference>>
private constructor(
    private val loopManager: LoopRegulator,
    private val referenceTracker: Tracker
) {

    private val links = ArrayList<InternalControlLink<*, *, *, *>>()

    /**
     * Represents an end of a control chain currently being built.
     * Add links using [add], or close the chain with a plant using [end]
     */
    inner class ControlChainEnd<Signal : Any, Measurement : Any> internal constructor() {

        private var appended = false
        /**
         * Adds another link to this control chain; given compatible [controller] and [observer].
         *
         * A link can only be [add]ed or [end]ed once.
         */
        fun <NewSignal : Any, NewMeasurement : Any> add(
            controller: Controller<Signal, Measurement, NewSignal>,
            observer: Observer<NewMeasurement, NewSignal, Measurement>
        ): ControlChainEnd<NewSignal, NewMeasurement> {
            check(!appended) { "Chain end already appended" }
            appended = true
            links += InternalControlLink(controller, observer)
            return ControlChainEnd()
        }

        /**
         * Adds a compatible [ControlLink] to this control chain.
         *
         * A link can only be [add]ed or [end]ed once.
         */
        infix fun <NewSignal : Any, NewMeasurement : Any> add(
            link: ControlLink<Measurement, Signal, NewSignal, NewMeasurement>
        ): ControlChainEnd<NewSignal, NewMeasurement> = add(link.controller, link.observer)


        /**
         * Adds the compatible pair of a [Controller] and an [Observer] to this control chain.
         *
         * A link can only be [add]ed or [end]ed once.
         */
        infix fun <NewSignal : Any, NewMeasurement : Any> add(
            link: Pair<Controller<Signal, Measurement, NewSignal>,
                    Observer<NewMeasurement, NewSignal, Measurement>>
        ): ControlChainEnd<NewSignal, NewMeasurement> = add(link.first, link.second)


        /**
         * Closes off this control chain using the supplied compatible [plant].
         */
        infix fun end(plant: Plant<Signal, Measurement>):
                ChainedControlSystem<State, Reference, Signal, Measurement, Tracker> {
            check(!appended) { "Chain end already appended" }
            appended = true
            return ChainedControlSystem(loopManager, referenceTracker, plant, links)
        }
    }

    companion object {
        /**
         * Starts building a control chain.
         * @see [ChainedControlSystemBuilder]
         */
        @Suppress("RemoveRedundantQualifierName")
        @JvmStatic
        fun <State : Any, Reference : Any, Tracker : ReferenceTracker<State, Reference>> start(
            loopManager: LoopRegulator,
            referenceTracker: Tracker
        ): ChainedControlSystemBuilder<State, Reference, Tracker>.ControlChainEnd<Reference, State> =
            ChainedControlSystemBuilder(loopManager, referenceTracker)
                .ControlChainEnd()

        /**
         * Starts building a control chain.
         * @see [ChainedControlSystemBuilder]
         */
        @Suppress("RemoveRedundantQualifierName")
        operator fun <State : Any, Reference : Any, Tracker : ReferenceTracker<State, Reference>> invoke(
            loopManager: LoopRegulator,
            referenceTracker: Tracker
        ): ChainedControlSystemBuilder<State, Reference, Tracker>.ControlChainEnd<Reference, State> =
            ChainedControlSystemBuilder(loopManager, referenceTracker)
                .ControlChainEnd()
    }
}
