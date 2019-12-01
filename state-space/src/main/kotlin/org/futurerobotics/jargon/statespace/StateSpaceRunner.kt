package org.futurerobotics.jargon.statespace

import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.util.builder

/**
 * Runs a state space system, supporting several components, and the "plant" is external.
 *
 * This can be created using a [StateSpaceRunnerBuilder]
 *
 * This operates the controller, observer, and any feed-forward/decorators.
 */
class StateSpaceRunner internal constructor(
    private val controller: StateSpaceController,
    private val observer: StateSpaceObserver,
    private val matricesProvider: StateSpaceMatricesProvider,
    private val pipings: List<StatePiping>,
    private val signalModifiers: List<SignalModifier>
) {

    private val decorators = pipings.filterIsInstance<StatePiping.StateModify>().map { it.modifier }

    private fun augmentInitialState(initialState: Vec, reset: Boolean): Vec {
        val prevXAug = if (reset) null else observer.currentState
        return decorators.fold(initialState) { x, d ->
            d.augmentInitialState(x, prevXAug)
        }
    }

    private fun deAugmentState(augState: Vec): Vec {
        return decorators.fold(augState) { xAug, d ->
            d.deAugmentState(xAug)
        }
    }

    private var lastMatrices: DiscreteStateSpaceMatrices? = null

    /** The period of discretization, in seconds. */
    val period: Double get() = matricesProvider.period

    /** The signal, including all feed-forwards. */
    var signal: Vec = zeroVec(matricesProvider.numInputs)
        private set

    /**
     * The signal, only including [FeedForward] in which [FeedForward.includeInObserver] is true.
     * This is what is used by the system.
     */
    var observerSignal: Vec = zeroVec(matricesProvider.numInputs)
        private set

    /**
     * Gets the current estimated state _after [StateModifier.deAugmentState]_,
     *
     * or set to manually override the current state _before [StateModifier.augmentInitialState]_.
     *
     * @throws IllegalStateException on _get_ if reset has never been called yet.
     *
     * @see allCurrentStates
     */
    var currentState: Vec
        get() = deAugmentState(observer.currentState)
        set(value) {
            observer.currentState = augmentInitialState(value, false)
        }

    /**
     * Gets the current estimated state _including any augmentations_,
     *
     * or set to manually override the state _including any augmentations_
     *
     * @throws IllegalStateException on _get_ if reset has never been called yet.
     *
     * @see currentState
     */
    var allCurrentStates: Vec
        get() = observer.currentState
        set(value) {
            observer.currentState = value
        }

    /**
     * Resets the system, maybe the given [initialState].
     */
    fun reset(initialState: Vec) {
        lastMatrices = null
        signal = zeroVec(matricesProvider.numInputs)
        observerSignal = zeroVec(matricesProvider.numInputs)
        observer.reset(augmentInitialState(initialState, true))
    }

    /**
     * Updates the runner, given measurement [y], reference [r], possible next reference [r1] (for reference tracking),
     * and the current [timeInNanos].
     *
     * [reset] has to have been called first.
     */
    fun update(y: Vec, r: Vec, r1: Vec?, timeInNanos: Long) {

        val lastState = observer.currentState

        val lastMatrices = lastMatrices
            ?: matricesProvider.getMatrices(lastState, observerSignal, timeInNanos)

        val x = observer.update(lastMatrices, observerSignal, y, timeInNanos)

        val matrices = matricesProvider.getMatrices(x, observerSignal, timeInNanos)
        this.lastMatrices = matrices

        val uffExt = zeroVec(matrices.numInputs)
        val uffInc = zeroVec(matrices.numInputs)

        var curR = r
        var curR1 = r1

        pipings.forEach { piping ->
            when (piping) {
                is StatePiping.StateModify -> {
                    val modifier = piping.modifier
                    curR = modifier.augmentReference(curR)
                    curR1?.let { theR1 ->
                        curR1 = modifier.augmentReference(theR1)
                    }
                }
                is StatePiping.FF -> {
                    val feedForward = piping.feedForward
                    val ff = feedForward.getFeedForward(matrices, curR, curR1)
                    if (feedForward.includeInObserver)
                        uffInc += ff
                    else
                        uffExt += ff
                }
            }
        }

        val rawU = controller.getSignal(matrices, x, curR, timeInNanos)
        val u = signalModifiers.fold(rawU) { curU, m ->
            m.modifySignal(matrices, x, curU, y)
        }
        observerSignal = u + uffInc
        signal = observerSignal + uffExt
    }
}

/**
 * A builder for a [StateSpaceRunner].
 *
 * This requires:
 *
 * - [StateSpaceMatrices]
 * - [StateSpaceObserver]
 * - [StateSpaceController]
 *
 * And can optionally include one or more of:
 *
 * - [StateModifier]
 * - [FeedForward]
 */
class StateSpaceRunnerBuilder {

    private var controller: StateSpaceController? = null
    private var observer: StateSpaceObserver? = null
    private var matricesProvider: StateSpaceMatricesProvider? = null
    private val statePipings = mutableListOf<StatePiping>()
    private val signalModifiers = mutableListOf<SignalModifier>()

    /** Sets the matrices to use the given [matricesProvider]. */
    fun setMatrices(matricesProvider: StateSpaceMatricesProvider): StateSpaceRunnerBuilder = builder {
        this.matricesProvider = matricesProvider
    }

    /** Sets the matrices to the given [matrices]. */
    fun setMatrices(matrices: DiscreteStateSpaceMatrices): StateSpaceRunnerBuilder = builder {
        this.matricesProvider = ConstantStateSpaceMatricesProvider(matrices)
    }

    /** Sets the [controller] */
    fun setController(controller: StateSpaceController): StateSpaceRunnerBuilder = builder {
        this.controller = controller
    }

    /** Sets the controller to a linear control gain controller with the given [K]. */
    fun addGainController(K: Mat): StateSpaceRunnerBuilder = builder {
        controller = GainController(K)
    }

    /** Sets the observer to the given [observer]. */
    fun setObserver(observer: StateSpaceObserver): StateSpaceRunnerBuilder = builder {
        this.observer = observer
    }

    /** Adds a given [feedForward]. */
    fun addFeedForward(feedForward: FeedForward): StateSpaceRunnerBuilder = builder {
        this.statePipings += StatePiping.FF(feedForward)
    }

    /**
     * Adds a [ReferenceTrackingFeedForward] with the given [Kff] feed-forward gain.
     */
    fun addReferenceTracking(Kff: Mat): StateSpaceRunnerBuilder = builder {
        addFeedForward(ReferenceTrackingFeedForward(Kff))
    }

    /** Adds a [stateModifier]. */
    fun addStateModifier(stateModifier: StateModifier): StateSpaceRunnerBuilder = builder {
        this.statePipings += StatePiping.StateModify(stateModifier)
    }

    /** Adds a [signalModifier]. */
    fun addSignalModifier(signalModifier: SignalModifier): StateSpaceRunnerBuilder = builder {
        this.signalModifiers += signalModifier
    }

    /**
     * Adds an observer as a [LinearKalmanFilter], configured by running the [configuration] on a
     * [LinearKalmanFilterBuilder].
     */
    @JvmSynthetic
    inline fun addKalmanFilter(configuration: LinearKalmanFilterBuilder.() -> Unit): StateSpaceRunnerBuilder = builder {
        setObserver(LinearKalmanFilterBuilder().apply(configuration).build())
    }

    /** Builds the [StateSpaceRunner]. */
    fun build(): StateSpaceRunner {
        return StateSpaceRunner(
            controller ?: throw IllegalStateException("Controller not provided"),
            observer ?: throw IllegalStateException("Observer not provided"),
            matricesProvider ?: throw IllegalStateException("Matrices not provided"),
            statePipings,
            signalModifiers
        )
    }
}

internal sealed class StatePiping {
    class StateModify(val modifier: StateModifier) : StatePiping()
    class FF(val feedForward: FeedForward) : StatePiping()
}
