package org.futurerobotics.jargon.statespace

import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.util.builder

/**
 * Runs a state space system, supporting several components, and the "plant" is external.
 *
 * This can be created using a [StateSpaceRunnerBuilder]
 *
 * This operates the controller, observer, and any feed-forward/decorators.
 *
 * This is experimental as this api is subject to change.
 */
@ExperimentalStateSpace
class StateSpaceRunner internal constructor(
    private val controller: StateSpaceController,
    private val observer: StateSpaceObserver,
    private val matricesProvider: StateSpaceMatricesProvider,
    private val pipings: List<StatePiping>,
    private val signalModifiers: List<SignalModifier>
) {

    private val stateModifiers = pipings.mapNotNull { (it as? StatePiping.StateModify)?.modifier }

    private var lastMatrices: DiscreteStateSpaceMatrices? = null

    /** The period which this run, in seconds. */
    val period: Double get() = matricesProvider.period

    /** The signal, including all feed-forwards. */
    var signal: Vec = Vec(matricesProvider.numInputs)
        private set

    /**
     * The signal, only including [FeedForward]s in which [FeedForward.includeInObserver] is true.
     * This is what is used by the system.
     */
    var observerSignal: Vec = Vec(matricesProvider.numInputs)
        private set

    /**
     * Gets the current estimated state _after [StateModifier.deAugmentState]_,
     *
     * or set to manually override the current state _before [StateModifier.augmentInitialState]_.
     *
     * @throws IllegalStateException on _get_ if reset has never been called yet.
     *
     * @see currentStateModified
     */
    var currentState: Vec
        get() = deAugmentState(observer.currentState)
        set(value) {
            observer.currentState = augmentInitialState(value, false)
        }

    private fun deAugmentState(xAug: Vec): Vec = stateModifiers.fold(xAug) { curX, d ->
        d.deAugmentState(curX)
    }

    private fun augmentInitialState(xRaw: Vec, reset: Boolean): Vec {
        val prevXAug = if (reset) null else observer.currentState
        return stateModifiers.fold(xRaw) { curX, d ->
            d.augmentInitialState(curX, prevXAug)
        }
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
    var currentStateModified: Vec
        get() = observer.currentState
        set(value) {
            observer.currentState = value
        }

    /**
     * Resets the system, maybe the given [initialState].
     */
    fun reset(initialState: Vec) {
        lastMatrices = null
        signal = Vec(matricesProvider.numInputs)
        observerSignal = Vec(matricesProvider.numInputs)
        observer.reset(augmentInitialState(initialState, true))
    }

    /**
     * Updates the runner, given measurement [y], reference [r], possible next reference [r1] (for reference tracking),
     * and the current [timeInNanos].
     *
     * [reset] has to have been called first.
     */
    fun update(y: Vec, r: Vec, r1: Vec?, timeInNanos: Long) {

        val xPrev = observer.currentState
        val uPrev = observerSignal

        val lastMatrices = lastMatrices
            ?: matricesProvider.getMatrices(xPrev, uPrev, timeInNanos)

        val x = observer.update(lastMatrices, uPrev, y, timeInNanos)

        val matrices = matricesProvider.getMatrices(x, uPrev, timeInNanos)
        this.lastMatrices = matrices

        val uffExt = Vec(matrices.numInputs)
        val uffInc = Vec(matrices.numInputs)

        var curR = r
        var curR1 = r1

        pipings.forEach { piping ->
            when (piping) {
                is StatePiping.StateModify -> {
                    val modifier = piping.modifier
                    curR = modifier.augmentReference(curR)
                    curR1?.let {
                        curR1 = modifier.augmentReference(it)
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

        val rawUInc = controller.getSignal(matrices, x, curR, timeInNanos) + uffInc
        val uInc = signalModifiers.fold(rawUInc) { curU, m ->
            if (m.includeInObserver)
                m.modifySignal(matrices, x, curU, y)
            else curU
        }
        observerSignal = uInc

        val rawUExt = uInc + uffExt
        val uExt = signalModifiers.fold(rawUExt) { curU, m ->
            if (!m.includeInObserver)
                m.modifySignal(matrices, x, curU, y)
            else curU
        }
        signal = uExt
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
@ExperimentalStateSpace
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
        this.matricesProvider = ConstantStateSpaceMatrices(matrices)
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
