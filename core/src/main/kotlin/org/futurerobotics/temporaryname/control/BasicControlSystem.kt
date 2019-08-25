package org.futurerobotics.temporaryname.control

import org.futurerobotics.temporaryname.control.controller.Controller
import org.futurerobotics.temporaryname.control.plant.Plant
import org.futurerobotics.temporaryname.control.reference.GenericReferenceProvider
import org.futurerobotics.temporaryname.control.stateEstimator.StateEstimator
import org.futurerobotics.temporaryname.system.LoopBasedSystem

/**
 * A basic digital control system consisting of the 4 parts shown in the constructor.
 * Upon starting, the [stateEstimator] is assumed to already hold the initial state.
 *
 * The clock is used for elapsedTime in the components shown.
 */
class BasicControlSystem<State, StateDeriv : Any, Signal, Measurement>(
    private val referenceProvider: GenericReferenceProvider<State, StateDeriv>,
    private val controller: Controller<State, StateDeriv, Signal>,
    private val plant: Plant<Signal, Measurement>,
    private val stateEstimator: StateEstimator<Measurement, State>,
    clock: Clock = Clock.Default
) : LoopBasedSystem {
    /**
     * The current state monitored by this control system.
     */
    @Volatile
    var currentState: State = stateEstimator.currentState
        private set

    private var elapsedNanos: Long = -1
    private var stopwatch = Stopwatch(clock)

    override fun init() {
        elapsedNanos = -1
        currentState = stateEstimator.currentState
    }

    /**
     * Runs one cycle of this digital control system. Reference provider is first to be updated, observer is last.
     */
    override fun tick(): Boolean {
        stopwatch.start()
        val (reference, referenceDeriv) = referenceProvider.update(currentState, elapsedNanos)
        val signal = controller.process(currentState, reference, referenceDeriv, elapsedNanos)
        val measurement = plant.signalAndMeasure(signal)
        stateEstimator.update(measurement, elapsedNanos)
        currentState = stateEstimator.currentState
        //todo: limit loop speed, maybe
        elapsedNanos = stopwatch.nanos()
        return true
    }

    override fun stop() {
        elapsedNanos = -1
        plant.stop()
    }

    /**
     * Resets the internal state using stateEstimator.resetState()
     */
    fun resetState(state: State) {
        with(stateEstimator) {
            currentState = state
            currentState = currentState
        }
    }
}