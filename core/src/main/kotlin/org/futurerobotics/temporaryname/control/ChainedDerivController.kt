package org.futurerobotics.temporaryname.control

import org.futurerobotics.temporaryname.control.controller.Controller
import org.futurerobotics.temporaryname.control.plant.Plant
import org.futurerobotics.temporaryname.control.reference.ExtendedDerivRefProvider
import org.futurerobotics.temporaryname.control.stateEstimator.StateEstimator
import org.futurerobotics.temporaryname.system.LoopBasedSystem

/**
 * A control system which uses two controllers, the first controller's signal is the reference for the
 * next controller.
 *
 * An example would be using the Ramsete Controller, which takes in position and outputs drive signal, and using
 * a second controller that manages the wheels.
 */
class ChainedDerivController<
        State1, State1Deriv : Any,
        State2, State2Deriv : Any,
        Signal, Measurement>(
    val referenceProvider: ExtendedDerivRefProvider<State1, State1Deriv, State2Deriv>,
    val controller1: Controller<State1, State1Deriv, State2>,
    val controller2: Controller<State2, State2Deriv, Signal>,
    val plant: Plant<Signal, Measurement>,
    val stateEstimator1: StateEstimator<Measurement, State1>,
    val stateEstimator2: StateEstimator<Measurement, State2>,
    clock: Clock = Clock.Default
) : LoopBasedSystem {

    private val state2System: BasicControlSystem<State2, State2Deriv, Signal, Measurement> = BasicControlSystem(

    )
    var currentState1: State1 = stateEstimator1.currentState
        private set

    var currentState2: State2 = stateEstimator2.currentState
        private set

    private var elapsedNanos: Long = -1
    private var stopwatch = Stopwatch(clock)

    override fun init() {
        elapsedNanos = -1
        currentState1 = stateEstimator1.currentState
        currentState2 = stateEstimator2.currentState
    }

    override fun tick(): Boolean {
        stopwatch.start()
        val (reference1, reference1Deriv, reference2Deriv) =
            referenceProvider.updateFull(currentState1, elapsedNanos)
        val reference2 = controller1.process(currentState1, reference1, reference1Deriv, elapsedNanos)
        val signal = controller2.process(currentState2, reference2, reference2Deriv, elapsedNanos)
        val measurement = plant.signalAndMeasure(signal)
        stateEstimator1.update(measurement, elapsedNanos)
        currentState1 = stateEstimator1.currentState
        stateEstimator2.update(measurement, elapsedNanos)
        currentState2 = stateEstimator2.currentState
        elapsedNanos = stopwatch.nanos()
        return true
    }

    override fun stop() {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }
}
