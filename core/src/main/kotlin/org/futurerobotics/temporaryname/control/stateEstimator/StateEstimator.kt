package org.futurerobotics.temporaryname.control.stateEstimator

import org.futurerobotics.temporaryname.system.StartStoppable

/**
 * Estimates the current [State] based on inputted [Measurement]s provided at regular intervals.
 *
 * This does not _have to_ include linear algebra and fancy math.
 */
interface StateEstimator<in Measurement, State> : StartStoppable {

    /**
     * Given the current [measurement] and elapsed time in nanoseconds [elapsedNanos], to update [currentState]
     *
     * An [elapsedNanos] value of -1 indicates the first iteration of the control loop.
     */
    fun update(measurement: Measurement, elapsedNanos: Long)
    /**
     * Gets or manually resets the current state
     */
    var currentState: State
}

/**
 * A [StateEstimator] for use when the measurement directly corresponds with the state.
 */
class DirectMeasurement<State>(initialState: State) :
    StateEstimator<State, State> {
    override var currentState: State = initialState
    override fun update(measurement: State, elapsedNanos: Long) {
        currentState = measurement
    }

    override fun start() {
    }

    override fun stop() {
    }
}