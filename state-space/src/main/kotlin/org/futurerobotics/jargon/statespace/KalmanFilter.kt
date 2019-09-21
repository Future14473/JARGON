package org.futurerobotics.jargon.statespace

import org.futurerobotics.jargon.control.Observer
import org.futurerobotics.jargon.linalg.*
import org.hipparchus.filtering.kalman.Measurement
import org.hipparchus.filtering.kalman.ProcessEstimate
import org.hipparchus.filtering.kalman.linear.LinearEvolution
import org.hipparchus.filtering.kalman.linear.LinearKalmanFilter
import org.hipparchus.filtering.kalman.linear.LinearProcess
import org.hipparchus.linear.CholeskyDecomposer
import org.hipparchus.linear.RealVector

private val cholDecomp = CholeskyDecomposer(1e-15, 1e-10)

/**
 * Kalman Filter as an [Observer]
 *
 * @param model the "base" state space model
 * @param Q the process noise covariance
 * @param R the measurement noise covariance
 *
 */
class KalmanFilter(
    private val model: DiscreteSSModel,
    private val Q: Mat,
    private val R: Mat
) : Observer<Vec, Vec, Vec> {

    private var filter: LinearKalmanFilter<Measurement>? = null
    private val measurementObj = TheMeasurement()
    private val process = TheLinearProcess()
    private var stateCovariance = steadyStateKalmanErrorCov(model, Q, R)
    private var lastUpdate: ProcessEstimate? = null
    @Volatile
    private var _state: Vec? = null
    override var state: Vec
        get() = _state ?: throw IllegalStateException("State not initialized")
        /**
         * Setting the state only provides the initial state to the Kalman Filter when first initialized.
         * Otherwise, will be overwritten pretty soon.
         */
        set(value) {
            _state = value
        }

    override fun update(measurement: Vec, lastSignal: Vec, elapsedSeconds: Double) {
        if (elapsedSeconds.isNaN() || filter == null) {
            filter = LinearKalmanFilter(
                cholDecomp,
                process,
                ProcessEstimate(
                    0.0,
                    _state ?: model.C.solve(measurement),
                    stateCovariance
                )
            )
        }
        measurementObj.value = measurement
        filter?.let {
            it.estimationStep(measurementObj)
            val corrected = it.corrected
            lastUpdate = corrected
            _state = corrected.state
        }
    }

    override fun start() {
    }

    override fun stop() {
        lastUpdate?.covariance?.let {
            stateCovariance = it
        }
        lastUpdate = null
        filter = null //reset filter
    }

    private inner class TheMeasurement : Measurement {
        @JvmField
        var value: Vec? = null

        override fun getCovariance(): Mat = R

        override fun getValue(): RealVector? = value

        override fun getTime(): Double = 0.0 //ignored
    }

    private inner class TheLinearProcess : LinearProcess<Measurement> {
        var signal: Vec = zeroVec(model.inputSize)

        private val linearEvolution = LinearEvolution(
            model.A,
            model.B,
            signal,
            Q,
            model.C
        )

        override fun getEvolution(measurement: Measurement?): LinearEvolution = linearEvolution
    }
}