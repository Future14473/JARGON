package org.futurerobotics.jargon.statespace

import org.futurerobotics.jargon.control.Block
import org.futurerobotics.jargon.control.Block.Processing.IN_FIRST_ALWAYS
import org.futurerobotics.jargon.control.ListStoreBlock
import org.futurerobotics.jargon.linalg.*
import org.hipparchus.filtering.kalman.Measurement
import org.hipparchus.filtering.kalman.ProcessEstimate
import org.hipparchus.filtering.kalman.linear.LinearEvolution
import org.hipparchus.filtering.kalman.linear.LinearKalmanFilter
import org.hipparchus.filtering.kalman.linear.LinearProcess
import org.hipparchus.linear.CholeskyDecomposer
import org.hipparchus.linear.CholeskyDecomposition
import org.hipparchus.linear.RealVector

private val DECOMP = CholeskyDecomposer(
    CholeskyDecomposition.DEFAULT_RELATIVE_SYMMETRY_THRESHOLD,
    CholeskyDecomposition.DEFAULT_ABSOLUTE_POSITIVITY_THRESHOLD
)

/**
 * A Kalman Filter [Block]. Assumes that the system always runs at the model's period.
 *
 * Inputs:
 * 1. Measurement Vector
 * 2. Signal Vector
 *
 * Outputs:
 * 1. Predicted state.
 *
 * @param model the "base" state space model
 * @param Q the process noise covariance
 * @param R the measurement noise covariance
 *
 */
class KalmanFilter(
    private val model: DiscreteLinSSModel,
    private val Q: Mat,
    private val R: Mat
) : ListStoreBlock(2, 1, IN_FIRST_ALWAYS) {

    private var filter: LinearKalmanFilter<Measurement>? = null
    private val measurementObj = TheMeasurement()
    private val process = TheLinearProcess()
    private var stateCovariance = steadyStateKalmanErrorCov(model, Q, R)
    private var lastUpdate: ProcessEstimate? = null
    private var pastOutput: Vec? = null
    override fun process(inputs: List<Any?>, outputs: MutableList<Any?>) {
        val measurement = inputs[0] as Vec
        val signal = inputs[0] as Vec

        measurementObj.value = measurement
        process.signal = signal

        if (filter == null) {
            filter = LinearKalmanFilter(
                DECOMP,
                process,
                ProcessEstimate(
                    0.0,
                    pastOutput ?: model.C.solve(measurement),
                    stateCovariance
                )
            )
        }

        filter!!.let {
            it.estimationStep(measurementObj)
            val corrected = it.corrected
            lastUpdate = corrected
            outputs[0] = corrected.state
        }
    }

    override fun init(outputs: MutableList<Any?>) {
        lastUpdate?.covariance?.let {
            stateCovariance = it
        }
        lastUpdate = null
        filter = null //reset filter with new state, perhaps.
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