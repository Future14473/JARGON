package org.futurerobotics.jargon.statespace

import org.futurerobotics.jargon.blocks.Block
import org.futurerobotics.jargon.blocks.Block.Processing.IN_FIRST_ALWAYS
import org.futurerobotics.jargon.blocks.BlocksConfig
import org.futurerobotics.jargon.blocks.SingleOutputBlock
import org.futurerobotics.jargon.blocks.SystemValues
import org.futurerobotics.jargon.linalg.*
import org.hipparchus.filtering.kalman.Measurement
import org.hipparchus.filtering.kalman.ProcessEstimate
import org.hipparchus.filtering.kalman.linear.LinearEvolution
import org.hipparchus.filtering.kalman.linear.LinearKalmanFilter
import org.hipparchus.filtering.kalman.linear.LinearProcess
import org.hipparchus.linear.LUDecomposer
import org.hipparchus.linear.RealVector
import kotlin.math.roundToInt

private val DECOMP = LUDecomposer(1e-11)/*CholeskyDecomposer(
    CholeskyDecomposition.DEFAULT_RELATIVE_SYMMETRY_THRESHOLD,
    CholeskyDecomposition.DEFAULT_ABSOLUTE_POSITIVITY_THRESHOLD
)*/

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
) : SingleOutputBlock<Vec>(2, IN_FIRST_ALWAYS) {

    private var filter: LinearKalmanFilter<Measurement>? = null
    private val measurementObj = TheMeasurement()
    private val process = TheLinearProcess()
    private var stateCovariance = steadyStateKalmanErrorCov(model, Q, R)
    private var lastUpdate: ProcessEstimate? = null
    private var pastOutput: Vec? = null
    override fun doInit(): Vec? {
        lastUpdate?.covariance?.let {
            stateCovariance = it
        }
        lastUpdate = null
        filter = null //reset filter with new state, perhaps.
        return null
    }

    override fun processOutput(inputs: List<Any?>, systemValues: SystemValues): Vec {
        val measurement = inputs[0] as Vec
        val filter = filter ?: LinearKalmanFilter(
            DECOMP,
            process,
            ProcessEstimate(
                0.0,
                pastOutput ?: model.C.solve(measurement),
                stateCovariance
            )
        ).also { filter = it }
        val loopTime = systemValues.loopTime
        if (!loopTime.isNaN())
            repeat((loopTime / model.period).roundToInt().coerceAtLeast(1)) {
                measurementObj.value = measurement

                val signal = inputs[1] as Vec
                process.signal setTo signal

                filter.estimationStep(measurementObj)
            }
        lastUpdate = filter.corrected
        return filter.corrected!!.state
    }

    /** Measurement Vec [BlocksConfig.Input] */
    val measurement: BlocksConfig.Input<Vec> get() = configInput(0)
    /** Signal Vec [BlocksConfig.Input] */
    val signal: BlocksConfig.Input<Vec> get() = configInput(1)

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