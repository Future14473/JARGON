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
import org.hipparchus.filtering.kalman.linear.LinearProcess
import org.hipparchus.linear.LUDecomposer
import org.hipparchus.linear.RealVector
import kotlin.math.roundToLong
import org.hipparchus.filtering.kalman.linear.LinearKalmanFilter as InnerKalmanFilter

private val DECOMP = LUDecomposer(1e-11)

/**
 * A Kalman Filter [Block]. Assumes that the system always runs at the model's period.
 *
 * @param model the "base" state space model
 * @param Q the process noise covariance
 * @param R the measurement noise covariance
 *
 */
class LinearKalmanFilter(
    private val model: DiscreteLinSSModelImpl,
    private val Q: Mat,
    private val R: Mat
) : SingleOutputBlock<Vec>(2, IN_FIRST_ALWAYS) {

    /** Measurement vector input */
    val measurement: BlocksConfig.Input<Vec> get() = configInput(0)
    /** Signal vector input */
    val signal: BlocksConfig.Input<Vec> get() = configInput(1)

    private var filter: InnerKalmanFilter<Measurement>? = null
    private val measurementObj = object : Measurement {
        @JvmField
        var value: Vec? = null
        @JvmField
        var timeNanos = 0L

        override fun getCovariance(): Mat = R

        override fun getValue(): RealVector? = value

        override fun getTime(): Double = timeNanos / 1e9
    }
    private val linearProcess = object : LinearProcess<Measurement> {
        val signal: Vec = zeroVec(model.inputSize)

        private val linearEvolution
            get() = LinearEvolution(model.A, model.B, signal, Q, model.C)

        override fun getEvolution(measurement: Measurement?): LinearEvolution = linearEvolution
    }
    private var stateCovariance = steadyStateKalmanErrorCov(model, Q, R)
    private var lastUpdate: ProcessEstimate? = null
    private var pastOutput: Vec? = null
    override fun initialValue(): Vec? {
        lastUpdate?.covariance?.let {
            stateCovariance = it
        }
        measurementObj.value = null
        measurementObj.timeNanos = 0L
        lastUpdate = null
        filter = null //reset filter with new state, perhaps.
        return null
    }

    override fun processOutput(inputs: List<Any?>, systemValues: SystemValues): Vec {
        val measurement = inputs[0] as Vec
        val filter = filter ?: InnerKalmanFilter(
            DECOMP, linearProcess, ProcessEstimate(
                systemValues.totalTime, pastOutput ?: model.C.solve(measurement),
                stateCovariance
            )
        ).also { filter = it }
        val loopTime = systemValues.loopTime
        val lastNanos = measurementObj.timeNanos
        val elapsedNanos = (loopTime * 1e9).roundToLong()
        val nowNanos = lastNanos + elapsedNanos
        val periodNanos = (model.period * 1e9).roundToLong()
        if (loopTime != 0.0) {
            var curNanos = lastNanos
            while (curNanos + periodNanos <= nowNanos) {
                curNanos += periodNanos
                measurementObj.value = measurement
                measurementObj.timeNanos = curNanos

                val signal = inputs[1] as Vec
                linearProcess.signal setTo signal

                filter.estimationStep(measurementObj)
            }
        }
        filter.corrected.let {
            lastUpdate = it
            return it.state
        }
    }
}
