package org.futurerobotics.jargon.statespace

import org.futurerobotics.jargon.blocks.Block
import org.futurerobotics.jargon.blocks.Block.Processing.ALWAYS
import org.futurerobotics.jargon.blocks.PrincipalOutputBlock
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
 * A Kalman Filter [Block]. Works best when the system always runs near the model's period.
 *
 * @param model the "base" state space model
 * @param Q the process noise covariance
 * @param R the measurement noise covariance
 *
 */
class LinearKalmanFilter(
    private val model: DiscreteLinearStateSpaceModel,
    private val Q: Mat,
    private val R: Mat
) : PrincipalOutputBlock<Vec>(ALWAYS) {

    /** Measurement vector input */
    val measurement: Input<Vec> = newInput()
    /** Signal vector input */
    val signal: Input<Vec> = newInput()

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
    override fun init() {
        lastUpdate?.covariance?.let {
            stateCovariance = it
        }
        measurementObj.value = null
        measurementObj.timeNanos = 0L
        lastUpdate = null
        filter = null //reset filter with new state, perhaps.
    }

    override fun Context.getOutput(): Vec {
        val measurement = measurement.get
        val filter = filter ?: InnerKalmanFilter(
            DECOMP, linearProcess, ProcessEstimate(
                totalTime, pastOutput
                    ?: model.C.solve(measurement),
                stateCovariance
            )
        ).also { filter = it }
        val loopTime = loopTime
        val lastNanos = measurementObj.timeNanos
        val elapsedNanos = (loopTime * 1e9).roundToLong()
        val nowNanos = lastNanos + elapsedNanos
        val periodNanos = (model.period * 1e9).roundToLong()
        if (loopTime != 0.0) {
            var curNanos = lastNanos
            do {
                curNanos += periodNanos
                measurementObj.value = measurement
                measurementObj.timeNanos = curNanos

                val signal = signal.get
                linearProcess.signal setTo signal

                filter.estimationStep(measurementObj)
            } while (curNanos + periodNanos <= nowNanos)
        }
        filter.corrected.let {
            lastUpdate = it
            return it.state
        }
    }
}
