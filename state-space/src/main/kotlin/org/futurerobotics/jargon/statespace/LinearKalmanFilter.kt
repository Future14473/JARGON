package org.futurerobotics.jargon.statespace

import org.futurerobotics.jargon.linalg.*
import org.hipparchus.filtering.kalman.Measurement
import org.hipparchus.filtering.kalman.ProcessEstimate
import org.hipparchus.filtering.kalman.linear.LinearEvolution
import org.hipparchus.filtering.kalman.linear.LinearProcess
import org.hipparchus.linear.LUDecomposer
import org.hipparchus.linear.RealMatrix
import org.hipparchus.linear.RealVector
import org.hipparchus.filtering.kalman.linear.LinearKalmanFilter as InnerKalmanFilter

private val DECOMPOSITION = LUDecomposer(1e-11)

/**
 * A Linear Kalman Filter for a [StateSpaceObserver].
 *
 * @param noiseCovarianceProvider the [NoiseCovarianceProvider]
 * @param initialCovariance the initial process covariance
 */
class LinearKalmanFilter(
    private val noiseCovarianceProvider: NoiseCovarianceProvider,
    private val initialCovariance: Mat
) : StateSpaceObserver {

    private var initialized: Boolean = false

    private var lastNanos = 0L
    private var filter: InnerKalmanFilter<Measurement>? = null

    private val data = object : Measurement, LinearProcess<Measurement> {

        private lateinit var evolution: LinearEvolution
        private lateinit var y: Vec
        lateinit var noise: NoiseCovariance
        private var time: Double = 0.0
        fun update(
            matrices: StateSpaceMatrices,
            x: Vec,
            u: Vec,
            y: Vec,
            timeInNanos: Long
        ) {
            noise = noiseCovarianceProvider.getNoiseCovariance(matrices, x, u, y, timeInNanos)
            this.y = y
            evolution = LinearEvolution(matrices.A, matrices.B, u, noise.Q, matrices.C)
            time = timeInNanos / 1e9
        }

        override fun getCovariance(): RealMatrix = noise.R

        override fun getValue(): RealVector = y

        override fun getTime(): Double = time

        override fun getEvolution(measurement: Measurement?): LinearEvolution = evolution
    }

    private var stateOverride: Vec? = null

    override var currentState: Vec
        get() = filter?.corrected?.state ?: stateOverride
        ?: throw IllegalStateException("Initial state must be provided.")
        set(value) {
            stateOverride = value
        }

    override fun reset(initialState: Vec) {
        stateOverride = initialState
        filter = null
    }

    override fun update(matrices: DiscreteStateSpaceMatrices, u: Vec, y: Vec, timeInNanos: Long): Vec {
        val filter = filter
        val stateOverride = stateOverride
        return if (stateOverride != null || filter == null) {
            //first time
            val x = stateOverride ?: throw IllegalStateException("Initial state must be provided.")
            this.stateOverride = null
            data.update(matrices, x, u, y, timeInNanos)

            val processEstimate = ProcessEstimate(0.0, x, initialCovariance)
            this.filter = InnerKalmanFilter(DECOMPOSITION, data, processEstimate)
            x
        } else {
            var curNanos = lastNanos
            var x: Vec = filter.corrected.state
            val periodNanos = matrices.periodInNanos
            while (curNanos + periodNanos / 2 <= timeInNanos) {
                curNanos += periodNanos
                data.update(matrices, x, u, y, timeInNanos)
                x = filter.estimationStep(data).state
            }
            x
        }.also {
            lastNanos = timeInNanos
        }
    }
}
