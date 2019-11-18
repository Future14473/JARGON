package org.futurerobotics.jargon.statespace

import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.util.builder

/**
 * A slightly more idiomatic way to create a [LinearKalmanFilter].
 */
class LinearKalmanFilterBuilder {

    private var noiseCovarianceProvider: NoiseCovarianceProvider? = null
    private var initialProcessCovariance: Mat? = null

    /** Sets the [noiseCovariance] to the given provider. */
    fun setNoiseCovariance(noiseCovariance: NoiseCovarianceProvider): LinearKalmanFilterBuilder = builder {
        this.noiseCovarianceProvider = noiseCovariance
    }

    /** Sets the [noiseCovariance] to the given value. */
    fun setNoiseCovariance(noiseCovariance: NoiseCovariance): LinearKalmanFilterBuilder = builder {
        this.noiseCovarianceProvider = ConstantNoiseCovarianceProvider(noiseCovariance)
    }

    /** Sets the initial process covariance to the given value. */
    fun setInitialProcessCovariance(covariance: Mat): LinearKalmanFilterBuilder = builder {
        this.initialProcessCovariance = covariance
    }

    /**
     * Builds the [LinearKalmanFilter]
     */
    fun build(): LinearKalmanFilter {
        val ncp = noiseCovarianceProvider ?: throw IllegalStateException("Noise covariance not given.")
        val ipc = initialProcessCovariance ?: throw IllegalStateException("Initial process covariance not given.")
        return LinearKalmanFilter(ncp, ipc)
    }
}
