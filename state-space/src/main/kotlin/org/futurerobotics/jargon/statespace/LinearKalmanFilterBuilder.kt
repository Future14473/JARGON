package org.futurerobotics.jargon.statespace

import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.util.builder

/**
 * A slightly more idiomatic way to create a [LinearKalmanFilter].
 */
class LinearKalmanFilterBuilder {

    private var varyingNoiseCovariance: VaryingNoiseCovariance? = null
    private var initialProcessCovariance: Mat? = null

    /** Sets the [varyingNoiseCovariance] to the given provider. */
    fun setNoiseCovariance(varyingNoiseCovariance: VaryingNoiseCovariance): LinearKalmanFilterBuilder = builder {
        this.varyingNoiseCovariance = varyingNoiseCovariance
    }

    /** Sets the [noiseCovariance] to the given value. */
    fun setNoiseCovariance(noiseCovariance: NoiseCovariance): LinearKalmanFilterBuilder = builder {
        this.varyingNoiseCovariance = ConstantNoiseCovariance(noiseCovariance)
    }

    /** Sets the initial process covariance to the given value. */
    fun setInitialProcessCovariance(covariance: Mat): LinearKalmanFilterBuilder = builder {
        this.initialProcessCovariance = covariance
    }

    /**
     * Builds the [LinearKalmanFilter]
     */
    fun build(): LinearKalmanFilter {
        val ncp = varyingNoiseCovariance ?: throw IllegalStateException("Noise covariance not given.")
        val ipc = initialProcessCovariance ?: throw IllegalStateException("Initial process covariance not given.")
        return LinearKalmanFilter(ncp, ipc)
    }
}
