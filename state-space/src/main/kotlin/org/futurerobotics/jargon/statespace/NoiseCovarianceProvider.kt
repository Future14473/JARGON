package org.futurerobotics.jargon.statespace

import org.futurerobotics.jargon.linalg.*

/**
 * Provides [NoiseCovariance] for a [LinearKalmanFilter].
 */
interface NoiseCovarianceProvider {

    /**
     * Gets the current [NoiseCovariance] matrices for processing and measurement given
     * [matrices], predicted state [x], past signal [u], measurement [y], and current time [timeInNanos].
     */
    fun getNoiseCovariance(
        matrices: DiscreteStateSpaceMatrices,
        x: Vec,
        u: Vec,
        y: Vec,
        timeInNanos: Long
    ): NoiseCovariance
}

/**
 * A [NoiseCovarianceProvider] that always returns the given [noiseCovariance].
 */
class ConstantNoiseCovariance(private val noiseCovariance: NoiseCovariance) : NoiseCovarianceProvider {

    override fun getNoiseCovariance(
        matrices: DiscreteStateSpaceMatrices,
        x: Vec,
        u: Vec,
        y: Vec,
        timeInNanos: Long
    ): NoiseCovariance = noiseCovariance
}
