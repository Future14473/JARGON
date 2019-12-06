package org.futurerobotics.jargon.statespace

import org.futurerobotics.jargon.linalg.*

/**
 * Provides [NoiseCovariance] for a [LinearKalmanFilter].
 */
interface VaryingNoiseCovariance {

    /**
     * Gets the current [NoiseCovariance] matrices for processing and measurement given
     * [matrices], predicted state [x], past signal [u], measurement [y], and current time [timeInNanos].
     */
    fun getNoise(
        matrices: StateSpaceMatrices,
        x: Vec,
        u: Vec,
        y: Vec,
        timeInNanos: Long
    ): NoiseCovariance
}

/**
 * A [VaryingNoiseCovariance] that always returns the given [noiseCovariance].
 */
class ConstantNoiseCovariance(private val noiseCovariance: NoiseCovariance) : VaryingNoiseCovariance {

    override fun getNoise(matrices: StateSpaceMatrices, x: Vec, u: Vec, y: Vec, timeInNanos: Long): NoiseCovariance =
        noiseCovariance
}
