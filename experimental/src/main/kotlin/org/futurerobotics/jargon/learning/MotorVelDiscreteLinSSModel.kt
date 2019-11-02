package org.futurerobotics.jargon.learning

import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.statespace.DiscreteLinearStateSpaceModel

/**
 * This extends a [DiscreteLinearStateSpaceModel] specificed for modeling wheel velocities, with a [F] matrix.
 *
 * This new model in the form of `x_(t+1) = A*x + B*u + F*sign(x)`,
 *
 * - Measurement directly corresponds with state.
 * - x is a motor velocity vector
 * - u is motor voltage signal
 * - A is system matrix
 * - B is control matrix
 * - F is the contribution of friction (can be addressed later via feed-forward).
 *
 */
interface MotorVelDiscreteLinSSModel : DiscreteLinearStateSpaceModel {

    /**
     * The friction matrix
     */
    @Suppress("PropertyName")
    val F: Mat

    /**
     * Gets the feed forward voltages based off the [F] term.
     */
    fun getFeedForward(x: Vec): Vec
}
