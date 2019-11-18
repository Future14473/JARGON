package org.futurerobotics.jargon.statespace

import org.futurerobotics.jargon.linalg.*

/**
 * Provides [DiscreteStateSpaceMatrices] for a system to use. This may vary over time/input (be linearized/time
 * variant).
 *
 * This should include any augmentations.
 */
interface StateSpaceMatricesProvider {

    /** The number of states by this system.*/
    val numStates: Int
    /** The number of inputs by this system. */
    val numInputs: Int
    /** The number of outputs by this system. */
    val numOutputs: Int

    /** The period of discretization, in seconds. */
    val period: Double

    /**
     * Given the past state [x], past signal [u], and the current [timeInNanos], gets the current possibly
     * linearized [StateSpaceMatrices] to get the next state/predicted measurement.
     */
    fun getMatrices(x: Vec, u: Vec, timeInNanos: Long): DiscreteStateSpaceMatrices
}

/**
 * A [StateSpaceMatricesProvider] that always returns the given [matrices]
 */
class ConstantStateSpaceMatricesProvider(private val matrices: DiscreteStateSpaceMatrices) :
    StateSpaceMatricesProvider {

    override val numStates: Int get() = matrices.numStates
    override val numInputs: Int get() = matrices.numInputs
    override val numOutputs: Int get() = matrices.numOutputs
    override val period: Double get() = matrices.period

    override fun getMatrices(x: Vec, u: Vec, timeInNanos: Long): DiscreteStateSpaceMatrices = matrices
}
