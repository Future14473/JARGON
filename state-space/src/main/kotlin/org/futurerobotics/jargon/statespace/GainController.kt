@file:Suppress("LocalVariableName")

package org.futurerobotics.jargon.statespace

import org.futurerobotics.jargon.linalg.*

/**
 * A [StateSpaceController] based on a gain matrix K, `u = K(r-x)`, where K may change over time.
 */
abstract class VaryingGainController : StateSpaceController {

    override fun getSignal(matrices: StateSpaceMatrices, x: Vec, r: Vec, timeInNanos: Long): Vec {
        val K = getK(matrices, x, r, timeInNanos)
        return K(r - x)
    }

    /**
     * Gets the K gain matrix, given the current [matrices], current state [x], reference [x], and time
     * in nanoseconds.
     *
     * Note that the stats will include feed-forward.
     */
    protected abstract fun getK(matrices: StateSpaceMatrices, x: Vec, r: Vec, timeInNanos: Long): Mat
}

/**
 * A [StateSpaceController] based on a K gain matrix [K], `u = K(r-x)`, where [K] is constant.
 */
class GainController(private val K: Mat) : VaryingGainController() {

    override fun getK(matrices: StateSpaceMatrices, x: Vec, r: Vec, timeInNanos: Long): Mat = K
}
