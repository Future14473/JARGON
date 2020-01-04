package org.futurerobotics.jargon.statespace

import org.futurerobotics.jargon.linalg.*

/**
 * A state space controller, that produces signals, _not including_ [FeedForward]s.
 *
 * This should take into account any augmentations.
 */
interface StateSpaceController {

    /**
     * Given the current [matrices], estimated state [x], reference state [r], and the current [timeInNanos],
     * returns a signal vector _without feed forward_.
     */
    fun getSignal(matrices: StateSpaceMatrices, x: Vec, r: Vec, timeInNanos: Long): Vec
}
