package org.futurerobotics.jargon.statespace

import org.futurerobotics.jargon.linalg.*

/**
 * A state space controller, that produces signals, _not including_ feed-forward.
 *
 * This should include any augmentations.
 *
 * [FeedForward]s may then be applied.
 */
interface StateSpaceController {

    /**
     * Given the current [matrices], estimated state [x], reference state [r], and the current [timeInNanos],
     * returns a signal vector _without feed forward_.
     */
    fun getSignal(matrices: StateSpaceMatrices, x: Vec, r: Vec, timeInNanos: Long): Vec
}
