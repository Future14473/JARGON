package org.futurerobotics.jargon.statespace

import org.futurerobotics.jargon.linalg.*

/**
 * Decorates state and reference vectors when given from the outside world in a [StateSpaceRunner].
 *
 * Useful if you want to encapsulate augmentations.
 *
 * StateDecorators may be applied before giving [FeedForward]s depending on the order they are added to a
 * [StateSpaceRunnerBuilder].
 */
interface StateModifier {

    /**
     * Augments an outside inputted initial state [x], also given the previous augmented state [prevXAug] if known.
     */
    fun augmentInitialState(x: Vec, prevXAug: Vec?): Vec

    /**
     * Augments the outside inputted reference, later fed into the controller.
     */
    fun augmentReference(r: Vec): Vec

    /**
     * De-augments the state; this is what the outside world will see.
     */
    fun deAugmentState(xAug: Vec): Vec
}

