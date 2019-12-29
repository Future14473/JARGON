package org.futurerobotics.jargon.statespace

import org.futurerobotics.jargon.linalg.*

/**
 * A [SignalModifier] that makes sure all values of the signal are within the [lower], [upper] bounds.
 */
class SignalLimiter(private val lower: Double, private val upper: Double) : SignalModifier {

    constructor(range: ClosedFloatingPointRange<Double>) : this(range.start, range.endInclusive)

    init {
        require(upper >= lower) { "Upper ($upper) must be >= lower ($lower)" }
    }

    override val includeInObserver: Boolean get() = true

    override fun modifySignal(matrices: DiscreteStateSpaceMatrices, x: Vec, u: Vec, y: Vec): Vec =
        u.map { it.coerceIn(lower, upper) }
}
