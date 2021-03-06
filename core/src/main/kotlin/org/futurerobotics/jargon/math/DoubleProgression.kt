package org.futurerobotics.jargon.math

import kotlin.math.ceil

/**
 * A progression of double values.
 *
 * See factory methods.
 * @param first The first value of this progression.
 * @param last The last value of this progression.
 * @param step the step size of this progression.
 * @param segments The number of "segments" in this progression, equal to the number of values - 1.
 */
class DoubleProgression private constructor(
    val first: Double,
    val last: Double,
    val step: Double,
    val segments: Int
) : Iterable<Double> {

    /**
     * If this progression is empty or not
     */
    fun isEmpty(): Boolean = segments < 0

    /**
     * Returns a new [DoubleProgression] that is this progression but reversed in direction.
     */
    fun reversed(): DoubleProgression = DoubleProgression(last, first, -step, segments)

    override fun iterator(): DoubleIterator = object : DoubleIterator() {
        private val it = (0..segments).iterator()
        override fun hasNext() = it.hasNext()
        override fun nextDouble() = first + it.nextInt() * step
    }

    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (other !is DoubleProgression) return false

        if (first != other.first) return false
        if (last != other.last) return false
        if (step != other.step) return false
        if (segments != other.segments) return false

        return true
    }

    override fun hashCode(): Int {
        var result = first.hashCode()
        result = 31 * result + last.hashCode()
        result = 31 * result + step.hashCode()
        result = 31 * result + segments
        return result
    }

    companion object {
        /**
         * Creates a [DoubleProgression] from a closed range and given step.
         * All values must be finite, and step must not be 0.
         */
        @JvmStatic
        fun fromClosedRange(start: Double, endInclusive: Double, step: Double): DoubleProgression {
            require(start.isFinite()) { "start ($start) should be finite" }
            require(endInclusive.isFinite()) { "endInclusive ($endInclusive) should be finite" }
            require(step.isFinite()) { "step ($step) should be finite" }
            require(step != 0.0) { "step ($step) must be non-zero" }
            val segments =
                ceilIfClose((endInclusive - start) / step)
                    .let { if (it < 0) -1 else it }
            return DoubleProgression(start, start + step * segments, step, segments)
        }

        /**
         * Creates a [DoubleProgression] from a closed range, and a given number of segments.
         * The progression will have [segments] + 1 values.
         */
        @JvmStatic
        fun fromNumSegments(start: Double, endInclusive: Double, segments: Int): DoubleProgression {
            require(start.isFinite()) { "start ($start) should be finite" }
            require(endInclusive.isFinite()) { "endInclusive ($endInclusive) should be finite" }
            require(segments >= 0) { "Number of segments ($segments) must be >= 0" }
            return DoubleProgression(
                start,
                endInclusive,
                ((endInclusive - start) / segments).ifNonFinite { 0.0 },
                segments
            )
        }
    }
}

private fun ceilIfClose(v: Double): Int {
    val ceil = ceil(v)
    return if (v epsEq ceil) ceil.toInt() else v.toInt()
}

