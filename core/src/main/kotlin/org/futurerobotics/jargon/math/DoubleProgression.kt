package org.futurerobotics.jargon.math

import org.futurerobotics.jargon.util.replaceIf
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
) : Iterable<Double>, java.io.Serializable {

    /**
     *  If this progression is empty or not
     *  */
    fun isEmpty(): Boolean = segments < 0

    /**
     * Returns a new DoubleProgression that is this progression but reversed in direction.
     */
    fun reversed(): DoubleProgression = DoubleProgression(last, first, -step, segments)

    override operator fun iterator(): DoubleIterator = object : DoubleIterator() {
        private val intIt = (0..segments).iterator()
        override fun hasNext() = intIt.hasNext()
        override fun nextDouble() = first + intIt.nextInt() * step.ifNan { 0.0 }
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
        private const val serialVersionUID: Long = -2319222987250823882
        /**
         * Creates a [DoubleProgression] from a closed range and given step.
         * All values must be finite, and step must not be 0
         */
        @JvmStatic
        fun fromClosedRange(start: Double, endInclusive: Double, step: Double): DoubleProgression {
            require(start.isFinite()) { "start ($start) should be finite" }
            require(endInclusive.isFinite()) { "endInclusive ($endInclusive) should be finite" }
            require(step.isFinite()) { "step ($step) should be finite" }
            require(step != 0.0) { "Step ($step) must be non-zero" }
            val segments = (ceilIfClose((endInclusive - start) / step)).replaceIf({ it < 0 }) { -1 }
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
            require(start != endInclusive) { "first and last cannot be same value" }
            require(segments >= 0) { "Number of segments must be > 0, got $segments instead" }
            return DoubleProgression(start, endInclusive, (endInclusive - start) / segments, segments)
        }
    }
}

private fun ceilIfClose(v: Double): Int {
    val ceil = ceil(v)
    return if (v epsEq ceil) ceil.toInt() else v.toInt()
}

