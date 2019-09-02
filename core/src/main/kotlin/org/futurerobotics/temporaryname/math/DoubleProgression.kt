package org.futurerobotics.temporaryname.math

import org.futurerobotics.temporaryname.util.replaceIf
import kotlin.math.ceil

/**
 * A progression of double values.
 * See factory methods.
 * @property first The first value of this progression.
 * @property last
 * @property step
 * @property segments The number of "fences", not fencePosts, of this double progression; equal to the number of values
 *                      - 1.
 */
class DoubleProgression
private constructor( //see factory methods
    val first: Double, val last: Double, val step: Double, val segments: Int
) : Iterable<Double> {

    /** @return if this progression is empty or not */
    fun isEmpty(): Boolean = segments < 0

    /**
     * Returns a new DoubleProgression that is this progression but reversed.
     */
    fun reversed(): DoubleProgression = DoubleProgression(last, first, -step, segments)

    override operator fun iterator(): DoubleIterator = object : DoubleIterator() {
        private val intIt = IntProgression.fromClosedRange(0, segments, 1).iterator()
        override fun hasNext() = intIt.hasNext()
        override fun nextDouble() = first + intIt.nextInt() * step.notNaNOrElse { 0.0 }
    }

    override fun equals(other: Any?): Boolean =
        this === other || (other is DoubleProgression && (isEmpty() && other.isEmpty() || first == other.first && last == other.last && step == other.step))

    override fun hashCode(): Nothing = throw UnsupportedOperationException()

    companion object {
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
            return DoubleProgression(start, step, start + step * segments, segments)
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

