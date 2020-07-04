package org.futurerobotics.jargon.math

import kotlin.math.abs

/**
 * Represents an interval of real numbers, represented by two endpoints, inclusive.
 *
 * Endpoints can be infinite, but cannot be NaN.
 *
 * An empty interval is represented by a start endpoint [start] greater than the end endpoint [end].
 *
 * See factory methods.
 *
 * @param start The lower bound of this interval.
 * @param end The upper bound of this interval.
 */
class Interval(val start: Double, val end: Double) {

    constructor(range: ClosedFloatingPointRange<Double>) : this(range.start, range.endInclusive)

    init {
        require(!start.isNaN()) { "start is NaN" }
        require(!end.isNaN()) { "end is NaN" }
    }

    /** If this interval is empty (has a [start] greater than [end])*/
    fun isEmpty(): Boolean = start > end

    /** If this interval is not empty. */
    fun isNotEmpty(): Boolean = !isEmpty()

    /** Gets the size of this interval. This is always non-negative; is 0 if the interval is empty. */
    val size: Double get() = if (isEmpty()) 0.0 else end - start

    /**
     * If this interval is finite.
     *
     * Will return true if [isEmpty].
     */
    fun isFinite(): Boolean = isEmpty() || (start.isFinite() && end.isFinite())

    /** @return if a value [v] is contained in the interval. */
    operator fun contains(v: Double): Boolean = v in start..end //includes empty case

    /** @return the intersection of this interval with another. */
    infix fun intersect(other: Interval): Interval {
        //if we ever get value types, replace this
        if (this.isEmpty() || other.isEmpty() || other.start > end || start > other.end) return EMPTY
        val gta: Interval
        val lta: Interval
        if (start < other.start) {
            gta = other
            lta = this
        } else {
            gta = this
            lta = other
        }
        if (gta.end <= lta.end) return gta //lta[ gta(--) ]
        return Interval(gta.start, lta.end)  //lta[ gta(--] )
    }

    override fun equals(other: Any?): Boolean = when {
        this === other -> true
        other !is Interval -> false
        else -> (isEmpty() && other.isEmpty()) || (start == other.start && end == other.end)
    }

    override fun hashCode(): Int = if (isEmpty()) -1 else 31 * start.hashCode() + end.hashCode()

    override fun toString(): String = when {
        isEmpty() -> "Interval[Empty]"
        else -> "Interval[$start, $end]"
    }

    companion object {
        /** An empty interval */
        @JvmField
        val EMPTY: Interval = Interval(0.0, -1.0)
        /** An interval spanning all real numbers. */
        @JvmField
        val REAL: Interval = Interval(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY)

        /**
         * Returns an interval by two endpoints [a] and [b], swapping endpoints if necessary so that
         * [b] is always greater than or equal to [a].
         */
        @JvmStatic
        fun between(a: Double, b: Double): Interval = if (b > a) Interval(a, b) else Interval(b, a)

        /**
         * Returns an interval using a [center] and a [radius].
         *
         * Will return an empty interval if radius < 0.
         */
        @JvmOverloads
        @JvmStatic
        fun symmetric(radius: Double, center: Double = 0.0): Interval {
            if (radius == Double.POSITIVE_INFINITY) return REAL
            return Interval(center - radius, center + radius)
        }

        /**
         * Returns an interval using a [center] and a [radius].
         *
         * Radius will be interpreted as absolute value.
         */
        @JvmOverloads
        @JvmStatic
        fun symmetricBetween(radius: Double, center: Double = 0.0): Interval = symmetric(abs(radius), center)
    }
}

/**
 * Constructs an [Interval] from [this] to [b].
 */
infix fun Double.intervalTo(b: Double): Interval = Interval(this, b)

/**
 * Constructs an regular [Interval] from [this] to [b].
 * @see Interval.between
 */
infix fun Double.regularIntervalTo(b: Double): Interval = Interval.between(this, b)

/**
 * Ensures that this value lies in the specified [Interval] i.
 * Will return [Double.NaN] if the interval is empty.
 */
fun Double.coerceIn(i: Interval): Double =
    if (i.isEmpty()) throw IllegalArgumentException("Cannot coerce to empty interval") else coerceIn(i.start, i.end)

/**
 * Returns this Double range as an interval.
 */
fun ClosedFloatingPointRange<Double>.asInterval(): Interval = Interval(this)
