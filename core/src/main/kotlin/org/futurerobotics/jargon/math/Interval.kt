package org.futurerobotics.jargon.math

import kotlin.math.abs

/**
 * Represents an Interval on the number line, represented by two endpoints.
 * May have endpoints of NaN if empty.
 *
 * See factory methods.
 *
 * @param a The lower bound of this interval.
 * @param b The upper bound of this interval. Can be [Double.POSITIVE_INFINITY]
 */
class Interval(val a: Double, val b: Double) {

    constructor(range: ClosedFloatingPointRange<Double>) : this(range.start, range.endInclusive)

    init {
        require(!a.isNaN()) { "a is NaN" }
        require(!b.isNaN()) { "b is NaN" }
    }

    /** If this interval is empty (contains no values) */
    fun isEmpty(): Boolean = a > b

    /** If this interval is not empty. */
    fun isNotEmpty(): Boolean = !isEmpty()

    /** Gets the size of this interval. Is always >= 0. */
    val size: Double get() = if (isEmpty()) 0.0 else b - a

    /** If this interval consists of a single point. */
    fun isPoint(): Boolean = a == b

    /**
     * If this interval is finite.
     *
     * Will return true if [isEmpty].
     */
    fun isFinite(): Boolean = isEmpty() || (a.isFinite() && b.isFinite())

    /**
     * Gets a bound by [index], which must be 0 or 1.
     *
     * An index of 0 will return the lower bound
     * An index of 1 will return the upper bound.
     */
    operator fun get(index: Int): Double = when (index) {
        0 -> a
        1 -> b
        else -> throw IndexOutOfBoundsException("Interval index must be 0 or 1, got $index")
    }

    /** @return if [v] is contained in the interval. */
    operator fun contains(v: Double): Boolean = v in a..b //includes empty case

    /** @return the intersection of this interval with another. */
    infix fun intersect(other: Interval): Interval {
        if (this.isEmpty() || other.isEmpty() || other.a > b || a > other.b) return EMPTY
        val gta: Interval
        val lta: Interval
        if (a < other.a) {
            gta = other
            lta = this
        } else {
            gta = this
            lta = other
        }
        if (gta.b <= lta.b) return gta //lta[ gta(--) ]
        return Interval(gta.a, lta.b)  //lta[ gta(--] )
    }

    override fun equals(other: Any?): Boolean = when {
        this === other -> true
        other !is Interval -> false
        else -> (isEmpty() && other.isEmpty()) || (a == other.a && b == other.b)
    }

    override fun hashCode(): Int = if (isEmpty()) -1 else 31 * a.hashCode() + b.hashCode()

    override fun toString(): String = when {
        isEmpty() -> "Interval [Empty]"
        else -> "Interval [$a, $b]"
    }

    companion object {
        /** An empty interval */
        @JvmField
        val EMPTY: Interval = Interval(0.0, -1.0)
        /** An interval spanning all real numbers. */
        @JvmField
        val REAL: Interval = Interval(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY)

        /**
         * Returns an interval by endpoints [a] and [b].
         *
         * Will swap endpoints if b < a.
         */
        @JvmStatic
        fun ofRegular(a: Double, b: Double): Interval = if (b > a) Interval(a, b) else Interval(b, a)

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
        fun symmetricRegular(radius: Double, center: Double = 0.0): Interval = symmetric(abs(radius), center)
    }
}

/**
 * Constructs an [Interval] from [this] to [b].
 */
infix fun Double.intervalTo(b: Double): Interval = Interval(this, b)

/**
 * Constructs an regular [Interval] from [this] to [b].
 * @see Interval.ofRegular
 */
infix fun Double.regularIntervalTo(b: Double): Interval = Interval.ofRegular(this, b)

/**
 * Ensures that this value lies in the specified [Interval] i.
 * Will return [Double.NaN] if the interval is empty.
 */
fun Double.coerceIn(i: Interval): Double =
    if (i.isEmpty()) throw IllegalArgumentException("Cannot coerce to empty interval") else coerceIn(i.a, i.b)

/**
 * Returns this Double range as an interval.
 */
fun ClosedFloatingPointRange<Double>.asInterval(): Interval = Interval(this)
