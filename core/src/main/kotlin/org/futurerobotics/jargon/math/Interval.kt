package org.futurerobotics.jargon.math

import kotlin.math.abs

/**
 * Represents an Interval on the number line, represented by two endpoints, and is better than ClosedFloatingPointRange
 * If the interval is empty, both the endpoints will be NaN.
 *
 * See factory methods.
 *
 * @param a The lower bound of this interval.
 * @param b The upper bound of this interval. Can be [Double.POSITIVE_INFINITY]
 */
data class Interval(val a: Double, val b: Double) {

    /** If this interval is empty (contains no values) */
    fun isEmpty(): Boolean = a.isNaN() || b.isNaN() || a > b

    /** If this interval is not empty. */
    fun isNotEmpty(): Boolean = !isEmpty()

    /** If this interval consists of a single point. */
    fun isPoint(): Boolean = a == b

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

    /** @return if [v] is contained in this interval, with leniency at the endpoints. */
    fun epsContains(v: Double): Boolean = !isEmpty() && v in (a - EPSILON)..(b + EPSILON)

    /** @return if this interval epsilon equals the other via endpoints */
    infix fun epsEq(other: Interval): Boolean = this.isEmpty() && other.isEmpty() || a epsEq other.a && b epsEq other.b

    /** @return the intersection of this interval with another. */
    fun intersect(other: Interval): Interval {
        if (this.isEmpty() || other.isEmpty() || other.a > this.b || this.a > other.b) return EMPTY
        val gta: Interval
        val lta: Interval
        if (this.a < other.a) {
            gta = other
            lta = this
        } else {
            gta = this
            lta = other
        }
        if (gta.b <= lta.b) return gta //lta[ gta(--) ]
        return Interval(gta.a, lta.b)  //lta[ gta(--] )
    }

    override fun toString(): String = when {
        isEmpty() -> "Interval [Empty]"
        else -> "Interval [$a, $b]"
    }

    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (other !is Interval) return false

        if (a != other.a) return false
        if (b != other.b) return false

        return true
    }

    override fun hashCode(): Int {
        var result = a.hashCode()
        result = 31 * result + b.hashCode()
        return result
    }


    companion object {
        /** An empty interval */
        @JvmField
        val EMPTY: Interval = Interval(Double.NaN, Double.NaN)
        /** An interval spanning all real numbers. */
        @JvmField
        val REAL: Interval = Interval(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY)

        /**
         * Returns a interval by endpoints [a] and [b].
         *
         * Will return an empty interval if b < a, or either endpoint is `NaN`
         */
        @JvmStatic
        fun of(a: Double, b: Double): Interval {
            if (a.isNaN() || b.isNaN() || b < a) return EMPTY
            return Interval(a, b)
        }

        /**
         * Returns an interval by endpoints [a] and [b].
         *
         * Will swap endpoints if b < a.
         *
         * Will return empty interval if either endpoint is `NaN`
         */
        @JvmStatic
        fun ofRegular(a: Double, b: Double): Interval {
            if (a.isNaN() || b.isNaN()) return EMPTY
            return if (b > a) Interval(a, b) else Interval(b, a)
        }

        /**
         * Returns an interval using a [center] and a [radius].
         *
         * Will return an empty interval if radius < 0,
         *
         * or `center` or `radius` is `NaN`
         */
        @JvmOverloads
        @JvmStatic
        fun symmetric(radius: Double, center: Double = 0.0): Interval {
            if (radius.isNaN() || center.isNaN() || radius < 0) return EMPTY
            if (radius == Double.POSITIVE_INFINITY) return REAL
            return Interval(center - radius, center + radius)
        }

        /**
         * Returns an interval using a [center] and a [radius].
         *
         * Radius will be interpreted as absolute value.
         *
         * Will return an empty interval if `center` or `radius` is `NaN`
         */
        @JvmOverloads
        @JvmStatic
        fun symmetricRegular(radius: Double, center: Double = 0.0): Interval = symmetric(abs(radius), center)
    }
}

/**
 * Constructs an [Interval] from [this] to [b].
 * @see Interval.of
 */
infix fun Double.intervalTo(b: Double): Interval = Interval.of(this, b)

/**
 * Constructs an regular [Interval] from [this] to [b].
 * @see Interval.ofRegular
 */
infix fun Double.regularIntervalTo(b: Double): Interval = Interval.ofRegular(this, b)

/**
 * @return if [this] is contained in this interval, with leniency at the endpoints.
 * @see Interval.epsContains
 */
infix fun Double.epsIn(i: Interval): Boolean = i.epsContains(this)

/**
 * Ensures that this value lies in the specified [Interval] i.
 * Will return [Double.NaN] if the interval is empty.
 */
infix fun Double.coerceIn(i: Interval): Double =
    if (i.isEmpty()) throw IllegalArgumentException("Cannot coerce to empty interval") else coerceIn(i.a, i.b)

/**
 * Returns this Double range as an interval.
 */
fun ClosedFloatingPointRange<Double>.asInterval(): Interval = Interval.of(this.start, this.endInclusive)
