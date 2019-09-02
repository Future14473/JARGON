@file:Suppress("KDocMissingDocumentation")

package org.futurerobotics.temporaryname.math

import koma.extensions.set
import koma.matrix.Matrix
import koma.zeros
import kotlin.math.*
import kotlin.random.Random

/**
 * Represents a 2d vector with values [x] and [y].
 *
 * Some calculations use cross products, in which it is calculated with the z component being 0, and a portion of the
 * result.
 * There exists [Vector2d.ZERO] for people who don't like garbage, like me.
 */
data class Vector2d(@JvmField val x: Double, @JvmField val y: Double) {

    /**@see Vector2d */
    constructor(x: Int, y: Int) : this(x.toDouble(), y.toDouble())

    /**
     * @return `||this||`, or the length of this vector
     *  @see length
     */
    val norm: Double get() = hypot(x, y)
    /**
     * @return `||this||`, or the norm of this vector
     *  @see norm
     */
    val length: Double get() = hypot(x, y)
    /**
     * @return `||this||^2`: the length of the vector squared.
     *  @see norm
     */
    val lengthSquared: Double get() = x * x + y * y
    /**
     *  `atan2(y,x)`, or the angle this vector makes with the positive x-axis from [-PI,PI]
     */
    val angle: Double get() = atan2(y, x)

    operator fun plus(v: Vector2d): Vector2d = Vector2d(x + v.x, y + v.y)
    operator fun minus(v: Vector2d): Vector2d = Vector2d(x - v.x, y - v.y)
    operator fun times(v: Double): Vector2d = Vector2d(x * v, y * v)
    operator fun times(v: Int): Vector2d = Vector2d(x * v, y * v)
    operator fun div(v: Double): Vector2d = Vector2d(x / v, y / v)
    operator fun div(v: Int): Vector2d = Vector2d(x / v, y / v)
    operator fun unaryMinus(): Vector2d = Vector2d(-x, -y)
    infix fun dot(v: Vector2d): Double = x * v.x + y * v.y
    /**
     * Returns the z component of the cross product between `this` and [v],
     *  interpreted as 3d vectors with a z component of 0.
     */
    infix fun cross(v: Vector2d): Double = x * v.y - y * v.x

    /** `|| this - v ||` */
    infix fun distTo(v: Vector2d): Double = hypot(x - v.x, y - v.y)

    /** If both components are finite */
    fun isFinite(): Boolean = x.isFinite() && y.isFinite()

    /**
     * Returns the cross product of (this interpreted as a 3d vector with a z component of 0), and the 3d vector
     *  <0,0,[z]>`, as a 2d vector, ignoring the resulting z-component of 0.
     *
     *  This is mainly used to chain 3d - cross products while still only using 2d vectors.
     *  @see [cross]
     */
    infix fun crossz(z: Double): Vector2d = Vector2d(y * z, -x * z)

    /**
     * @return `||this||^[power]: the length of this vector to the specified power.
     */
    fun lengthPow(power: Double): Double = lengthSquared.pow(power / 2)

    /** `this / ||this||', or this vector normalized */
    fun normalized(): Vector2d = this / norm

    /** this vector rotated [a] radians */
    fun rotated(a: Double): Vector2d {
        val c = cos(a)
        val s = sin(a)
        return Vector2d(x * c - y * s, x * s + y * c)
    }

    /**
     * If this vector equals [other] with a leniency of [EPSILON].
     * @see [epsEq]
     */
    infix fun epsEq(other: Vector2d): Boolean = x epsEq other.x && y epsEq other.y

    /** Gets by component. [index] must only be 0 or 1 */
    operator fun get(index: Int): Double = when (index) {
        0 -> x
        1 -> y
        else -> throw IndexOutOfBoundsException("Vector index must be 0 or 1, instead got $index")
    }

    /** Returns a new column vector matrix with this vector's data. */
    fun toColumnVector(): Matrix<Double> = zeros(2, 1).apply {
        this[0] = x
        this[1] = y
    }

    /** Returns a new row vector matrix with this vector's data. */
    fun toRowVector(): Matrix<Double> = zeros(1, 2).apply {
        this[0] = x
        this[1] = y
    }

    override fun toString(): String = "Vector2d(%.6f, %.6f)".format(x, y)

    companion object {
        /** The zero vector <0, 0> */
        @JvmField
        val ZERO: Vector2d = Vector2d(0, 0)
        /**
         * A vector with [Double.NaN] components, usually used to signify some calculation went wrong.
         */
        val NAN: Vector2d = Vector2d(Double.NaN, Double.NaN)

        /**
         * Creates a vector from polar coordinates
         * @param r the radius
         * @param a the angle in radians
         */
        @JvmStatic
        fun polar(r: Double, a: Double): Vector2d = Vector2d(r * cos(a), r * sin(a))

        /**
         * Creates a vector from polar coordinates
         * @param r the radius
         * @param a the angle in radians
         */
        @JvmStatic
        fun polar(r: Int, a: Double): Vector2d = Vector2d(r * cos(a), r * sin(a))

        /**
         * Creates a random vector
         * @param random the [Random] to use
         * @param range the range of values for x and y
         */
        @JvmStatic
        fun random(random: Random, range: Double = 1.0): Vector2d =
            Vector2d(random.nextDouble(-range, range), random.nextDouble(-range, range))
    }
}

operator fun Double.times(v: Vector2d): Vector2d = v * this
operator fun Int.times(v: Vector2d): Vector2d = v * this
/**
 * Is [Vector2d.crossz] but with operands switched.
 */
infix fun Double.zcross(v: Vector2d): Vector2d = v crossz -this

/** Convenience extension function for [Vector2d.random] */
fun Random.nextVector2d(range: Double = 1.0): Vector2d = Vector2d.random(this, range)

/**
 * Ensure's a [Vector2d]'s length is less than or equal to [maximumLength].
 */
infix fun Vector2d.coerceLengthAtMost(maximumLength: Double): Vector2d {
    require(maximumLength >= 0.0) { "maximumLength ($maximumLength) must be >= 0 as negative vector lengths are impossible" }
    return when {
        maximumLength == 0.0 -> Vector2d.ZERO
        length > maximumLength -> this * (maximumLength / length)
        else -> this
    }
}
