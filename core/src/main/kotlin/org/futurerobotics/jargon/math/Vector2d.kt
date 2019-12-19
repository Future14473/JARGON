@file:Suppress("KDocMissingDocumentation")

package org.futurerobotics.jargon.math

import java.io.Serializable
import kotlin.math.*
import kotlin.random.Random

/**
 * Represents a 2d vector in the plane with values [x] and [y].
 *
 * The standard is that +y is counterclockwise of +x, and positive angle is counter clockwise.
 * We recommend using NorthWestUp orientation where forward is +x, and left is +y.
 * This way math still checks out, and 0 degrees is forward.
 *
 * Some calculations use cross products ([cross],[crossz]), in which it is calculated with interpreting vectors as 3d
 * vectors with a z component of 0, and only a portion of the result (that is not zero) is returned.
 */
data class Vector2d(@JvmField val x: Double, @JvmField val y: Double) : Serializable {

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

    operator fun plus(other: Vector2d): Vector2d = Vector2d(x + other.x, y + other.y)
    operator fun minus(other: Vector2d): Vector2d = Vector2d(x - other.x, y - other.y)
    operator fun times(v: Double): Vector2d = Vector2d(x * v, y * v)
    operator fun times(v: Int): Vector2d = Vector2d(x * v, y * v)
    operator fun div(v: Double): Vector2d = Vector2d(x / v, y / v)
    operator fun div(v: Int): Vector2d = Vector2d(x / v, y / v)
    operator fun unaryMinus(): Vector2d = Vector2d(-x, -y)
    /** Dot product. */
    infix fun dot(v: Vector2d): Double = x * v.x + y * v.y

    /**
     * Returns the z component of the cross product between `this` and [v],
     * interpreted as 3d vectors with a z component of 0.
     */
    infix fun cross(v: Vector2d): Double = x * v.y - y * v.x

    /** `|| this - v ||` */
    infix fun distTo(v: Vector2d): Double = hypot(x - v.x, y - v.y)

    infix fun angleTo(v: Vector2d): Double = atan2(v.y - this.y, v.x - this.x)

    /** If both components are finite */
    fun isFinite(): Boolean = x.isFinite() && y.isFinite()

    /** If any component is NaN */
    fun isNaN(): Boolean = x.isNaN() || y.isNaN()

    /**
     * Returns the cross product of (this interpreted as a 3d vector with a z component of 0), and the 3d vector
     *  <0,0,[z]>`, as a 2d vector, ignoring the resulting z-component of 0.
     *
     *  This is mainly used to chain 3d - cross products while still only using 2d vectors.
     *  @see [cross]
     */
    infix fun crossz(z: Double): Vector2d = Vector2d(y * z, -x * z)

    /**
     * `||this||^[power]: the length of this vector to the specified power.
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

    override fun toString(): String = "Vector2d(%.6f, %.6f)".format(x, y)

    companion object {
        private const val serialVersionUID: Long = -6820664735430027415
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
        @JvmOverloads
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
