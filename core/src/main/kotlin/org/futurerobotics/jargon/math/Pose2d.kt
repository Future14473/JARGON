@file:Suppress("KDocMissingDocumentation")

package org.futurerobotics.jargon.math

import org.futurerobotics.jargon.linalg.*
import java.io.Serializable

/**
 * Represents a 2d pose, i.e., both position ([vector2d]) and [heading] angle.
 *
 * This should follow the standard that +y is counterclockwise of +x, and positive angle is counter
 * clockwise.
 *
 * We recommend using NorthWestUp orientation where forward is +x, and left is +y.
 * This way calculations still follows standards, and a heading of 0 is intuitively forward.
 *
 * Sometimes it is more useful to work with a linear algebra vector, so [toVec] can convert this pose into a
 * vector in (x, y, heading) order.
 *
 * Heading should be normalized using [angleNorm] or [angleNormalized], whenever it makes sense to do so.
 *
 * @property x x component of this pose
 * @property x y component of this pose
 * @property heading the heading of this pose
 * @see Vector2d
 */
data class Pose2d(
    @JvmField val x: Double,
    @JvmField val y: Double,
    @JvmField val heading: Double
) : Serializable {

    /** Constructs a pose from [vec] and [y] position components, and [heading] */
    constructor(vec: Vector2d, heading: Double) : this(vec.x, vec.y, heading)

    /** Extracts a [Vector2d] component from this pose. */
    val vector2d: Vector2d get() = Vector2d(x, y)

    operator fun plus(other: Pose2d): Pose2d = Pose2d(x + other.x, y + other.y, heading + other.heading)
    operator fun minus(other: Pose2d): Pose2d = Pose2d(x - other.x, y - other.y, heading - other.heading)
    operator fun times(v: Double): Pose2d = Pose2d(x * v, y * v, heading * v)
    operator fun div(v: Double): Pose2d = Pose2d(x / v, y / v, heading / v)
    operator fun unaryMinus(): Pose2d = Pose2d(-x, -y, -heading)

    /** Returns a new [Pose2d] with the _vector_ rotated by [angle]. */
    fun vecRotated(angle: Double): Pose2d = Pose2d(vector2d.rotated(angle), heading)

    /** Returns a new [Pose2d] with the _heading_ rotated by [angle]. Heading will not be normalized. */
    fun headingRotated(angle: Double): Pose2d = Pose2d(x, y, heading + angle)

    /** Returns a new [Pose2d] with _both the heading and vector_ rotated by [angle]. Heading will not be normalized. */
    fun fullRotated(angle: Double): Pose2d = Pose2d(vector2d.rotated(angle), heading + angle)

    /** Returns a new [Pose2d] with the heading normalized. */
    fun angleNormalized(): Pose2d = angleNorm(heading).let {
        if (it == heading) this
        else Pose2d(x, y, it)
    }

    /** If this pose is equal to another with a leniency of [EPSILON]. to account for floating point errors. Heading
     * will be normalized when compared. */
    infix fun epsEq(other: Pose2d): Boolean =
        x epsEq other.x &&
            y epsEq other.y &&
            angleNorm(heading - other.heading) epsEq 0.0

    /** If this pose is equal to another with a leniency of a given [EPSILON]. to account for floating point errors.
     * Heading
     * will be normalized when compared. */
    fun epsEq(other: Pose2d, epsilon: Double): Boolean =
        x.epsEq(other.x, epsilon) &&
            y.epsEq(other.y, epsilon) &&
            angleNorm(heading - other.heading).epsEq(0.0, epsilon)

    /** If all components are finite. */
    fun isFinite(): Boolean = x.isFinite() && y.isFinite() && heading.isFinite()

    /**
     * Converts this to a linear algebra vector, in the order of `[x, y, heading]`.
     *
     * This provides a bridge between poses and linear algebra.
     *
     * All vectors that directly represent poses should be in this order.
     */
    fun toVec(): Vec = vecOf(x, y, heading)

    override fun toString(): String = "Pose2d(x: %.4f, y: %.4f, h: %.4f)".format(x, y, heading)

    companion object {
        private const val serialVersionUID: Long = -1480830446990142354

        /** Pose with all components equal to zero. */
        @JvmField
        val ZERO: Pose2d = Pose2d(Vector2d.ZERO, 0.0)

        /**
         * Constructs a pose from values a linear algebra [vecFrom], which should have three values in [x], [y], [heading]
         * order.
         *
         * @see toVec
         */
        @JvmStatic
        fun fromVec(vec: Vec): Pose2d {
            require(vec.size == 3) { "Given vector size (${vec.size} != 3" }
            return Pose2d(vec[0], vec[1], vec[2])
        }
    }
}

operator fun Double.times(p: Pose2d): Pose2d = p * this

/**
 * Constructs a pose from values a linear algebra [vecFrom], which should have three values in [x], [y], [heading]
 * order.
 *
 * @see Pose2d.fromVec
 * @see Pose2d.toVec
 */
fun Vec.toPose(): Pose2d = Pose2d.fromVec(this)
