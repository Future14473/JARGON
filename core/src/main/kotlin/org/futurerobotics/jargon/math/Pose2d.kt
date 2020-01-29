@file:Suppress("KDocMissingDocumentation")

package org.futurerobotics.jargon.math

import org.futurerobotics.jargon.linalg.*
import java.io.Serializable

/**
 * Represents a 2d pose, i.e., both position ([vec]) and [heading] angle.
 *
 * The standard is that +y is counterclockwise of +x, and positive angle is counter clockwise.
 * We recommend using NorthWestUp orientation where forward is +x, and left is +y.
 * This way math still checks out, and 0 degrees is forward.
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
    val vec: Vector2d get() = Vector2d(x, y)

    operator fun plus(other: Pose2d): Pose2d = Pose2d(x + other.x, y + other.y, heading + other.heading)
    operator fun minus(other: Pose2d): Pose2d = Pose2d(x - other.x, y - other.y, heading - other.heading)
    operator fun times(v: Double): Pose2d = Pose2d(x * v, y * v, heading * v)
    operator fun div(v: Double): Pose2d = Pose2d(x / v, y / v, heading / v)
    operator fun unaryMinus(): Pose2d = Pose2d(-x, -y, -heading)

    /** Returns a new [Pose2d] with the _vector_ rotated by [angle] */
    fun vecRotated(angle: Double): Pose2d = Pose2d(vec.rotated(angle), heading)

    /** Returns a new [Pose2d] with the _angle_ rotated by [angle] */
    fun angleRotated(angle: Double): Pose2d = Pose2d(vec, heading + angle)

    /** Returns a new [Pose2d] with the heading normalized. */
    fun angleNormalized(): Pose2d = angleNorm(heading).let {
        if (it == heading) this
        else Pose2d(x, y, it)
    }

    /** If this pose is equal to another with epsilon leniency. */
    infix fun epsEq(other: Pose2d): Boolean = vec epsEq other.vec && angleNorm(heading - other.heading) epsEq 0.0

    /** If all components are finite. */
    fun isFinite(): Boolean = vec.isFinite() && heading.isFinite()

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
