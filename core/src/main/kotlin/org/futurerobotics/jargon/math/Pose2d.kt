@file:Suppress("KDocMissingDocumentation")

package org.futurerobotics.jargon.math

import org.futurerobotics.jargon.linalg.*

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
 * Heading should be normalized using [angleNorm] or [normalizeAngle], whenever it makes sense to do so.
 *
 * @property vec The vector (position) component of this Pose
 * @property heading the heading (orientation) of this Pose
 * @see Vector2d
 */
data class Pose2d(@JvmField val vec: Vector2d, @JvmField val heading: Double) : java.io.Serializable {

    /** Constructs a pose from [x] and [y] position components, and [heading] */
    constructor(x: Double, y: Double, heading: Double) : this(Vector2d(x, y), heading)

    /**
     * Constructs a pose from values within a [Vec], which should have three values in [x], [y], [heading]
     * order.
     *
     * @see toVec
     */
    constructor(values: Vec) : this(values[0], values[1], values[2]) {
        require(values.size == 3) { "Give vector size (${values.size} != 3" }
    }

    /** The x component of the position ([vec]) of this Pose */
    val x: Double get() = vec.x

    /** The y component of the position ([vec]) of this Pose */
    val y: Double get() = vec.y

    operator fun plus(other: Pose2d): Pose2d = Pose2d(vec + other.vec, heading + other.heading)
    operator fun minus(other: Pose2d): Pose2d = Pose2d(vec - other.vec, heading - other.heading)
    operator fun times(v: Double): Pose2d = Pose2d(vec * v, heading * v)
    operator fun times(v: Int): Pose2d = Pose2d(vec * v, heading * v)
    operator fun div(v: Double): Pose2d = Pose2d(vec / v, heading / v)
    operator fun div(v: Int): Pose2d = Pose2d(vec / v, heading / v)
    operator fun unaryMinus(): Pose2d = Pose2d(-vec, -heading)

    /** Returns a new [Pose2d] with the _vector_ rotated by [angle] */
    fun vecRotated(angle: Double): Pose2d = Pose2d(vec.rotated(angle), heading)

    /** Returns a new [Pose2d] with the _angle_ rotated by [angle] */
    fun angleRotated(angle: Double): Pose2d = Pose2d(vec, angleNorm(heading + angle))

    /** Returns a new [Pose2d] with the heading normalized. */
    fun normalizeAngle(): Pose2d = angleNorm(heading).let {
        if (it == heading) this
        else Pose2d(vec, it)
    }

    /** If this pose is equal to another with epsilon leniency. */
    infix fun epsEq(other: Pose2d): Boolean = vec epsEq other.vec && angleNorm(heading - other.heading) epsEq 0.0

    /** If all components are finite. */
    fun isFinite(): Boolean = vec.isFinite() && heading.isFinite()

    /**
     * Converts this to a linear algebra vector, in the order of `[x, y, heading]`. This provides a bridge between
     * poses and linear algebra.
     */
    fun toVec(): Vec = createVec(x, y, heading)

    override fun toString(): String = "Pose2d(x: %.4f, y: %.4f, h: %.4f)".format(x, y, heading)

    companion object {
        private const val serialVersionUID: Long = -1480830446990142354
        /** Pose with all components equal to zero. */
        @JvmField
        val ZERO: Pose2d = Pose2d(Vector2d.ZERO, 0.0)
    }
}

operator fun Double.times(p: Pose2d): Pose2d = p * this
operator fun Int.times(p: Pose2d): Pose2d = p * this
