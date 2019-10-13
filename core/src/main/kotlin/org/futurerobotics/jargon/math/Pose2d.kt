@file:Suppress("KDocMissingDocumentation")

package org.futurerobotics.jargon.math

import org.futurerobotics.jargon.linalg.*


/**
 * Represents a 2d pose, i.e. both position ([vec]) and orientation ([heading])
 *
 * @property vec The vector (position) component of this Pose
 * @property heading the heading (orientation) of this Pose
 */
data class Pose2d(val vec: Vector2d, val heading: Double) {

    /** Constructs a pose from [x] and [y] position components, and [heading] */
    constructor(x: Double, y: Double, heading: Double) : this(Vector2d(x, y), heading)

    constructor(values: DoubleArray) : this(values[0], values[1], values[2])

    /** The x component of the position ([vec]) of this Pose */
    val x: Double get() = vec.x

    /** The y component of the position ([vec]) of this Pose */
    val y: Double get() = vec.y

    operator fun plus(other: Pose2d): Pose2d = Pose2d(vec + other.vec, this.heading + other.heading)
    operator fun minus(other: Pose2d): Pose2d = Pose2d(vec - other.vec, this.heading - other.heading)
    operator fun times(v: Double): Pose2d = Pose2d(vec * v, heading * v)
    operator fun times(v: Int): Pose2d = Pose2d(vec * v, heading * v)
    operator fun div(v: Double): Pose2d = Pose2d(vec / v, heading / v)
    operator fun div(v: Int): Pose2d = Pose2d(vec / v, heading / v)
    operator fun unaryMinus(): Pose2d = Pose2d(-vec, -heading)

    /** Returns a new [Pose2d] with the _vector_ rotated by [angle] */
    fun vecRotated(angle: Double): Pose2d = Pose2d(vec.rotated(angle), this.heading)

    /** Returns a new [Pose2d] with the _angle_ rotated by [angle] */
    fun angleRotated(angle: Double): Pose2d = Pose2d(vec, angleNorm(this.heading + angle))

    /** Returns a new [Pose2d] with the heading normalized. */
    fun normalizeAngle(): Pose2d = angleNorm(heading).let {
        if (it == heading) this
        else copy(heading = it)
    }

    /** If this pose is equal to another with epsilon leniency. */
    infix fun epsEq(other: Pose2d): Boolean = vec epsEq other.vec && angleNorm(heading - other.heading) epsEq 0.0

    /** If all components are finite. */
    fun isFinite(): Boolean = vec.isFinite() && heading.isFinite()

    fun toVector(): Vec = createVec(x, y, heading)

    override fun toString(): String = "Pose2d(v:<%.4f, %.4f>, h: %.4f)".format(x, y, heading)

    companion object {
        @JvmField
        val ZERO: Pose2d = Pose2d(Vector2d.ZERO, 0.0)
    }
}

operator fun Double.times(p: Pose2d): Pose2d = p * this
operator fun Int.times(p: Pose2d): Pose2d = p * this


