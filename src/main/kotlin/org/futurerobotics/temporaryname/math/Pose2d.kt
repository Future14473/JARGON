@file:Suppress("NOTHING_TO_INLINE", "KDocMissingDocumentation", "unused")

package org.futurerobotics.temporaryname.math

/**
 * Represents a 2d pose, i.e. both position ([vec]) and orientation ([heading])
 *
 * @property vec The vector (position) component of this Pose
 * @property heading the heading (facing direction) of this Pose
 */
data class Pose2d(val vec: Vector2d, val heading: Double) {
    /** Constructs a pose from [x] and [y] position components, and [heading] */
    constructor(x: Double, y: Double, heading: Double) : this(Vector2d(x, y), heading)

    /** The x component of the position ([vec]) of this Pose */
    val x: Double get() = vec.x
    /** The y component of the position ([vec]) of this Pose */
    val y: Double get() = vec.y

    operator fun plus(that: Pose2d): Pose2d = Pose2d(vec + that.vec, this.heading + that.heading)
    operator fun minus(that: Pose2d): Pose2d = Pose2d(vec - that.vec, this.heading - that.heading)
    operator fun times(v: Double): Pose2d = Pose2d(vec * v, heading * v)
    operator fun times(v: Int): Pose2d = Pose2d(vec * v, heading * v)
    operator fun div(v: Double): Pose2d = Pose2d(vec / v, heading / v)
    operator fun div(v: Int): Pose2d = Pose2d(vec / v, heading / v)
    /** Returns if this pose is equal to another with epsilon leniency. */
    fun epsEq(other: Pose2d): Boolean {
        return vec epsEq other.vec && heading epsEq other.heading
    }

    override fun hashCode(): Nothing = throw UnsupportedOperationException()
    override fun toString(): String {
        return "Pose2d(v:(%.4f, %.4f), h: %.4f)".format(x, y, heading)
    }

    override fun equals(other: Any?): Boolean =
        this === other || (other is Pose2d && vec == other.vec && heading == other.heading)
}

operator fun Double.times(p: Pose2d): Pose2d = p * this
operator fun Int.times(p: Pose2d): Pose2d = p * this


