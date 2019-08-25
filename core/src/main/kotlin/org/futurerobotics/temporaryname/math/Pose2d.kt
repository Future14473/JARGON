@file:Suppress("KDocMissingDocumentation")

package org.futurerobotics.temporaryname.math

import koma.extensions.get
import koma.extensions.set
import koma.matrix.Matrix
import koma.zeros

/**
 * Represents a 2d pose, i.e. both position ([vec]) and orientation ([heading])
 *
 * @property vec The vector (position) component of this Pose
 * @property heading the heading (facing direction) of this Pose
 */
data class Pose2d(val vec: Vector2d, val heading: Double) {

    /** The x component of the position ([vec]) of this Pose */
    val x: Double get() = vec.x
    /** The y component of the position ([vec]) of this Pose */
    val y: Double get() = vec.y

    /** Constructs a pose from [x] and [y] position components, and [heading] */
    constructor(x: Double, y: Double, heading: Double) : this(Vector2d(x, y), heading)

    /** Constructs a pose from a vector matrix; only takes the first 3 elements */
    constructor(matrix: Matrix<Double>) : this(matrix[0], matrix[1], matrix[2])

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

    /**
     * @return true if all components are finite.
     */
    fun isFinite(): Boolean = vec.isFinite() && heading.isFinite()

    /**
     * Returns a new column matrix with this pose's data, as [x, y, heading]
     */
    fun toColumnVector(): Matrix<Double> = zeros(3, 1).apply {
        this[0] = x
        this[1] = y
        this[2] = heading
    }

    /**
     * Returns a new row matrix with this pose's data, as [x, y, heading]
     */
    fun toRowVector(): Matrix<Double> = zeros(1, 3).apply {
        this[0] = x
        this[1] = y
        this[2] = heading
    }

    override fun toString(): String {
        return "Pose2d(v:(%.4f, %.4f), h: %.4f)".format(x, y, heading)
    }

    companion object {
        @JvmField
        val ZERO: Pose2d = Pose2d(Vector2d.ZERO, 0.0)
    }
}

operator fun Double.times(p: Pose2d): Pose2d = p * this
operator fun Int.times(p: Pose2d): Pose2d = p * this


