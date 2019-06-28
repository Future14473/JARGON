@file:Suppress("KDocMissingDocumentation")

package org.futurerobotics.temporaryname.math

/** Used to convey information about value and first and second derivatives of a numerical value. */
interface Derivatives {
    /** The value */
    val value: Double
    /** The value's derivative */
    val valueDeriv: Double
    /** The value's second derivative */
    val valueSecondDeriv: Double
}

/** A simple [Derivatives] implementation that holds value by fields */
class ValueDerivatives(
    override val value: Double, override val valueDeriv: Double, override val valueSecondDeriv: Double
) : Derivatives

/**
 * Used to convey information about value and first and second derivatives of a [Vector2d] value
 */
interface VectorDerivatives {
    /** The vector */
    val vec: Vector2d
    /** The vectors's derivative */
    val vecDeriv: Vector2d
    /** The vectors's second derivative */
    val vecSecondDeriv: Vector2d
}

/** A simple [VectorDerivatives] implementation using only fields */
class ValueVectorDerivatives(
    override val vec: Vector2d, override val vecDeriv: Vector2d, override val vecSecondDeriv: Vector2d
) : VectorDerivatives

/**
 * Used to convey information about value and first and second derivatives of a [Pose2d] value
 */
interface PoseDerivatives {
    /** The pose */
    val pose: Pose2d
    /** The pose's derivative */
    val poseDeriv: Pose2d
    /** The pose's second derivative */
    val poseSecondDeriv: Pose2d
}

/** A simple [PoseDerivatives] implementation using only fields */
class ValuePoseDerivatives(
    override val pose: Pose2d, override val poseDeriv: Pose2d, override val poseSecondDeriv: Pose2d
) : PoseDerivatives


operator fun PoseDerivatives.plus(d: PoseDerivatives): ValuePoseDerivatives =
    ValuePoseDerivatives(pose + d.pose, poseDeriv + d.poseDeriv, poseSecondDeriv + d.poseSecondDeriv)

operator fun PoseDerivatives.minus(d: PoseDerivatives): ValuePoseDerivatives =
    ValuePoseDerivatives(pose - d.pose, poseDeriv - d.poseDeriv, poseSecondDeriv - d.poseSecondDeriv)

operator fun PoseDerivatives.times(
    v: Double
): ValuePoseDerivatives = ValuePoseDerivatives(pose * v, poseDeriv * v, poseSecondDeriv * v)

operator fun PoseDerivatives.times(v: Int): ValuePoseDerivatives = this * v.toDouble()
operator fun PoseDerivatives.div(v: Double): ValuePoseDerivatives =
    ValuePoseDerivatives(pose / v, poseDeriv / v, poseSecondDeriv / v)

operator fun PoseDerivatives.div(v: Int): ValuePoseDerivatives = this / v.toDouble()