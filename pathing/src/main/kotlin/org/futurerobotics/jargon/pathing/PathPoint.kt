@file:JvmMultifileClass
@file:JvmName("Paths")

package org.futurerobotics.jargon.pathing

import org.futurerobotics.jargon.math.*

/**
 * Holds info about a specific point on a [Curve]: the value, first derivative, and second derivative,
 * of position and tangent angle.
 *
 * This does not include heading (see [PathPoint]).
 *
 * Due to being parameterized by arc length, there are several nice mathematical properties/relationships among the
 * values given, that are possibly useful for both implementation and usage. These are explained in the docs.
 *
 * @see PathPoint
 */
interface CurvePoint {

    /** The length of the curve/path that this came from */
    val curveLength: Double

    /** The position of this point. */
    val position: Vector2d

    /**
     * The [position]'s derivative w/rt arc length at this point.
     *
     * If this is not a PointTurn, this will always have a length of 1, else it will be 0.0.
     *
     * - The direction of this is the [tanAngle].
     */
    val positionDeriv: Vector2d

    /**
     *  The [position]'s second derivative w/rt arc length at this point
     *
     *  If this is not a point turn
     *  - this is equivalent to `curvature zcross positionDeriv`
     *  - has a magnitude of [curvature]/[tanAngleDeriv]
     *  - is always perpendicular to [positionDeriv]
     *  - if [curvature]/[tanAngleDeriv] is positive, this faces left of [positionDeriv], else it faces to the right.
     */
    val positionSecondDeriv: Vector2d

    /**
     * The tangent angle of the curve at this point
     *
     * - This is also the direction of [positionDeriv]
     */
    val tanAngle: Double

    /**
     * The [tanAngle]'s derivative w/rt arc length at this point
     *
     * - This is equal to the [curvature].
     */
    val tanAngleDeriv: Double

    /**
     * The [tanAngle]'s second derivative w/rt arc length at this point. This may be discontinuous.
     *
     * - This is equal to the [curvature]'s derivative w/rt arc length.
     */
    val tanAngleSecondDeriv: Double
}

/** The signed curvature at this point, which is exactly equal to [CurvePoint.tanAngleDeriv]*/
inline val CurvePoint.curvature: Double
    get() = tanAngleDeriv

/**
 * The curvature's derivative w/rt arc length at this point, which is exactly equal
 * to [CurvePoint.tanAngleSecondDeriv]
 */
inline val CurvePoint.curvatureDeriv: Double
    get() = tanAngleSecondDeriv

/** Gets a position motion state for this [CurvePoint]. */
fun CurvePoint.positionMotionState(): MotionState<Vector2d> =
    MotionState(position, positionDeriv, positionSecondDeriv)

/** Gets a tanAngle motion state for this [CurvePoint]. */
fun CurvePoint.tanAngleMotionState(): RealMotionState =
    RealMotionState(tanAngle, tanAngleDeriv, tanAngleSecondDeriv)

/**
 * Holds info about a specific point on a [Path]: the value, first derivative, and second derivative,
 * of position, tangent angle, **and heading**.
 *
 * Due to being parameterized by arc length, there are several nice mathematical properties/relationships among the
 * values given, that are possibly useful for both implementation and usage. These are explained in the docs.
 *
 * @see CurvePoint
 */
interface PathPoint : CurvePoint {

    /** The heading at this point */
    val heading: Double

    /** The [heading]'s derivative w/rt arc length at this point */
    val headingDeriv: Double

    /**
     * The [heading]'s second derivative w/rt arc length this point. This may be instantaneously discontinuous.
     */
    val headingSecondDeriv: Double

    /**
     * The pose at this point, which includes both
     * [position] and [heading]
     */
    val pose: Pose2d
        get() = Pose2d(position, heading)

    /**
     * The pose's derivative w/rt arc length at this point, which includes both
     * [positionDeriv] and [headingDeriv]
     */
    val poseDeriv: Pose2d
        get() = Pose2d(positionDeriv, headingDeriv)

    /**
     * The pose's derivative w/rt arc length at this point, which includes both
     * [positionSecondDeriv] and [headingSecondDeriv]
     */
    val poseSecondDeriv: Pose2d
        get() = Pose2d(positionSecondDeriv, headingSecondDeriv)
}

/** Gets a heading motion state for this [PathPoint]. */
fun PathPoint.headingMotionState(): RealMotionState =
    RealMotionState(heading, headingDeriv, headingSecondDeriv)

/** Gets a pose motion state for this [PathPoint]. */
fun PathPoint.poseMotionState(): MotionState<Pose2d> =
    MotionState(pose, poseDeriv, poseSecondDeriv)
