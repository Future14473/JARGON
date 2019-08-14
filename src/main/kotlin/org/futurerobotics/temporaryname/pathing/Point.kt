package org.futurerobotics.temporaryname.pathing

import org.futurerobotics.temporaryname.math.Pose2d
import org.futurerobotics.temporaryname.math.Vector2d

/**
 * Holds all needed info about a specific point on a [Curve]. This does not include heading info.
 *
 * This includes the value, first derivative, and second derivatives, of position and tanAngle
 *
 * This interface is used to try to discourage unnecessary re-calculation and provide room for possible
 * calculation optimizations.
 *
 *  _Due to being parameterized by arc length, there are several nice properties/relationships among the values
 *  given explained in the docs given possibly useful for both implementation and usage._
 *
 * @see PathPoint
 */
interface CurvePoint {
    /** The length of the curve/path that this came from */
    val length: Double

    /** The position of this point, in the global coordinate frame. */
    val position: Vector2d
    /**
     * The [position]'s derivative w/rt arc length at this point.
     *
     * If this is not a PointTurn, this will always have a length of 1, else 0.0.
     * */
    val positionDeriv: Vector2d
    /**
     *  The [position]'s second derivative w/rt arc length at this point
     *
     *  If this is not a point turn, this is equivalent to `curvature zcross positionDeriv`, is always
     *  perpendicular to [positionDeriv], and always has a magnitude of [curvature], facing left of [positionDeriv]
     *  if [curvature] is positive, else facing to the right.
     */
    val positionSecondDeriv: Vector2d

    /**
     * The tangent angle of the curve at this point
     */
    val tanAngle: Double
    /**
     * The [tanAngle]'s derivative w/rt arc length at this point
     *
     * This is also equal to the [curvature] at this point.
     */
    val tanAngleDeriv: Double
    /**
     * The [tanAngle]'s second derivative w/rt arc length at this point. This may be discontinuous.
     *
     * This is also equal to the [curvature]'s derivative w/rt arc length.
     */
    val tanAngleSecondDeriv: Double
}

/** The curvature at this point, which is exactly equal to [CurvePoint.tanAngleDeriv]*/
inline val CurvePoint.curvature: Double
    get() = tanAngleDeriv

/**
 * The curvature's derivative w/rt arc length at this point, which is exactly equal
 * to [CurvePoint.tanAngleSecondDeriv]
 */
inline val CurvePoint.curvatureDeriv: Double
    get() = tanAngleSecondDeriv

/**
 * Holds all needed info about a specific point on a [Curve]. This does not include heading info.
 *
 * This includes the value, first derivative, and second derivatives, of position and tanAngle
 *
 * This interface is used to try to discourage unnecessary re-calculation and provide room for possible
 * calculation optimizations.
 *
 *  _Due to being parameterized by arc length, there are several nice properties/relationships among the values
 *  given explained in the docs given possibly useful for both implementation and usage._
 *
 * @see CurvePoint
 */
interface PathPoint : CurvePoint {
    /** The heading at this point along the path */
    val heading: Double
    /** The [heading]'s derivative w/rt arc length at this point along the path */
    val headingDeriv: Double
    /**
     * The [heading]'s second derivative w/rt arc length this point along the path. This may be instantaneously
     * discontinuous.
     */
    val headingSecondDeriv: Double
}

/**
 * The pose at this point along the path, which includes both
 * [position][PathPoint.position] and [heading][PathPoint.heading]
 */
val PathPoint.pose: Pose2d
    get() = Pose2d(position, heading)

/**
 * The pose's derivative w/rt arc length at this point along the path, which includes both
 * [positionDeriv][PathPoint.positionDeriv] and [headingDeriv][PathPoint.headingDeriv]
 */
val PathPoint.poseDeriv: Pose2d
    get() = Pose2d(positionDeriv, headingDeriv)

/**
 * The pose's derivative w/rt arc length at this point along the path, which includes both
 * [positionSecondDeriv][PathPoint.positionSecondDeriv] and [headingSecondDeriv][PathPoint.headingSecondDeriv]
 */
val PathPoint.poseSecondDeriv: Pose2d
    get() = Pose2d(positionSecondDeriv, headingSecondDeriv)
