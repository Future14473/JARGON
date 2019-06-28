package org.futurerobotics.temporaryname.pathing.path

import org.futurerobotics.temporaryname.math.Pose2d
import org.futurerobotics.temporaryname.math.Vector2d

/**
 * Holds all useful info about a point along a [Path].
 *
 * This includes the value, first derivative, and second derivatives, of: position, tangentAngle, and heading.
 *
 * This is used to avoid re-calculation and possible calculation optimization. All values also have a parallel called
 * directly on `Path`
 *
 * If you don't care about optimizations, see [SimplePathPointInfo]
 * @see CurvePointInfo
 */
interface PathPointInfo : CurvePointInfo {
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

/** The pose at this point along the path, composed of a combination of position and heading */
val PathPointInfo.pose: Pose2d get() = Pose2d(position, heading)
/** The [pose]'s derivative at this point along the path, composed of a combination of position and heading */
val PathPointInfo.poseDeriv: Pose2d get() = Pose2d(positionDeriv, headingDeriv)
/** The [pose]'s second derivative at this point along the path, composed of a combination of position and heading */
val PathPointInfo.poseSecondDeriv: Pose2d get() = Pose2d(positionDeriv, headingDeriv)

/**
 * A simple, no optimizations implementation of [PathPointInfo] that simply delegates to [PathSegment] given by [path],
 * given a point [s] units along the curve
 * @see CurvePointInfo
 * @see SimpleCurvePointInfo
 */
class SimplePathPointInfo(private val path: PathSegment, private val s: Double) : PathPointInfo {
    override val position: Vector2d get() = path.position(s)
    override val positionDeriv: Vector2d get() = path.positionDeriv(s)
    override val positionSecondDeriv: Vector2d get() = path.positionSecondDeriv(s)
    override val tanAngle: Double get() = path.tanAngle(s)
    override val tanAngleDeriv: Double get() = path.tanAngleDeriv(s)
    override val tanAngleSecondDeriv: Double get() = path.tanAngleSecondDeriv(s)
    override val heading: Double get() = path.heading(s)
    override val headingDeriv: Double get() = path.headingDeriv(s)
    override val headingSecondDeriv: Double get() = path.headingSecondDeriv(s)
}