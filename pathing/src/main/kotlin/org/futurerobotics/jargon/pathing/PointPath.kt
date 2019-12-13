package org.futurerobotics.jargon.pathing

import org.futurerobotics.jargon.math.Pose2d
import org.futurerobotics.jargon.math.Vector2d
import org.futurerobotics.jargon.util.uncheckedCast

/**
 * A point for a [PointCurve]/[PointPath] at the given [pose].
 *
 * @see [PointCurve]
 * @see [PointPath]
 */
class SinglePathPoint(override val pose: Pose2d) : PathPoint {

    override val length: Double get() = 0.0
    override val position: Vector2d get() = pose.vec
    override val positionDeriv: Vector2d get() = Vector2d.ZERO
    override val positionSecondDeriv: Vector2d get() = Vector2d.ZERO
    override val tanAngle: Double get() = pose.heading
    override val tanAngleDeriv: Double get() = 0.0
    override val tanAngleSecondDeriv: Double get() = 0.0
    override val heading: Double get() = pose.heading
    override val headingDeriv: Double get() = 0.0
    override val headingSecondDeriv: Double get() = 0.0
    override val poseDeriv: Pose2d get() = Pose2d.ZERO
    override val poseSecondDeriv: Pose2d get() = Pose2d.ZERO
}

/**
 * Common superclass of both [PointCurve] and [PointPath]
 */
sealed class PointGenericPath<Path : GenericPath<Point>, Point : CurvePoint>(pose: Pose2d) :
    GenericPath<Point> {

    override val length: Double get() = 0.0
    private val point = SinglePathPoint(pose)
    override fun pointAt(s: Double): Point = point.uncheckedCast()
}

/**
 * A curve of length 0 that consists of a single point. If a pose is supplied for constructor, the heading
 * is used for tangent angle.
 */
class PointCurve(pose: Pose2d) : PointGenericPath<Curve, CurvePoint>(pose), Curve {

    @JvmOverloads
    constructor(position: Vector2d, tanAngle: Double = 0.0) : this(Pose2d(position, tanAngle))
}

/**
 * A path of length 0 that consists of a single pose.
 */
class PointPath(pose: Pose2d) : PointGenericPath<Path, PathPoint>(pose), Path {

    @JvmOverloads
    constructor(position: Vector2d, heading: Double = 0.0) : this(Pose2d(position, heading))
}
