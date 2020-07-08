package org.futurerobotics.jargon.pathing.builder

import org.futurerobotics.jargon.math.Vector2d
import org.futurerobotics.jargon.pathing.Curve
import org.futurerobotics.jargon.pathing.CurvePoint
import org.futurerobotics.jargon.pathing.Path

/**
 * A waypoint used to form spline-based curves. Contains [position] and other optional values.
 */
data class CurveWaypoint(
    /** The position of this waypoint. */
    val position: Vector2d,
    /** The optionally specified target direction the curve should go at this point. */
    val direction: Double? = null,
    /**
     * The "roundness" of the curve at this point. Higher values result in less sharp turns. A value of 0.0
     * makes straight lines.
     *
     * Mathematically, this is a multiplier for position derivative used to create the splines.
     */
    val roundness: Double = 1.0
)

internal sealed class Edge(var toWaypoint: CurveWaypoint)

internal class StartEdge(endWaypoint: CurveWaypoint) : Edge(endWaypoint)

/**
 * This gets special treatment
 */
internal class SplineEdge(endWaypoint: CurveWaypoint) : Edge(endWaypoint)

internal class CurveEdge(val curve: Curve) : Edge(fromPoint(curve.endPoint()))

internal class PathEdge(val path: Path) : Edge(fromPoint(path.endPoint()))

internal fun fromPoint(point: CurvePoint): CurveWaypoint =
    CurveWaypoint(
        position = point.position,
        direction = point.tanAngle
    )

