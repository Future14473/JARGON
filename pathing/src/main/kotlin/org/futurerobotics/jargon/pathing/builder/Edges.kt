package org.futurerobotics.jargon.pathing.builder

import org.futurerobotics.jargon.pathing.Curve
import org.futurerobotics.jargon.pathing.Path

/**
 * Represents a connection from two waypoints
 */
internal sealed class Edge {

    abstract val startWaypoint: Waypoint
    abstract val endWaypoint: Waypoint
}

internal class StartEdge(
    override val endWaypoint: Waypoint
) : Edge() {

    override val startWaypoint: Nothing get() = error("")
}

/**
 * An edge that uses a spline connection
 *
 * This gets special treatment
 */
internal class SplineEdge(
    override val startWaypoint: Waypoint,
    override val endWaypoint: Waypoint,
    val headingInterpolator: HeadingInterpolator
) : Edge()

/** An edge that uses a curve */
internal class CurveEdge(
    val curve: Curve,
    val headingInterpolator: HeadingInterpolator
) : Edge() {

    override val startWaypoint: Waypoint = Waypoint.fromCurvePoint(curve.startPoint())
    override val endWaypoint: Waypoint = Waypoint.fromCurvePoint(curve.endPoint())
}

/** An edge that uses a path. */
internal class PathEdge(
    val path: Path
) : Edge() {

    override val startWaypoint: Waypoint = Waypoint.fromPathPoint(path.startPoint())
    override val endWaypoint: Waypoint = Waypoint.fromPathPoint(path.endPoint())
}
