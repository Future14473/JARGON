package org.futurerobotics.jargon.pathing.builder

import org.futurerobotics.jargon.math.Vector2d
import org.futurerobotics.jargon.pathing.*
import org.futurerobotics.jargon.util.builder

/**
 * Common parts of [CurveBuilder] and [PathBuilder]
 */
abstract class AnyPathBuilder<Self : AnyPathBuilder<Self>>
constructor(start: Vector2d) {

    private val edges = mutableListOf<Edge>(StartEdge(CurveWaypoint(start)))
    private val lastWaypoint get() = edges.last().toWaypoint

    private fun splineTo(waypoint: CurveWaypoint): Self = builder { edges += SplineEdge(waypoint) }

    /**
     * Continues the path with a spline to the given [location].
     */
    fun splineTo(location: Vector2d): Self = splineTo(CurveWaypoint(location))

    /**
     * Continues the path with a spline to the given location.
     */
    fun splineTo(x: Double, y: Double): Self = splineTo(Vector2d(x, y))

//    private fun lineTo(waypoint: Waypoint): Self = builder {
//        addCurve(Line(lastWaypoint.position, waypoint.position))
//        constrainLastWaypoint(waypoint)
//    }

    /**
     * Continues the path by using a spline to the given [location].
     */
    fun lineTo(location: Vector2d): Self = builder {
        addCurve(Line(lastWaypoint.position, location))
    }

    /**
     * Continues the path with a line to the given location.
     */
    fun lineTo(x: Double, y: Double): Self = lineTo(Vector2d(x, y))

    /**
     * Continues the path by using any given [curve]. The start of the curve must meet with the previous point.
     */
    fun addCurve(curve: Curve): Self = builder {
        checkMatchingStart(curve)
        changeLastPoint { copy(direction = direction ?: curve.startPoint().tanAngle) }
        edges += CurveEdge(curve)
    }

    /**
     * Sets a [direction][CurveWaypoint.direction] at the last waypoint. This has no effect if neither adjacent
     * connections are splines.
     */
    fun direction(direction: Double): Self = builder {
        changeLastPoint { copy(direction = direction) }
    }

    /**
     * Sets a [roundness][CurveWaypoint.roundness] at the last waypoint. This has no effect if neither adjacent
     * connections are splines.
     */
    fun roundness(roundness: Double): Self = builder {
        changeLastPoint { copy(roundness = roundness) }
    }

    private fun checkMatchingStart(path: AnyPath<*>) {
        val startPos = path.startPoint().position
        require(startPos epsEq lastWaypoint.position) {
            "Previous end location ${lastWaypoint.position} must match " +
                "new curve's start point $startPos"
        }
    }

    private inline fun changeLastPoint(newWaypoint: CurveWaypoint.() -> CurveWaypoint) {
        val lastEdge = edges.last()
        lastEdge.toWaypoint = lastEdge.toWaypoint.newWaypoint()
    }

    @OptIn(ExperimentalStdlibApi::class)
    protected fun buildCurves(curveGenParams: CurveGenParams = CurveGenParams.DEFAULT): List<Curve> {
        check(edges.size > 1) { "Only 1 point given; there is no path" }

        val curves = ArrayList<Curve>()

        val iterator = edges.listIterator()

        var lastPoint: CurveWaypoint = (iterator.next() as StartEdge).toWaypoint

        while (iterator.hasNext()) {
            val edge = iterator.next()
            when (edge) {
                is SplineEdge -> {
                    val splineWaypoints = buildList {
                        //first point
                        add(lastPoint)
                        //this edge point
                        add(edge.toWaypoint)
                        while (iterator.hasNext()) {
                            val next = iterator.next()
                            if (next is SplineEdge) add(next.toWaypoint)
                            else iterator.previous()
                        }
                    }
                    curves += connectWaypoints(splineWaypoints, curveGenParams)
                }
                is CurveEdge -> {
                    curves += edge.curve
                }
                else -> throw AssertionError()
            }
            lastPoint = edge.toWaypoint
        }
        return curves
    }
}

/**
 * A utility to build a multi-part curve.
 */
class CurveBuilder(start: Vector2d) : AnyPathBuilder<CurveBuilder>(start) {

    fun build(params: CurveGenParams, sharpTurnBehavior: DiscontinuityBehavior) =
        joinCurves(buildCurves(curveGenParams = params), sharpTurnBehavior)
}

/**
 * A utility to build a multi-part path.
 */
class PathBuilder(start: Vector2d) : AnyPathBuilder<CurveBuilder>(start) {

    private val interpolator = mutableListOf<HeadingInterpolator>()

    fun build(params: CurveGenParams) = joinCurves(buildCurves(curveGenParams = params))
}
