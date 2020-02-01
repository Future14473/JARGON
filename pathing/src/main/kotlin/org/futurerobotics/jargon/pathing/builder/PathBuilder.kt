package org.futurerobotics.jargon.pathing.builder

import org.futurerobotics.jargon.pathing.*
import org.futurerobotics.jargon.util.builder

/**
 * Utility for making paths, with easy spline and line creation.
 */
class PathBuilder(
    start: Waypoint,
    var defaultHeadingInterpolator: HeadingInterpolator = LinearOffsetInterpolator
) {

    private val _edges = mutableListOf<Edge>(StartEdge(start))
    private val pastWaypoint get() = _edges.last().endWaypoint

    /**
     * Continues the path by using a spline to the given [waypoint].
     *
     * If a [headingInterpolator] is not supplied, the [defaultHeadingInterpolator] will be used.
     */
    @JvmOverloads
    fun splineTo(
        waypoint: Waypoint,
        headingInterpolator: HeadingInterpolator = defaultHeadingInterpolator
    ) = builder {
        _edges += SplineEdge(pastWaypoint, waypoint, headingInterpolator)
    }

    /**
     * Continues the path by using a line to the given [waypoint]
     *
     * If a [headingInterpolator] is not supplied, the [defaultHeadingInterpolator] will be used.
     */
    fun lineTo(
        waypoint: Waypoint,
        headingInterpolator: HeadingInterpolator = defaultHeadingInterpolator
    ) = addCurve(Line(pastWaypoint.position, waypoint.position), headingInterpolator)

    /**
     * Continues the path by using a given [curve].
     *
     * If a [headingInterpolator] is not supplied, the [defaultHeadingInterpolator] will be used.
     */
    @JvmOverloads
    fun addCurve(
        curve: Curve,
        headingInterpolator: HeadingInterpolator = defaultHeadingInterpolator
    ) = builder {
        checkStartPosition(curve)
        _edges += CurveEdge(curve, headingInterpolator)
    }

    /**
     * Continues the path by using a given [path].
     */
    fun addPath(path: Path) = builder {
        checkStartPosition(path)
        _edges += PathEdge(path)
    }

    private fun checkStartPosition(path: GenericPath<*>) {
        val startPos = path.startPoint().position
        require(startPos epsEq pastWaypoint.position) {
            "Previous end position ${pastWaypoint.position} must match " +
                "new curve's start point $startPos"
        }
    }

    fun build(curveGenParams: CurveGenParams): Path {
        if (_edges.size == 1) {
            val pastWaypoint = pastWaypoint
            return SinglePointPath(
                pastWaypoint.position,
                pastWaypoint.heading ?: pastWaypoint.direction ?: 0.0
            )
        }

        val paths = ArrayList<Path>()

        val sweeper = Sweeper(_edges)
        lateinit var previousEndWaypoint: Waypoint
        while (sweeper.hasNext()) {
            val curEdge = sweeper.next()
            when (curEdge) {
                is SplineEdge -> {
                    @Suppress("UNCHECKED_CAST")
                    val splineEdges = sweeper.takeWhile { it is SplineEdge } as List<SplineEdge>

                    val nextEdge = sweeper.peek()

                    val firstWaypoint = splineEdges.first().startWaypoint + previousEndWaypoint
                    val rawWaypoints = splineEdges.mapTo(mutableListOf(firstWaypoint)) { it.endWaypoint }
                    if (nextEdge != null) {
                        val lastIndex = rawWaypoints.size - 1
                        rawWaypoints[lastIndex] = rawWaypoints[lastIndex] + nextEdge.startWaypoint
                    }

                    //decomposed, so that we can get the resolved waypoints (naming to be finalized)
                    //todo: add public function to facilitate this
                    val waypoints = resolveWaypoints(rawWaypoints, curveGenParams)
                    val curves = waypoints.resolvedWaypointsToCurves(curveGenParams)

                    curves.mapIndexedTo(paths) { i, curve ->
                        val interpolator = splineEdges[i].headingInterpolator
                        val curWaypoint = waypoints[i]
                        val nextWaypoint = waypoints[i + 1]
                        interpolator.addHeadingTo(curve, curWaypoint.heading!!, nextWaypoint.heading!!)
                    }
                }
                is CurveEdge -> {
                    val startWaypoint = curEdge.startWaypoint + previousEndWaypoint
                    val nextEdge = sweeper.peek()
                    val endWaypoint = curEdge.endWaypoint.let {
                        if (nextEdge != null) it + nextEdge.startWaypoint
                        else it
                    }
                    val startHeading = startWaypoint.aHeading()
                    val endHeading = endWaypoint.aHeading()
                    paths += curEdge.curve.addHeading(curEdge.headingInterpolator, startHeading, endHeading)
                }
                is PathEdge -> {
                    paths += curEdge.path
                }
                is StartEdge -> {
                }
            }
            previousEndWaypoint = curEdge.endWaypoint
        }
        return multiplePath(paths)
    }

    private fun Waypoint.aHeading() = heading ?: direction!!
}

