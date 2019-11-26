package org.futurerobotics.jargon.pathing.builder

import org.futurerobotics.jargon.math.Pose2d
import org.futurerobotics.jargon.math.Vector2d
import org.futurerobotics.jargon.math.angleNorm
import org.futurerobotics.jargon.pathing.*
import org.futurerobotics.jargon.util.builder

/**
 * Common superclass of both [CurveBuilder] and [PathBuilder].
 *
 * @property curveGenParams the curve gen params used for curve generation.
 */
sealed class GenericPathBuilder<Self : GenericPathBuilder<Self>>(private val curveGenParams: CurveGenParams) {

    /**
     * The waypoints in building a spline-heuristic based curve.
     *
     * Must always contain at least 1 waypoint.
     */
    protected val waypoints: ArrayList<Waypoint> = ArrayList()

    /**
     * The number of curves, including given by [waypoints].
     */
    protected val numCurves: Int get() = curves.size + (waypoints.size - 1).coerceAtLeast(0)
    /**
     * The previous curves built; when a not-waypoint is added.
     */
    protected val curves: ArrayList<Curve> = ArrayList()

    /**
     * Appends a new [waypoint].
     */
    infix fun toPoint(waypoint: Waypoint): Self = builder {
        waypoints += waypoint
    }

    /**
     * Appends a new waypoint at the given [point].
     */
    infix fun toPoint(point: Vector2d): Self = builder {
        toPoint(Waypoint(point))
    }

    /**
     * Appends a new waypoint that goes through the given [pose]. The path will face the same direction as the pose's
     * heading.
     */
    infix fun toPoint(pose: Pose2d): Self = builder {
        toPoint(Waypoint(pose.vec, pose.heading))
    }

    /**
     * Appends all of the following [waypoints] to this curve.
     */
    infix fun toPoints(waypoints: List<Waypoint>): Self = builder {
        this.waypoints += waypoints
    }

    /**
     * Appends a new [Waypoint] to this curve with the given params.
     */
    @JvmOverloads
    fun toPoint(
        x: Double,
        y: Double,
        direction: Double? = null,
        derivMagnitude: Double? = null,
        secondDeriv: Vector2d? = null
    ): Self = builder {
        toPoint(Waypoint(Vector2d(x, y), direction, derivMagnitude, secondDeriv))
    }

    /**
     * Adds a curve to the end of this builder.
     */
    infix fun addCurve(curve: Curve): Self = builder {
        mergeWaypointFromPoint(curve.startPoint())
        waypointsToCurves() //resolve curves now.
        curves += curve
        waypoints[0] = Waypoint.fromPoint(curve.endPoint())
    }

    /**
     * Merges the last waypoint with info from [point].
     */
    protected fun mergeWaypointFromPoint(point: CurvePoint) {
        waypoints.lastOrNull()?.let {
            waypoints[waypoints.lastIndex] = it.mergeFrom(Waypoint.fromPoint(point))
        } ?: error("No waypoint given.")
    }

    /**
     * Converts a series of waypoints into curves, adds them to [curves], then clears all but the last waypoint.
     * Ignores if less than 2 waypoints.
     */
    protected fun waypointsToCurves() {
        //if only 0 or 1 ignore.
        check(waypoints.isNotEmpty()) { "No waypoints given." }
        if (waypoints.size < 2) return
        curves += heuristicCurves(waypoints, curveGenParams)
        val lastWaypoint = waypoints.last()
        waypoints.clear()
        waypoints += lastWaypoint
    }

    internal fun getPoints(): List<Vector2d> = waypoints.map { it.location }
}

/**
 * Builder for creating a spline-based curve using points or [Waypoint]s.
 * @see CurveBuilder
 */
class CurveBuilder
@JvmOverloads constructor(curveGenParams: CurveGenParams = CurveGenParams()) :
    GenericPathBuilder<CurveBuilder>(curveGenParams) {

    /**
     * Builds the [Curve].
     *
     * @see heuristicCurves
     */
    fun build(): Curve = multipleCurve(buildCurves())

    /**
     * Builds the curve as a list of connected [Curve] (segments).
     *
     * @see heuristicCurves
     */
    fun buildCurves(): List<Curve> {
        waypointsToCurves()
        return curves.toList().also { curves.clear() }
    }
}

/**
 * Builder for creating a spline-based path using points or [Waypoint]s.
 *
 * After any [toPoints] operation, a [heading] must be provided. This can be a [HeadingProvider] or a
 * [ContinuationHeadingProvider].
 *
 * @see CurveBuilder
 */
class PathBuilder(curveGenParams: CurveGenParams = CurveGenParams()) : GenericPathBuilder<PathBuilder>(curveGenParams) {

    //corresponds to curves.
    private val headingProviders = arrayListOf<ContinuationHeadingProvider>()
    private val paths = arrayListOf<Path>()

    private var lastPoint: PathPoint? = null

    /**
     * Sets the initial heading; can only be called after setting the first waypoint.
     */
    fun initialHeading(heading: Double): PathBuilder = builder {
        check(lastPoint == null && waypoints.size == 1) { "Can only set initial heading after first point" }
        val waypoint = waypoints.first()
        lastPoint = object : PathPoint {
            @Suppress("ObjectPropertyName")
            private var _point: CurvePoint? = null
            private val point
                get() = _point ?: curves.firstOrNull()?.startPoint()?.also {
                    _point = it
                }
            override val length: Double get() = point?.length ?: 1.0
            override val position: Vector2d = waypoint.location
            override val positionDeriv: Vector2d
                get() = point?.positionDeriv ?: waypoint.direction?.let {
                    Vector2d.polar(1.0, it)
                } ?: Vector2d.ZERO
            override val positionSecondDeriv: Vector2d get() = point?.positionSecondDeriv ?: Vector2d.ZERO
            override val tanAngle: Double get() = point?.tanAngle ?: waypoint.direction ?: heading
            override val tanAngleDeriv: Double get() = point?.tanAngleDeriv ?: 0.0
            override val tanAngleSecondDeriv: Double get() = point?.tanAngleSecondDeriv ?: 0.0
            override val heading: Double get() = heading
            override val headingDeriv: Double get() = 0.0
            override val headingSecondDeriv: Double get() = 0.0
        }
    }

    /**
     * Sets the default heading provider, the heading filled in when no heading is supplied.
     *
     * A value of `null` will throw instead.
     */
    var defaultHeadingProvider: ContinuationHeadingProvider? = null

    /**
     * Sets the default heading provider, the heading filled in when no heading is supplied.
     *
     * A value of `null` will throw instead.
     */
    fun setDefaultHeadingProvider(value: ContinuationHeadingProvider?): PathBuilder = builder {
        defaultHeadingProvider = value
    }

    /**
     * Sets the default heading provider, the heading filled in when no heading is supplied.
     *
     * A value of `null` will throw instead.
     */
    fun setDefaultHeadingProvider(value: HeadingProvider?): PathBuilder = builder {
        setDefaultHeadingProvider(value?.toContinuationHeadingProvider())
    }

    private fun HeadingProvider.toContinuationHeadingProvider(): ContinuationHeadingProvider =
        object : ContinuationHeadingProvider {
            override fun getHeadingProvider(lastPoint: PathPoint, curve: Curve): HeadingProvider =
                this@toContinuationHeadingProvider
        }

    private fun fillInHeadingProviders() {
        val numCurves = numCurves
        if (numCurves == headingProviders.size) return
        val defaultProvider = defaultHeadingProvider
            ?: error("All curves must have headings. Either specify headings or add a default heading provider.")
        headingProviders.ensureCapacity(numCurves)
        repeat(numCurves - headingProviders.size) {
            headingProviders += defaultProvider
        }
    }

    /** Adds a [ContinuationHeadingProvider] to provide heading to the previous curve operation. */
    fun heading(heading: ContinuationHeadingProvider): PathBuilder = builder {
        check(numCurves != 0) { "No curve to add heading to" }
        val numCurves = numCurves
        when (headingProviders.size) {
            numCurves -> headingProviders[headingProviders.lastIndex] = heading
            numCurves - 1 -> headingProviders += heading
            else -> {
                fillInHeadingProviders()
                headingProviders[headingProviders.lastIndex] = heading
            }
        }
    }

    /** Adds a [HeadingProvider] to provide heading to the previous curve operation. */
    fun heading(heading: HeadingProvider): PathBuilder = builder {
        heading(heading.toContinuationHeadingProvider())
    }

    /** Adds a path to this builder. */
    fun addPath(path: Path): PathBuilder = builder {
        /** see [addCurve] */
        mergeWaypointFromPoint(path.startPoint())
        allToPaths() //resolve paths now.
        paths += path
        waypoints[0] = Waypoint.fromPoint(path.endPoint())
    }

    /** Adds a [PointTurn] that rotates to the given [toHeading]. */
    fun pointTurn(toHeading: Double): PathBuilder = builder {
        allToPaths()
        val point = paths.lastOrNull()?.endPoint() ?: error(
            "Previous heading must be known for point turn."
        )
        addPath(PointTurn(point.position, point.heading, angleNorm(toHeading - point.heading)))
    }

    private fun allToPaths() {
        waypointsToCurves()
        fillInHeadingProviders()
        var prevPath: Path? = paths.lastOrNull()
        paths += curves.zip(headingProviders) { curve, providerProvider ->
            if (prevPath != null)
                lastPoint = prevPath!!.endPoint()
            val heading = providerProvider.getHeadingProvider(lastPoint!!, curve)
            curve.addHeading(heading).also { prevPath = it }
        }
        curves.clear()
        headingProviders.clear()
    }

    /**
     * Builds the [Path].
     */
    fun build(): Path {
        allToPaths()
        return multiplePath(paths).also { paths.clear() }
    }

    /**
     * Builds the path as a list of connected [Path] (segments).
     */
    fun buildPaths(): List<Path> {
        allToPaths()
        return paths.toList().also { paths.clear() }
    }
}

