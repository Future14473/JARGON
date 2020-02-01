package org.futurerobotics.jargon.pathing.builder

import org.futurerobotics.jargon.math.MotionState
import org.futurerobotics.jargon.math.Pose2d
import org.futurerobotics.jargon.math.Vector2d
import org.futurerobotics.jargon.math.angleNorm
import org.futurerobotics.jargon.pathing.CurveHeadingPath
import org.futurerobotics.jargon.pathing.CurvePoint
import org.futurerobotics.jargon.pathing.PathPoint
import org.futurerobotics.jargon.pathing.ReparamCurve
import kotlin.math.PI

/**
 * Represents the constraints at a given waypoint for curve generation,
 * where there _may_ be constricted [heading], [direction], [derivMagnitude], and [secondDeriv].
 */
open class WaypointConstraint
@JvmOverloads constructor(
    /** The optionally restricted heading at this point. */
    val heading: Double? = null,
    /** The optionally restricted direction of motion at this waypoint (tangent angle). */
    val direction: Double? = null,
    /**
     * The optionally restricted magnitude of the derivative at this waypoint. A larger value makes a less sharp turn
     * around this waypoint.
     */
    val derivMagnitude: Double? = null,
    /** The optionally restricted second derivative. You probably won't be changing this manually. */
    val secondDeriv: Vector2d? = null
) {

    /**
     * Returns a [WaypointConstraint] but is reversed in [direction] only.
     */
    fun reverseDirection(): WaypointConstraint =
        WaypointConstraint(
            direction?.let { angleNorm(it + PI) },
            heading,
            derivMagnitude,
            secondDeriv
        )

    /**
     * Returns a new [WaypointConstraint] with unknown values filled from the [other] constraint.
     */
    open operator fun plus(other: WaypointConstraint): WaypointConstraint = WaypointConstraint(
        heading = heading ?: other.heading,
        direction = direction ?: other.direction,
        derivMagnitude = derivMagnitude ?: other.derivMagnitude,
        secondDeriv = secondDeriv ?: other.secondDeriv
    )
}

/**
 * Represents a waypoint along a path, with a known [position], and the same constraints in [WaypointConstraint].
 * @see heuristicSplineCurves
 */
class Waypoint
@JvmOverloads constructor(
    /** The position of this waypoint. */
    val position: Vector2d,
    heading: Double? = null,
    direction: Double? = null,
    derivMagnitude: Double? = null,
    secondDeriv: Vector2d? = null
) : WaypointConstraint(
    heading,
    direction,
    derivMagnitude,
    secondDeriv
) {

    constructor(
        position: Vector2d,
        waypointConstraint: WaypointConstraint
    ) : this(
        position,
        waypointConstraint.heading,
        waypointConstraint.direction,
        waypointConstraint.derivMagnitude,
        waypointConstraint.secondDeriv
    )

    @JvmOverloads
    constructor(
        pose: Pose2d,
        direction: Double? = null,
        derivMagnitude: Double? = null,
        secondDeriv: Vector2d? = null
    ) : this(
        pose.vector2d,
        heading = pose.heading,
        direction = direction,
        derivMagnitude = derivMagnitude,
        secondDeriv = secondDeriv
    )

    @JvmOverloads
    constructor(
        x: Double, y: Double,
        heading: Double? = null,
        direction: Double? = null,
        derivMagnitude: Double? = null,
        secondDeriv: Vector2d? = null
    ) : this(
        Vector2d(x, y), WaypointConstraint(
            heading = heading,
            direction = direction,
            derivMagnitude = derivMagnitude,
            secondDeriv = secondDeriv
        )
    )

    /**
     * Returns a new waypoint with unknown constraints merged from the [other] waypoint constraint.
     */
    override operator fun plus(other: WaypointConstraint): Waypoint = Waypoint(
        position, super.plus(other)
    )

    /**
     * Converts this waypoint to a [MotionState] of [Vector2d]. If any component is null, may throw
     * null pointer exception.
     */
    fun toMotionState(): MotionState<Vector2d> =
        MotionState(position, Vector2d.polar(derivMagnitude!!, direction!!), secondDeriv!!)

    companion object {
        /**
         * Creates a waypoint from a Vector2d [MotionState], and possibly heading.
         *
         * If heading is supplied, this is fully constrained.
         */
        @JvmStatic
        @JvmOverloads
        fun fromMotionState(state: MotionState<Vector2d>, heading: Double? = null): Waypoint = state.run {
            Waypoint(
                position = value,
                direction = deriv.angle,
                heading = heading,
                derivMagnitude = deriv.length,
                secondDeriv = secondDeriv
            )
        }

        /**
         * Creates a waypoint from a curve [curvePoint]: position and direction are filled.
         *
         * If the point is from a [ReparamCurve], the underlying function's derivatives are used.
         */
        @JvmStatic
        fun fromCurvePoint(curvePoint: CurvePoint): Waypoint =
            fromPoint(curvePoint, null)

        /**
         * Creates a waypoint from a curve [pathPoint]: position, direction, _and heading_ are filled.
         *
         * If the point is from a [ReparamCurve], the underlying function's derivatives are used.
         */
        @JvmStatic
        fun fromPathPoint(pathPoint: PathPoint): Waypoint {
            val curvePoint = (pathPoint as? CurveHeadingPath.Point)?.curvePoint ?: pathPoint
            return fromPoint(curvePoint, pathPoint.heading)
        }

        private fun fromPoint(point: CurvePoint, heading: Double? = null): Waypoint =
            when (point) {
                is ReparamCurve.Point ->
                    fromMotionState(point.motionState(), heading)
                else -> Waypoint(
                    position = point.position,
                    direction = point.tanAngle,
                    heading = heading
                )
            }
    }
}

