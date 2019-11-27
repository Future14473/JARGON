package org.futurerobotics.jargon.pathing.graph

import org.futurerobotics.jargon.math.MotionState
import org.futurerobotics.jargon.math.ValueMotionState
import org.futurerobotics.jargon.math.Vector2d
import org.futurerobotics.jargon.math.angleNorm
import org.futurerobotics.jargon.pathing.ComponentPath
import org.futurerobotics.jargon.pathing.CurvePoint
import org.futurerobotics.jargon.pathing.PathPoint
import org.futurerobotics.jargon.pathing.reparam.ReparamCurve
import kotlin.math.PI

/**
 * Represents the constraints at a given waypoint for curve generation, where there may be known or not known
 * [direction], [heading], [derivMagnitude], and [secondDeriv].
 */
data class WaypointConstraint
@JvmOverloads constructor(
    /** The optionally known direction (angle) at this point on the curve. Equal to the angle of the deriv. */
    val direction: Double? = null,
    /** The optionally known heading at this point. If not known, tangent angle (direction) will be used. */
    val heading: Double? = null,
    /** The optionally known magnitude of the derivative. A larger value makes a less sharp turn around this waypoint. */
    val derivMagnitude: Double? = null,
    /** The possibly not yet known position second deriv. You probably won't be changing this manually. */
    val secondDeriv: Vector2d? = null
) {

    /**
     * Returns a [WaypointConstraint] but is reversed in [direction] only.
     */
    fun reverseDirection(): WaypointConstraint = WaypointConstraint(
        this@WaypointConstraint.direction?.let { angleNorm(it + PI) },
        heading, derivMagnitude, secondDeriv

    )

    /**
     * Returns a new [WaypointConstraint] with unknown values possibly filled from the [other] constraint.
     */
    fun mergeFrom(other: WaypointConstraint): WaypointConstraint = WaypointConstraint(
        direction ?: other.direction,
        heading ?: other.heading,
        derivMagnitude ?: other.derivMagnitude,
        secondDeriv ?: other.secondDeriv
    )

    companion object {

        /** A [WaypointConstraint] defining NO constraints. */
        @JvmField
        val NONE: WaypointConstraint = WaypointConstraint(null, null, null, null)
    }
}

/**
 * Represents a waypoint along a path, with a known [position], but also with [WaypointConstraint]s.
 * @see heuristicSplineCurves
 */
data class Waypoint
constructor(
    /** The known position. */
    val position: Vector2d,
    /** The [WaypointConstraint] at this waypoint. */
    val constraint: WaypointConstraint
) {

    @JvmOverloads
    constructor(
        position: Vector2d,
        direction: Double? = null,
        heading: Double? = null,
        derivMagnitude: Double? = null,
        secondDeriv: Vector2d? = null
    ) : this(
        position, WaypointConstraint(
            direction = direction,
            heading = heading,
            derivMagnitude = derivMagnitude,
            secondDeriv = secondDeriv
        )
    )

    @JvmOverloads
    constructor(
        x: Double, y: Double,
        direction: Double? = null,
        heading: Double? = null,
        derivMagnitude: Double? = null,
        secondDeriv: Vector2d? = null
    ) : this(
        Vector2d(x, y), WaypointConstraint(
            direction = direction,
            heading = heading,
            derivMagnitude = derivMagnitude,
            secondDeriv = secondDeriv
        )
    )

    /** Returns a new waypoint with unknown constraints possibly filled in from the given [other] constraint. */
    fun mergeFrom(other: WaypointConstraint): Waypoint = Waypoint(position, constraint.mergeFrom(other))

    /**
     * Converts this waypoint to a [MotionState] of [Vector2d]. If any component is null, may throw
     * null pointer exception.
     */
    fun toMotionState(): MotionState<Vector2d> = constraint.run {
        ValueMotionState(position, Vector2d.polar(derivMagnitude!!, direction!!), secondDeriv!!)
    }

    companion object {

        /**
         * Creates a waypoint from a Vector2d MotionState, and possibly heading.
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
         * Creates a waypoint from a curve [point]: position, direction are filled,
         *
         * heading is filled if known, and if the point is from a [ReparamCurve], the underlying function's derivatives
         * are used.
         */
        @JvmStatic
        fun fromCurvePoint(point: CurvePoint): Waypoint {
            var curvePoint = point
            val heading = (curvePoint as? PathPoint)?.heading
            if (curvePoint is ComponentPath.Point)
                curvePoint = curvePoint.curvePoint
            if (curvePoint is ReparamCurve.Point)
                return fromMotionState(curvePoint.motionState(), heading)
            return Waypoint(
                position = curvePoint.position,
                direction = curvePoint.tanAngle,
                heading = heading
            )
        }
    }
}

