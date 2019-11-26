package org.futurerobotics.jargon.pathing.builder

import org.futurerobotics.jargon.math.MotionState
import org.futurerobotics.jargon.math.Vector2d
import org.futurerobotics.jargon.math.function.VectorFunction
import org.futurerobotics.jargon.pathing.CurvePoint
import org.futurerobotics.jargon.pathing.reparam.ReparamCurve

/**
 * Represents waypoints along which to generate a curve.
 *
 * Values are given in terms of a [VectorFunction] parameter `t`.
 *
 * Has known [location] and possibly not known [direction], [derivMagnitude], and [secondDeriv].
 *
 * Unknown values will be optimized heuristically to try and generate a smooth and efficient curve.
 * @see heuristicCurves
 */
data class Waypoint(
    /** The known position. */
    val location: Vector2d,
    /** The optionally known direction (angle) at this point on the curve. Equal to the angle of the deriv. */
    val direction: Double? = null,
    /**
     * The optionally known magnitude of the derivative. A larger value makes a less sharp turn around this waypoint.
     */
    val derivMagnitude: Double? = null,
    /** The possibly not yet known position deriv. */
    val secondDeriv: Vector2d? = null
) {

    companion object {

        /**
         * Creates a waypoint from a Vector2d MotionState, and possibly heading.
         */
        @JvmStatic
        fun fromMotionState(state: MotionState<Vector2d>): Waypoint = state.run {
            Waypoint(value, deriv.angle, deriv.length, secondDeriv)
        }

        /**
         * Creates a waypoint from a curve [point]: position and direction are filled.
         *
         * If the point is from a [ReparamCurve], the underlying function's derivatives are used instead.
         */
        @JvmStatic
        fun fromPoint(point: CurvePoint): Waypoint {
            if (point is ReparamCurve.Point) {
                return fromMotionState(point.motionState())
            }
            return Waypoint(point.position, point.tanAngle)
        }
    }

    /**
     * Fills in all `null` values of this waypoint with values from the other waypoint.
     * This also checks to make sure that the locations match.
     * (if other waypoint contains null values to, may still be null).
     */
    fun mergeFrom(other: Waypoint): Waypoint {
        require(location epsEq other.location) { "Position discontinuity in waypoints" }
        return Waypoint(
            location,
            direction ?: other.direction,
            derivMagnitude ?: other.derivMagnitude,
            secondDeriv ?: other.secondDeriv
        )
    }
}

