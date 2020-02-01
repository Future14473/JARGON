@file:JvmName("CurveGen")

package org.futurerobotics.jargon.pathing.builder

import org.futurerobotics.jargon.math.Vector2d
import org.futurerobotics.jargon.math.angleNorm
import org.futurerobotics.jargon.math.times
import org.futurerobotics.jargon.pathing.*
import kotlin.math.min

/**
 * Parameters given to [heuristicSplineCurves] for curve generation.
 */
data class CurveGenParams @JvmOverloads constructor(
    /**
     * The factor by which auto-generated derivative magnitude is multiplied by. Values larger than 1
     * produce more rounded curves around waypoints, while smaller makes sharper turns. A value of 0.0
     * makes a straight line.
     */
    val derivMagnitudeMultiplier: Double = 1.0,
    /**
     * The reparameterizer used to convert splines into curves.
     */
    val reparameterizer: Reparameterizer = Reparameterizer.DEFAULT
) {

    init {
        require(derivMagnitudeMultiplier >= 0) { "derivMagnitudeMutiplier ($derivMagnitudeMultiplier) must be >= 0" }
    }

    companion object {
        /** Default [CurveGenParams]. */
        @JvmField
        val DEFAULT: CurveGenParams =
            CurveGenParams()
    }
}

/**
 * Creates a list of connected and heuristically optimized smooth splines that goes through all the given [waypoints].
 *
 * [curveGenParams] provides additional options.
 */
fun heuristicSplineCurves(waypoints: List<Waypoint>, curveGenParams: CurveGenParams): List<Curve> =
    resolveWaypoints(waypoints, curveGenParams).resolvedWaypointsToCurves(curveGenParams)

internal fun List<Waypoint>.resolvedWaypointsToCurves(curveGenParams: CurveGenParams): List<ReparamCurve> =
    asSequence()
        .map { it.toMotionState() }
        .zipWithNext { a, b -> QuinticSpline.fromDerivatives(a, b) }
        .mapTo(mutableListOf()) {
            it.reparameterizeToCurve(curveGenParams.reparameterizer)
        }

/**
 * Waypoints, connected by splines, and also with _no_ null elements. Heading will be filled with tan angle
 * if none.
 */
internal fun resolveWaypoints(
    waypoints: List<Waypoint>,
    curveGenParams: CurveGenParams
): List<Waypoint> {
    require(waypoints.size > 1) { "Num waypoints (${waypoints.size}) must be > 1" }
    val derivs = waypoints.mapIndexed { i, cur ->
        val (fromVec, toVec) = getVecs(waypoints, i, cur)
        val fromDirection = fromVec?.angle
        val toDirection = toVec?.angle

        val direction = cur.direction
            ?: if (fromDirection != null && toDirection != null) {
                fromDirection + 0.5 * angleNorm(toDirection - fromDirection)
            } else {
                fromDirection ?: toDirection!!
            }
        val magnitude = cur.derivMagnitude
            ?: curveGenParams.derivMagnitudeMultiplier * 0.5 * if (fromVec != null && toVec != null) {
                min(fromVec.length, toVec.length)
            } else {
                fromVec?.length ?: toVec!!.length
            }

        Vector2d.polar(magnitude, direction)
    }
    val secondDerivs = waypoints.mapIndexed { i, cur ->
        cur.secondDeriv?.let { return@mapIndexed it }
        val (fromVec, toVec) = getVecs(waypoints, i, cur)

        val prevDeriv = derivs.getOrNull(i - 1)
        val curDeriv = derivs[i]
        val nextDeriv = derivs.getOrNull(i + 1)

        val prevSplineSecondDeriv = if (prevDeriv == null) null else {
            -6 * fromVec!! + 2 * prevDeriv + 4 * curDeriv
        }
        val nextSplineSecondDeriv = if (nextDeriv == null) null else {
            6 * toVec!! - 2 * nextDeriv - 4 * curDeriv
        }
        if (fromVec != null && toVec != null) {
            val sums = fromVec.length + toVec.length
            val a = toVec.length / sums
            val b = fromVec.length / sums
            a * prevSplineSecondDeriv!! + b * nextSplineSecondDeriv!!
        } else prevSplineSecondDeriv ?: nextSplineSecondDeriv!!
    }
    return waypoints.mapIndexed { index, c ->
        val deriv = derivs[index]
        val secondDeriv = secondDerivs[index]
        val direction = deriv.angle
        Waypoint(
            position = c.position,
            direction = direction,
            derivMagnitude = deriv.length,
            secondDeriv = secondDeriv,
            heading = c.heading ?: direction
        )
    }
}

private fun getVecs(
    points: List<Waypoint>, i: Int, cur: Waypoint
): Pair<Vector2d?, Vector2d?> {
    val prev = points.getOrNull(i - 1)
    val next = points.getOrNull(i + 1)

    val fromVec = if (prev == null) null else cur.position - prev.position
    val toVec = if (next == null) null else next.position - cur.position
    return Pair(fromVec, toVec)
}
