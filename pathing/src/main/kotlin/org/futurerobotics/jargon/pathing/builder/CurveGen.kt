@file:JvmName("CurveGen")

package org.futurerobotics.jargon.pathing.builder

import org.futurerobotics.jargon.math.MotionState
import org.futurerobotics.jargon.math.Vector2d
import org.futurerobotics.jargon.math.angleNorm
import org.futurerobotics.jargon.math.times
import org.futurerobotics.jargon.pathing.*
import kotlin.math.min

/**
 * Parameters given to [connectWaypoints] for curve generation.
 */
data class CurveGenParams(
    /**
     * The reparameterizer used to convert splines into curves.
     */
    var reparameterizer: Reparameterizer = Reparameterizer.DEFAULT
) {

    companion object {

        /** Default [CurveGenParams]. */
        @JvmField
        val DEFAULT: CurveGenParams = CurveGenParams()
    }
}

/**
 * Creates a list of spline based curves that goes through all the given [waypoints] in
 * order. It uses some heuristics for smoothness/optimality.
 *
 * You can then pass the result to [joinCurves] to get one curve.
 *
 * [curveGenParams] provides some options.
 */
@JvmOverloads
fun connectWaypoints(
    waypoints: List<CurveWaypoint>,
    curveGenParams: CurveGenParams = CurveGenParams.DEFAULT
): List<Curve> =
    doConnectWaypoints(waypoints, curveGenParams)

private fun doConnectWaypoints(
    waypoints: List<CurveWaypoint>,
    curveGenParams: CurveGenParams
): List<ReparamCurve> {
    require(waypoints.size > 1) { "Num waypoints (${waypoints.size}) must be > 1" }
    val derivs = waypoints.mapIndexed { i, curWaypoint ->
        val (fromVec, toVec) = getVecs(waypoints, i, curWaypoint)
        val fromDirection = fromVec?.angle
        val toDirection = toVec?.angle

        val direction = curWaypoint.direction
            ?: if (fromDirection != null && toDirection != null) {
                fromDirection + 0.5 * angleNorm(toDirection - fromDirection)
            } else {
                fromDirection ?: toDirection!!
            }
        val magnitude = if (fromVec != null && toVec != null) {
            min(fromVec.length, toVec.length)
        } else {
            fromVec?.length ?: toVec!!.length
        } * 0.5 * curWaypoint.roundness

        Vector2d.polar(magnitude, direction)
    }
    val secondDerivs = waypoints.mapIndexed { i, cur ->
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
    return waypoints.asSequence().mapIndexed { index, c ->
        MotionState(c.position, derivs[index], secondDerivs[index])
    }.zipWithNext { a, b -> QuinticSpline.fromDerivatives(a, b) }
        .map { it.toCurve(curveGenParams.reparameterizer) }
        .toList()
}

private fun getVecs(
    points: List<CurveWaypoint>, i: Int, cur: CurveWaypoint
): Pair<Vector2d?, Vector2d?> {
    val prev = points.getOrNull(i - 1)
    val next = points.getOrNull(i + 1)

    val fromVec = if (prev == null) null else cur.position - prev.position
    val toVec = if (next == null) null else next.position - cur.position
    return Pair(fromVec, toVec)
}
