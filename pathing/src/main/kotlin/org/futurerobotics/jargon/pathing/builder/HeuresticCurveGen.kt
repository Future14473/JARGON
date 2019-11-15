package org.futurerobotics.jargon.pathing.builder

import org.futurerobotics.jargon.math.ValueMotionState
import org.futurerobotics.jargon.math.Vector2d
import org.futurerobotics.jargon.math.angleNorm
import org.futurerobotics.jargon.math.function.QuinticSpline
import org.futurerobotics.jargon.math.function.VectorFunction
import org.futurerobotics.jargon.math.times
import org.futurerobotics.jargon.pathing.Curve
import org.futurerobotics.jargon.pathing.reparam.reparamByIntegration
import kotlin.math.min

/**
 * Parameters given to [heruesticCurves] for curve generation.
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
    val reparameterizer: (VectorFunction) -> Curve = { it.reparamByIntegration() }
) {

    init {
        require(derivMagnitudeMultiplier >= 0) { "derivFactor ($derivMagnitudeMultiplier) must be >= 0" }
    }
}

/**
 * Creates a list of connected and heruestic optimized smooth splines that goes through all the given [waypoints],
 * using the given [curveGenParams] for additional options.
 *
 * This uses just the heruestics given by TODO
 */
fun heruesticCurves(waypoints: List<Waypoint>, curveGenParams: CurveGenParams): List<Curve> {
    return heruesticSplines(waypoints, curveGenParams)
        .map { curveGenParams.reparameterizer(it) }
}

/**
 * Creates a MultipleCurve that goes through all the given [waypoints],
 * using the given [curveGenParams].
 *
 * This is also used by [CurveBuilder].
 */
fun heruesticSplines(
    waypoints: List<Waypoint>,
    curveGenParams: CurveGenParams
): List<QuinticSpline> {
    require(waypoints.size > 1) { "Num waypoints (${waypoints.size}) must be > 1" }
    val points = waypoints.toList()

    val derivs = points.mapIndexed { i, cur ->
        val (fromVec, toVec) = getVecs(points, i, cur)
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
    val secondDerivs = points.mapIndexed { i, cur ->
        cur.secondDeriv?.let { return@mapIndexed it }
        val (fromVec, toVec) = getVecs(points, i, cur)

        val prevDeriv = derivs.tryGet(i - 1)
        val curDeriv = derivs[i]
        val nextDeriv = derivs.tryGet(i + 1)

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
    return List(points.size) {
        val loc = points[it].location
        val deriv = derivs[it]
        val secondDeriv = secondDerivs[it]
        ValueMotionState(loc, deriv, secondDeriv)
    }.zipWithNext { a, b -> QuinticSpline.fromDerivatives(a, b) }
}

private fun <T : Any> List<T>.tryGet(index: Int): T? = if (index !in indices) null else this[index]

private fun getVecs(
    points: List<Waypoint>, i: Int, cur: Waypoint
): Pair<Vector2d?, Vector2d?> {
    val prev = points.tryGet(i - 1)
    val next = points.tryGet(i + 1)

    val fromVec = if (prev == null) null else cur.location - prev.location
    val toVec = if (next == null) null else next.location - cur.location
    return Pair(fromVec, toVec)
}
