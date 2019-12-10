@file:JvmName("GenerateMotionProfile")
@file:Suppress("ExplicitThis")

package org.futurerobotics.jargon.profile

import org.futurerobotics.jargon.math.DoubleProgression
import org.futurerobotics.jargon.math.EPSILON
import org.futurerobotics.jargon.math.distTo
import org.futurerobotics.jargon.math.ifNan
import org.futurerobotics.jargon.util.extendingDownDoubleSearch
import org.futurerobotics.jargon.util.mapToSelf
import org.futurerobotics.jargon.util.stepToAll
import kotlin.math.*

/**
 * Parameters for [generateDynamicProfile].
 *
 * @param targetStartVel the target start velocity of the profile. May actually be lower if constraints are not
 * satisfied.
 * @param targetEndVel the target end vel. May actually be lower if constraints demand it.
 * @param maxSegmentSize the maximum segment size allowed when the object is divided.
 * @param maxVelSearchTolerance the tolerance used when binary searching the maximum velocity due to _acceleration_
 *      constraints. Note that we made an effort to avoid binary search as much possible and the algorithm is
 *      heuristically optimized.
 */
data class MotionProfileGenParams(
    val targetStartVel: Double = 0.0,
    val targetEndVel: Double = 0.0,
    val maxSegmentSize: Double = 0.02,
    val maxVelSearchTolerance: Double = 0.02
) {

    init {
        require(targetStartVel >= 0) { "targetStartVel ($targetStartVel) must be >= 0" }
        require(targetEndVel >= 0) { "targetEndVel ($targetEndVel) must be >= 0" }
        require(maxSegmentSize > 0) { "segmentSize ($maxSegmentSize) must be > 0" }
        require(maxVelSearchTolerance > 0) { "maxVelSearchTolerance ($maxVelSearchTolerance) must be > 0" }
    }
}

private const val MAX_VEL = 10000.0
private const val EXTENDING_SEARCH_INITIAL_STEP_RATIO = 2

/**
 * Calculates an approximately optimal [MotionProfile], given a [MotionProfileConstrainer] and [MotionProfileGenParams]
 *
 * This uses a modified version of the algorithm described in section 3.2 of:
 *  [http://www2.informatik.uni-freiburg.de/~lau/students/Sprunk2008.pdf].
 *
 *  The differences include: this method only considers the maximum velocity and acceleration at the point at one
 *  endpoint of a segment (or the other endpoint when doing a backwards pass), and uses that to determine the
 *  maximum velocities and acceleration of the entire segment. This changes the formulas used to determine the
 *  constraints to use, but makes it much simpler. This does introduce some approximation error, and requires some
 *  binary search at some point, but it _greatly_ simplifies calculations,  More importantly, this allows for a more
 *  modular and a wider variety of combination of constraints. The error introduced is negligible or tiny when segments
 *  are sufficiently small, and binary search has shown to not significantly impact generation times, and can possibly
 *  be avoided altogether with "non-demanding" constraints.
 *
 *  I will write a short paper about this someday.
 */
fun generateDynamicProfile(
    constrainer: MotionProfileConstrainer,
    totalDistance: Double,
    params: MotionProfileGenParams
    //may introduce a parameters class if start to have too many parameters
): MotionProfile = params.run {
    require(totalDistance > 0) { "distance ($totalDistance) must be > 0" }

    require(maxSegmentSize <= totalDistance) {
        "segmentSize ($maxSegmentSize) must be <= dist ($totalDistance)"
    }
    val segments = ceil(totalDistance / this.maxSegmentSize).toInt()
    val points: List<Double> = DoubleProgression.fromNumSegments(0.0, totalDistance, segments)
        .toSortedSet()
        .also { it.addAll(constrainer.requiredPoints) }
        .toList()
    val pointConstraints: List<PointConstraint> = constrainer.stepToAll(points)
    val maxVels = pointConstraints.mapIndexedTo(ArrayList(points.size)) { i, it ->
        it.maxVelocity.also {
            require(it >= 0) { "All maximum velocities given by constrainer should be >= 0, got $it at segment $i" }
        }
    }


    maxVels[0] = min(maxVels[0], this.targetStartVel)
    maxVels.lastIndex.let {
        maxVels[it] = min(maxVels[it], targetEndVel)
    }
    maxVels.mapToSelf { min(it, MAX_VEL) }

    accelerationPass(maxVels, pointConstraints, points, maxVelSearchTolerance, false)
    accelerationPass( //reverse
        maxVels.asReversed(),
        pointConstraints.asReversed(),
        points.asReversed(),
        maxVelSearchTolerance,
        true
    )
    val pointVelPairs = points.zip(maxVels)
    return SegmentsMotionProfile.fromPointVelPairs(pointVelPairs)
}

private fun accelerationPass(
    maxVels: MutableList<Double>,
    accelGetters: List<PointConstraint>,
    points: List<Double>,
    binarySearchTolerance: Double,
    reversed: Boolean
) {
    val tolerance = max(binarySearchTolerance, EPSILON)
    repeat(points.size - 1) {
        var v0 = maxVels[it]
        val accelGetter = accelGetters[it]
        val dx = points[it] distTo points[it + 1] //works for backwards
        var aMax = getAMaxOrNaN(dx, v0, accelGetter, reversed)
        if (aMax.isNaN()) {
            if (v0 == 0.0) throwBadAccelAtZeroVel(points[it], points[it + 1], reversed)
            val newV0 = extendingDownDoubleSearch(
                0.0, v0, tolerance, searchingFor = false
            ) { v -> getAMaxOrNaN(dx, v, accelGetter, reversed).isNaN() }
            aMax = getAMaxOrNaN(dx, newV0, accelGetter, reversed)
                .ifNan { throwBadAccelAtZeroVel(points[it], points[it + 1], reversed) }
            v0 = newV0
            maxVels[it] = newV0
        }
        val v1 = sqrt(v0.pow(2) + 2 * aMax * dx).ifNan { 0.0 }
        val actualV1 = min(v1, maxVels[it + 1])
        maxVels[it + 1] = actualV1
    }
}

private fun getAMaxOrNaN(dx: Double, v: Double, accelGetter: PointConstraint, reversed: Boolean): Double {
    val aMin = -v.pow(2) / 2 / dx
    val interval = accelGetter.accelRange(v)
    val aMaxMaybe = if (reversed) -interval.a else interval.b
    return if (aMaxMaybe > aMin) aMaxMaybe else Double.NaN
}

private fun throwBadAccelAtZeroVel(x1: Double, x2: Double, reversed: Boolean): Nothing {
    val (p1, p2) = if (reversed) x2 to x1 else x1 to x2
    throw RuntimeException(
        "On the interval from ($p1 to $p2, reversed = $reversed), constraints did not return a non-empty acceleration" +
                " range even with a current velocity of 0.0."
    )
}
