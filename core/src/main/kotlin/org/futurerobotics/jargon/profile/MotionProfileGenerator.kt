package org.futurerobotics.jargon.profile

import org.futurerobotics.jargon.math.*
import org.futurerobotics.jargon.util.extendingDownDoubleSearch
import org.futurerobotics.jargon.util.localMap
import org.futurerobotics.jargon.util.stepToAll
import kotlin.math.ceil
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sqrt

private const val MAX_VEL = 10000.0

/**
 * Holds the algorithm to generates approximate optimal [MotionProfiled]
 */
object MotionProfileGenerator {

    private const val BINARY_SEARCH_INITIAL_STEP_RATIO = 2
    /**
     * Calculates an numerically approximated optimal [MotionProfiled], given [MotionProfileConstrainer].
     *
     * [targetStartVel] and [targetEndVel] specify the target start and end velocities, respectively, however, if this
     * results in a profile not within constraints, the actual start and end velocities may be lower.
     *
     * [segmentSize] specifies the size of the segments used in the profile generation algorithm.
     *
     * [maxVelDueToAccelsSearchTolerance] specifies the tolerance in which the maximum velocities due to
     * satisfying acceleration constraints will be searched for, if needed. (heavy heuristic
     * binary search). If constraints are "non demanding", binary search will not happen.
     *
     * This uses a modified version of the algorithm described in section 3.2 of:
     *  [http://www2.informatik.uni-freiburg.de/~lau/students/Sprunk2008.pdf].
     *
     *  The differences include: this method only considers the maximum velocity and acceleration at the point at the
     *  start of a segment (or the other endpoint when doing a backwards pass), and uses that
     *  to determine the maximum velocities and acceleration of the entire segment. This changes the formulas used
     *  to determine maximum centripetal acceleration and others. This does introduce some approximation
     *  error, and requires some binary search, but it _greatly_ simplifies calculations,  More importantly, this allows
     *  for a more modular and a wider variety of combination of constraints.
     *  The error introduced is negligible or tiny when segments are sufficiently small, and binary search has shown
     *  to not significantly impact generation times, and can be avoided altogether with "non-demanding" constraints.
     *
     *  I will write a paper someday.
     */
    @JvmOverloads
    @JvmStatic
    fun generateDynamicProfile(
        constrainer: MotionProfileConstrainer,
        distance: Double,
        targetStartVel: Double = 0.0,
        targetEndVel: Double = 0.0,
        segmentSize: Double = 0.01,
        maxVelDueToAccelsSearchTolerance: Double = 0.01
        //may introduce a class if start to have too many parameters
    ): MotionProfile {
        require(distance > 0) { "distance ($distance) must be > 0" }
        require(targetStartVel >= 0) { "targetStartVel ($targetStartVel) must be >= 0" }
        require(targetEndVel >= 0) { "targetEndVel ($targetEndVel) must be >= 0" }
        require(segmentSize > 0) { "segmentSize ($segmentSize) must be > 0" }
        require(segmentSize <= distance) { "segmentSize ($$segmentSize) must be <= dist ($distance)" }
        require(maxVelDueToAccelsSearchTolerance > 0) { "nonIntersectSearchTolerance ($maxVelDueToAccelsSearchTolerance) must be > 0" }
        val segments = ceil(distance / segmentSize).toInt()
        val points = DoubleProgression.fromNumSegments(0.0, distance, segments).toList()
        val pointConstraints: List<PointConstraint> = constrainer.stepToAll(points)
        val maxVels = pointConstraints.mapTo(ArrayList(points.size)) { it.maxVelocity }
        require(maxVels.all { it >= 0 }) { "All maximum velocities given by constrainer should be >= 0" }

        maxVels[0] = min(maxVels[0], targetStartVel)
        maxVels.lastIndex.let {
            maxVels[it] = min(maxVels[it], targetEndVel)
        }
        maxVels.localMap { min(it, MAX_VEL) }

        accelerationPass(maxVels, pointConstraints, points, maxVelDueToAccelsSearchTolerance, false)
        accelerationPass( //reverse
            maxVels.asReversed(),
            pointConstraints.asReversed(),
            points.asReversed(),
            maxVelDueToAccelsSearchTolerance,
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
                if (v0 == 0.0) throwBadAccelAt0Vel()
                //OH NO, ITS BINARY SEARCH!
                // heuristic search, typically < 10 iterations, and only occurs when necessary,
                // and typically happens < 1% of the time
                val newV0 = extendingDownDoubleSearch(
                    0.0, v0 - tolerance, tolerance, searchingFor = false
                ) { v -> getAMaxOrNaN(dx, v, accelGetter, reversed).isNaN() }
                aMax = getAMaxOrNaN(dx, v0, accelGetter, reversed).notNaNOrElse(::throwBadAccelAt0Vel)
                v0 = newV0
                maxVels[it] = newV0 //is new
            }
            val v1 = sqrt(v0.squared() + 2 * aMax * dx).notNaNOrElse { 0.0 }
            val actualV1 = min(v1, maxVels[it + 1])
            maxVels[it + 1] = actualV1
        }
    }

    private fun throwBadAccelAt0Vel(): Nothing =
        throw RuntimeException(
            "Unsatisfiable constraints: The current constraints's acceleration did not return a non-empty interval" +
                    " even at a given velocity of 0.0."
        )

    private fun getAMaxOrNaN(dx: Double, v: Double, accelGetter: PointConstraint, reversed: Boolean): Double {
        val aMin = -v.squared() / 2 / dx
        val interval = accelGetter.accelRange(v)
        val aMaxMaybe = if (reversed) -interval.a else interval.b
        return if (aMaxMaybe > aMin) aMaxMaybe else Double.NaN
    }
}
