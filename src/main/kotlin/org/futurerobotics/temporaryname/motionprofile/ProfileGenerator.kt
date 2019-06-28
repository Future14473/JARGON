package org.futurerobotics.temporaryname.motionprofile

import org.futurerobotics.temporaryname.math.*
import org.futurerobotics.temporaryname.util.localMap
import kotlin.math.ceil
import kotlin.math.min
import kotlin.math.sqrt

private const val MAX_VEL = 10000.0

/**
 * Generates approximate optimal [MotionProfile]
 */
object MotionProfileGenerator {
    /**
     * Calculates an numerically approximated optimal [MotionProfile], given [ProfileConstraint].
     *
     * [targetStartVel] and [targetEndVel] specify the target start and end velocities, respectively, however, if this
     * results in a profile not within constraints, the actual start and end velocities may be lower.
     *
     * [segmentSize] specifies the size of the segments used in the profile generation algorithm.
     *
     * [nonIntersectSearchTolerance] specifies the tolerance in which the maximum velocities due to
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
    fun generateProfile(
        constraint: ProfileConstraint,
        distance: Double,
        targetStartVel: Double = 0.0,
        targetEndVel: Double = 0.0,
        segmentSize: Double = 0.01,
        nonIntersectSearchTolerance: Double = 0.01 //may introduce a class if start to have too many parameters
    ): MotionProfile {
        val segments = getNumSegmentsAndCheck(distance, segmentSize)
        require(targetStartVel >= 0) { "targetStartVel ($targetStartVel) must be >= 0" }
        require(targetEndVel >= 0) { "targetStartVel ($targetEndVel) must be >= 0" }
        val progression = DoubleProgression.fromNumSegments(0.0, distance, segments)
        val points = progression.toList()
        val (rawVels, accelGetters) = constraint.getAllVelsAndAccels(points)
        require(rawVels.size == points.size) {
            "Max velocities returned by constraint has size (${rawVels.size}) not equal to the points given's size${points.size}"
        }
        require(accelGetters.size == points.size) {
            "Acceleration getters  returned constraint has size (${accelGetters.size}) not equal to the points given's size${points.size}"
        }
        require(rawVels.all { it >= 0 }) { "All maximum velocities should be >= 0" }
        val maxVels = rawVels.toMutableList()

        maxVels[0] = min(maxVels[0], targetStartVel)
        maxVels.lastIndex.let { maxVels[it] = min(maxVels[it], targetEndVel) }
        maxVels.localMap { min(it, MAX_VEL) }

        accelerationPass(maxVels, accelGetters, points, false, nonIntersectSearchTolerance)
        accelerationPass( //reverse
            maxVels.asReversed(), accelGetters.asReversed(), points.asReversed(), true, nonIntersectSearchTolerance
        )

        val pointVelPairs = points.zip(maxVels)
        return MotionProfile.fromPointVelPairs(pointVelPairs)
    }

    private fun getNumSegmentsAndCheck(dist: Double, segmentSize: Double): Int {
        require(dist > 0) { "distance ($dist) must be > 0" }
        require(segmentSize > 0) { "segmentSize ($segmentSize) must be > 0" }
        require(segmentSize <= dist) { "segmentSize ($$segmentSize) must be <= dist ($dist)" }
        return ceil(dist / segmentSize).toInt()
    }

    private const val BINARY_SEARCH_INITIAL_STEP_RATIO = 3
    private fun accelerationPass(
        maxVels: MutableList<Double>,
        accelGetters: List<MaxAccelGetter>,
        points: List<Double>,
        reversed: Boolean,
        binarySearchTolerance: Double
    ) {
        val size = points.size
        repeat(size - 1) {
            var v0 = maxVels[it]
            val dx = points[it + 1] distTo points[it] //works for backwards
            val accelGetter = accelGetters[it]

            var aMax = getAMaxOrNaN(dx, v0, accelGetter, reversed)
            if (aMax.isNaN()) {
                val tolerance = v0 * binarySearchTolerance
                val initialStep = tolerance * BINARY_SEARCH_INITIAL_STEP_RATIO
                //OH NO, ITS BINARY SEARCH!
                // typically < 10 iterations, and happens < 1% of the time, and only occurs when nessecary
                val newV0 = extendingDownDoubleSearch(
                    0.0,
                    v0 - tolerance,
                    initialStep,
                    tolerance,
                    searchingFor = false
                ) { v ->
                    getAMaxOrNaN(dx, v, accelGetter, reversed).isNaN()
                }
                v0 = newV0
                maxVels[it] = newV0
                aMax = getAMaxOrNaN(dx, v0, accelGetter, reversed)
            }
            val v1 = sqrt(v0.squared() + 2 * aMax * dx).notNaNOrElse { 0.0 } //>=0
            val actualV1 = min(v1, maxVels[it + 1])
            maxVels[it + 1] = actualV1
        }
    }

    private fun getAMaxOrNaN(dx: Double, v: Double, accelGetter: MaxAccelGetter, reversed: Boolean): Double {
        val aMin = -v.squared() / 2 / dx
        val aMaxMaybe = accelGetter.getMaxAccel(v, reversed)
        return if (aMaxMaybe > aMin) aMaxMaybe else Double.NaN
    }
}
