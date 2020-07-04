package org.futurerobotics.jargon.profile

import org.futurerobotics.jargon.math.RealMotionState
import org.futurerobotics.jargon.math.avg
import org.futurerobotics.jargon.util.Stepper
import kotlin.math.pow

/**
 * A [ForwardMotionProfile] composed of interpolated segments with constant acceleration, for one-dimensional motion.
 */
class SegmentsForwardMotionProfile private constructor(private val segments: List<Segment>) : ForwardMotionProfile {

    override val duration: Double = segments.last().t
    override val distance: Double = segments.last().state.value
    override fun atTime(time: Double): RealMotionState {
        var i = segments.binarySearchBy(time) { it.t }
        if (i >= 0) return segments[i].state
        i = (-i - 2).coerceAtLeast(0)
        return segments[i].stateAtTime(time)
    }

    override fun atDistance(distance: Double): RealMotionState = segmentByDistance(distance).stateAtDist(distance)

    override fun timeAtDistance(distance: Double): Double = segmentByDistance(distance).timeAtDist(distance)

    private fun segmentByDistance(
        distance: Double
    ): Segment {
        var i = segments.binarySearchBy(distance) { it.x }
        if (i < 0) {
            i = (-i - 2).coerceAtLeast(0)
        }
        return segments[i]
    }

    override fun timeStepper(): Stepper<RealMotionState> = object : Stepper<RealMotionState> {
        private var i = -1

        override fun stepTo(step: Double): RealMotionState {
            //delegate past-end behavior... to the ends segments.
            if (i == -1) {
                i = when {
                    step <= 0 -> 0
                    step >= duration -> segments.lastIndex
                    else -> segments.binarySearchBy(step) { it.t }
                        .let { if (it < 0) -it - 2 else it }
                        .coerceIn(0, segments.lastIndex)
                }
            } else {
                while (i < segments.lastIndex && step >= segments[i + 1].t) i++
                while (i > 0 && step < segments[i].t) i--
            }
            return segments[i].stateAtTime(step)
        }
    }

    /** A pre-calculated representation of the endpoints of segments. */
    private class Segment(val state: RealMotionState, val t: Double) {

        val x get() = state.s
        fun stateAtTime(t: Double) = state.afterTime(t - this.t)
        fun stateAtDist(x: Double) = state.atDistance(x)
        fun timeAtDist(x: Double) = t + state.timeElapsedAtDistance(x)
    }

    companion object {

        /**
         * Constructs a [SegmentsForwardMotionProfile] from a pair of points with associated velocities.
         *
         * This only works if the motion is always progressing forward (v>=0)
         *
         * Pairs must have finite values, be sorted on x, and have positive velocities.
         *
         * Time and accelerations are calculated.
         */
        @JvmStatic
        fun fromPointVelPairs(pairs: List<Pair<Double, Double>>): SegmentsForwardMotionProfile {
            require(pairs.isNotEmpty()) { "Must have at least one point" }
            require(pairs.all { it.first.isFinite() && it.second.isFinite() }) { "All x and v should be finite" }
            require(pairs.isSortedBy { it.first }) { "Motion must be progressing forward; x's must progress forward" }
            require(pairs.all { it.second >= 0 }) { "Motion must be progressing forward; All velocities must be >= 0" }
            var t = 0.0
            val segs = pairs.zipWithNext { (x1, v1), (x2, v2) ->
                require(!(v1 == 0.0 && v2 == 0.0)) { "Velocity can only be instantaneously 0, got two 0 velocities." }
                val dx = x2 - x1
                val a = (v2.pow(2) - v1.pow(2)) / 2 / dx //acceleration given (positive) velocities
                val seg = Segment(RealMotionState(x1, v1, a), t)
                val dt = dx / avg(v1, v2)
                t += dt
                seg
            } + pairs.last().let {
                Segment(RealMotionState(it.first, it.second, 0.0), t)
            }
            return SegmentsForwardMotionProfile(segs)
        }
        //Other factory methods perhaps someday.
    }
}

/** @return true if the values of this iterator is sorted with values given by the [selector], inlined. */
private inline fun <T, V : Comparable<V>> Iterable<T>.isSortedBy(selector: (T) -> V): Boolean {
    val iterator = iterator()
    var prev = if (iterator.hasNext()) selector(iterator.next()) else return true
    while (iterator.hasNext()) {
        val cur = selector(iterator.next())
        if (cur < prev) return false
        prev = cur
    }
    return true
}
