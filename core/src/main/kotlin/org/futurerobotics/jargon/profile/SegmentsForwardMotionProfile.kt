package org.futurerobotics.jargon.profile

import org.futurerobotics.jargon.math.LinearMotionState
import org.futurerobotics.jargon.math.avg
import org.futurerobotics.jargon.util.Stepper
import org.futurerobotics.jargon.util.isSortedBy
import org.futurerobotics.jargon.util.replaceIf
import kotlin.math.pow

/**
 * A [ForwardMotionProfile] composed of interpolated segments with constant acceleration, for one-dimensional motion.
 */
class SegmentsForwardMotionProfile private constructor(private val segments: List<Segment>) : ForwardMotionProfile {

    override val duration: Double = segments.last().t
    override val length: Double = segments.last().state.value
    override fun atTime(time: Double): LinearMotionState {
        var i = segments.binarySearchBy(time) { it.t }
        if (i >= 0) return segments[i].state
        i = (-i - 2).coerceAtLeast(0)
        return segments[i].stateAtTime(time)
    }

    override fun atLength(length: Double): LinearMotionState = segmentByDistance(length).stateAtDist(length)

    override fun timeAtLength(length: Double): Double = segmentByDistance(length).timeAtDist(length)

    private fun segmentByDistance(
        distance: Double
    ): Segment {
        var i = segments.binarySearchBy(distance) { it.x }
        if (i < 0) {
            i = (-i - 2).coerceAtLeast(0)
        }
        return segments[i]
    }

    override fun stepper(): Stepper<LinearMotionState> = object : Stepper<LinearMotionState> {
        private var i = -1

        override fun stepTo(step: Double): LinearMotionState {
            //delegate extreme point behavior to the ends.
            if (i == -1) {
                i = when {
                    step <= 0 -> 0
                    step >= duration -> segments.lastIndex
                    else -> segments.binarySearchBy(step) { it.t }
                        .replaceIf({ it < 0 }) { -it - 2 }
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
    private class Segment(val state: LinearMotionState, val t: Double) {

        val x get() = state.value
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
            require(pairs.all { it.first.isFinite() && it.second.isFinite() }) { "All x and v should be finite" }
            require(pairs.isSortedBy { it.first }) { "Motion must be progressing forward; x's must progress forward" }
            require(pairs.all { it.second >= 0 }) { "Motion must be progressing forward; All velocities must be >= 0" }
            var t = 0.0
            val segs = pairs.zipWithNext { (x1, v1), (x2, v2) ->
                require(!(v1 == 0.0 && v2 == 0.0)) { "Velocity can only be instantaneously 0, got two 0 velocities." }
                val dx = x2 - x1
                val a = (v2.pow(2) - v1.pow(2)) / 2 / dx //acceleration given (positive) velocities
                val seg = Segment(LinearMotionState(x1, v1, a), t)
                val dt = dx / avg(v1, v2)
                t += dt
                seg
            } + pairs.last().let {
                Segment(LinearMotionState(it.first, it.second, 0.0), t)
            }
            return SegmentsForwardMotionProfile(segs)
        }
        //Other factory methods someday, probably not necessary.
    }
}
