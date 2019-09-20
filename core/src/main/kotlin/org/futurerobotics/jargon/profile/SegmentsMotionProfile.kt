package org.futurerobotics.jargon.profile

import org.futurerobotics.jargon.math.avg
import org.futurerobotics.jargon.math.squared
import org.futurerobotics.jargon.mechanics.LinearMotionState3
import org.futurerobotics.jargon.util.Stepper
import org.futurerobotics.jargon.util.isSortedBy
import org.futurerobotics.jargon.util.replaceIf

/**
 * A [MotionProfile] composed of interpolated segments with constant acceleration, for one-dimensional motion.
 */
class SegmentsMotionProfile private constructor(private val segments: List<Segment>) : MotionProfile {

    override val duration: Double = segments.last().t
    override val distance: Double = segments.last().state.s
    private inline val maxInd get() = segments.size - 1
    override fun atTime(time: Double): LinearMotionState3 {
        var i = segments.binarySearchBy(time) { it.t }
        if (i >= 0) return segments[i].state
        i = -i - 2
        return segments[i].stateAtTime(time)
    }

    override fun atDistance(distance: Double): LinearMotionState3 {
        var i = segments.binarySearchBy(distance) { it.x }
        if (i >= 0) return segments[i].state
        i = -i - 2
        return segments[i].stateAtDist(distance)
    }

    override fun stepper(): Stepper<Double, LinearMotionState3> = object :
        Stepper<Double, LinearMotionState3> {
        private var i = -1
        override fun stepTo(step: Double): LinearMotionState3 = step.let { time ->
            if (i == -1) {
                i = segments.binarySearchBy(time) { it.t }
                    .replaceIf({ it < 0 }) { -it - 2 }
                    .coerceIn(0, maxInd)
            } else {
                while (i < maxInd && time >= segments[i + 1].t) i++

                while (i > 0 && time < segments[i].t) i--
                // Delegate extreme points behavior to the ends
            }
            return segments[i].stateAtTime(time)
        }
    }

    /** A pre-calculated representation of the endpoints of segments. */
    private class Segment(val state: LinearMotionState3, val t: Double) {

        val x get() = state.s
        fun stateAtTime(t: Double) = state.afterTime(t - this.t)
        fun stateAtDist(x: Double) = state.atDist(x)
    }

    companion object {

        /**
         * Constructs a [SegmentsMotionProfile] from a pair of points with associated velocities.
         *
         * This only works if the motion is always progressing forward (v>=0)
         *
         * Pairs must have finite values, be sorted on x, and have positive velocities.
         *
         * Time and accelerations are calculated.
         */
        @JvmStatic
        fun fromPointVelPairs(pairs: List<Pair<Double, Double>>): SegmentsMotionProfile {
            require(pairs.all { it.first.isFinite() && it.second.isFinite() }) { "All x and v should be finite" }
            require(pairs.isSortedBy { it.first }) { "Motion must be progressing forward; x's must progress forward" }
            require(pairs.all { it.second >= 0 }) { "Motion must be progressing forward; All velocities must be >= 0" }
            var t = 0.0
            val segs = pairs.zipWithNext { (x1, v1), (x2, v2) ->
                val dx = x2 - x1
                val a = (v2.squared() - v1.squared()) / 2 / dx //acceleration given (positive) velocities
                val seg = Segment(LinearMotionState3(x1, v1, a), t)
                val dt = dx / avg(v1, v2)
                require(dt.isFinite()) { "Velocity can only be instantaneously 0, got two 0 velocities." }
                t += dt
                seg
            } + pairs.last().let {
                Segment(LinearMotionState3(it.first, it.second, 0.0), t)
            }
            return SegmentsMotionProfile(segs)
        }
        //Other factory methods someday, probably not necessary.
    }
}
