package org.futurerobotics.temporaryname.motionprofile

import org.futurerobotics.temporaryname.math.avg
import org.futurerobotics.temporaryname.math.squared
import org.futurerobotics.temporaryname.util.isSortedBy

/**
 * A [MotionProfile] composed of interpolated segments with constant acceleration, for one-dimensional motion.
 */
class SegmentsMotionProfile private constructor(private val segments: List<Segment>) : MotionProfile {

    override val duration: Double = segments.last().t
    override val length: Double = segments.last().state.x

    override fun atTime(time: Double): MotionState1d {
        var i = segments.binarySearchBy(time) { it.t }
        if (i >= 0) return segments[i].state
        i = -i - 2
        val curSeg = segments[i]
        val segT = time - curSeg.t
        return curSeg.stateAfterTime(segT)
    }

    override fun atDistance(distance: Double): MotionState1d {
        var i = segments.binarySearchBy(distance) { it.x }
        if (i >= 0) return segments[i].state
        i = -i - 2
        val curSeg = segments[i]
        val segX = distance - curSeg.x
        return curSeg.stateAfterDist(segX)
    }

    /** A pre-calculated representation of the endpoints of segments. */
    private class Segment(val state: MotionState1d, val t: Double) {
        val x get() = state.x
        fun stateAfterTime(t: Double) = state.afterTime(t)
        fun stateAfterDist(x: Double) = state.afterForwardDist(x)
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
                val seg = Segment(MotionState1d(x1, v1, a), t)
                val dt = dx / avg(v1, v2)
                require(dt.isFinite()) { "Velocity can only be instantaneously 0, got two 0 velocities." }
                t += dt
                seg
            } + pairs.last().let {
                Segment(MotionState1d(it.first, it.second, 0.0), t)
            }
            return SegmentsMotionProfile(segs)
        }
        //Other factory methods someday, probably not necessary.
    }
}
