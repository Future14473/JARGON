package org.futurerobotics.temporaryname.motionprofile

import org.futurerobotics.temporaryname.math.avg
import org.futurerobotics.temporaryname.math.squared
import org.futurerobotics.temporaryname.util.inlineIsSortedBy

/**
 * Represents a motion profile, constructed segments with a starting velocity and constant acceleration.
 * Velocities must always be positive or only instantaneously 0.
 */
class MotionProfile private constructor(private val segments: List<Segment>) {

    /** The total duration of this profile, in seconds */
    val duration: Double = segments.last().t
    /** The total length of this profile, in distance. */
    val length: Double = segments.last().state.x

    /** Gets the [MotionState1d] at the duration [time] after the start of this profile */
    fun getByTime(time: Double): MotionState1d = when {
        time <= 0.0 -> segments.first().state
        time >= length -> segments.last().state
        else -> run {
            var i = segments.binarySearchBy(time) { it.t }
            if (i >= 0) return segments[i].state
            i = -i - 2
            val curSeg = segments[i]
            val segT = time - curSeg.t
            return curSeg.stateAfterTime(segT)
        }
    }

    /** Gets the Motion State at the point traveling [distance] units along this profile */
    fun getByDistance(distance: Double): MotionState1d = when {
        distance <= 0.0 -> segments.first().state
        distance >= length -> segments.last().state
        else -> run {
            var i = segments.binarySearchBy(distance) { it.x }
            if (i >= 0) return segments[i].state
            i = -i - 2
            val curSeg = segments[i]
            val segX = distance - curSeg.x
            return curSeg.stateAfterDist(segX)
        }
    }

    /**
     * A pre-calculated representation of the endpoints of segments.
     */
    private class Segment(val state: MotionState1d, val t: Double) {
        val x get() = state.x
        fun stateAfterTime(t: Double) = state.afterTime(t)
        fun stateAfterDist(x: Double) = state.afterForwardDist(x)
    }

    companion object {

        /**
         * Constructs a [MotionProfile] simpleFrom a pair of points with associated velocities.
         * Pairs must have finite values, be sorted on x, and have positive velocities.
         * Time is calculated.
         */
        @JvmStatic
        fun fromPointVelPairs(pairs: List<Pair<Double, Double>>): MotionProfile {
            require(pairs.all { it.first.isFinite() && it.second.isFinite() }) { "All x and v should be finite" }
            require(pairs.inlineIsSortedBy { it.first }) { "Pairs must be sorted on x" }
            require(pairs.all { it.second >= 0 }) { "All v must be >= 0" }
            var t = 0.0
            val segs = pairs.zipWithNext { (x1, v1), (x2, v2) ->
                val dx = x2 - x1
                val a = (v2.squared() - v1.squared()) / 2 / dx //acceleration given (positive) velocities
                val seg = Segment(MotionState1d(x1, v1, a), t)
                val dt = dx / avg(v1, v2)
                require(dt.isFinite()) { "Velocity can only be instantaneously 0, got two 0 velocities." }
                t += dt
                return@zipWithNext seg
            } as MutableList<Segment> //cheeky
            val lastPair = pairs.last()
            segs += Segment(MotionState1d(lastPair.first, lastPair.second, 0.0), t)
            return MotionProfile(segs)
        }
        //Other factory methods someday, probably not necessary.
    }
}
