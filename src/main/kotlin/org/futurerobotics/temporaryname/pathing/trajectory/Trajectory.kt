package org.futurerobotics.temporaryname.pathing.trajectory

import org.futurerobotics.temporaryname.math.epsEq
import org.futurerobotics.temporaryname.motionprofile.MotionProfile
import org.futurerobotics.temporaryname.motionprofile.MotionState1d
import org.futurerobotics.temporaryname.pathing.path.Path
import org.futurerobotics.temporaryname.pathing.path.pose
import org.futurerobotics.temporaryname.pathing.path.poseDeriv

/**
 * Represents a trajectory; that is a Path paired with time/velocity info on traversal (a [MotionProfile]).
 *
 * @see TrajectoryGenerator
 */
class Trajectory(private val path: Path, internal val profile: MotionProfile) {

    init {
        require(path.length epsEq profile.length) {
            "Path length ${path.length} and profile length ${profile.length} must match"
        }
    }

    /**
     * The duration of time to traverse this [Trajectory] (ideally)
     *
     * _in a perfect world where friction and entropy and floating-
     * point errors and capacitance and noise and delay and approximation errors and internal resistance and
     * dampening and time and space don't exist._
     * */
    val duration: Double get() = profile.duration

    /**
     * The total length of this Trajectory
     * @see [Path]
     */
    val length: Double get() = path.length

    /**
     * Gets the [FieldMotion] at the specified [time] traversing this trajectory.
     */
    fun getByTime(time: Double): FieldMotion {
        return getByState(profile.getByTime(time))
    }

    /**
     * Gets the the [FieldMotion] at the specified [distance] along this trajectory.
     */
    fun getByDist(distance: Double): FieldMotion {
        return getByState(profile.getByDistance(distance))
    }

    private fun getByState(state: MotionState1d): FieldMotion {
        val point = path.getPointInfo(state.x)
        return FieldMotion(point.pose, point.poseDeriv * state.v)
    }
}