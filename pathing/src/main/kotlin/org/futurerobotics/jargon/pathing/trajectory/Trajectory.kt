package org.futurerobotics.jargon.pathing.trajectory

import org.futurerobotics.jargon.math.Pose2d
import org.futurerobotics.jargon.math.epsEq
import org.futurerobotics.jargon.math.squared
import org.futurerobotics.jargon.mechanics.LinearMotionState3
import org.futurerobotics.jargon.mechanics.MotionState3
import org.futurerobotics.jargon.mechanics.ValueMotionState3
import org.futurerobotics.jargon.pathing.*
import org.futurerobotics.jargon.profile.MotionProfile
import org.futurerobotics.jargon.profile.MotionProfiled
import org.futurerobotics.jargon.util.Stepper

/**
 * Represents a trajectory; that is a Path paired with time/velocity info on traversal (a [MotionProfiled]).
 *
 * @see TrajectoryGenerator
 */
class Trajectory(private val path: Path, private val profile: MotionProfile) : MotionProfiled<MotionState3<Pose2d>> {

    /**
     * The duration of time to traverse this [Trajectory] (ideally)
     *
     * _in a perfect world where friction and entropy and floating-
     * point errors and capacitance and noise and delay and approximation errors and internal resistance and
     * dampening and time and space don't exist._
     * */
    override val duration: Double get() = profile.duration
    /**
     * The total length of this Trajectory
     * @see [Path]
     */
    val distance: Double get() = path.length

    init {
        require(path.length epsEq profile.distance) {
            "Path length ${path.length} and profile length ${profile.distance} must match"
        }
    }

    /**
     * Gets the [MotionState3] of Poses after the specified [time] traversing this trajectory.
     */
    override fun atTime(time: Double): MotionState3<Pose2d> {
        val state = profile.atTime(time)
        val point = path.pointAt(state.s)
        return getState(state, point)
    }

    override fun stepper(): Stepper<Double, MotionState3<Pose2d>> {
        val pathStepper = path.stepper()
        val profileStepper = profile.stepper()
        return Stepper {
            val state = profileStepper.stepTo(it)
            val point = pathStepper.stepTo(state.s)
            getState(state, point)
        }
    }

    private fun getState(state: LinearMotionState3, point: PathPoint): MotionState3<Pose2d> {
        val pose = point.pose
        val poseDeriv = point.poseDeriv
        val poseSecondDeriv = point.poseSecondDeriv
        return ValueMotionState3(
            pose,
            poseDeriv * state.v,
            poseSecondDeriv * state.v.squared() + poseDeriv * state.a
        )
    }
}
