package org.futurerobotics.jargon.pathing.trajectory

import org.futurerobotics.jargon.math.MotionState
import org.futurerobotics.jargon.math.Pose2d
import org.futurerobotics.jargon.math.RealMotionState
import org.futurerobotics.jargon.math.epsEq
import org.futurerobotics.jargon.pathing.Path
import org.futurerobotics.jargon.pathing.PathPoint
import org.futurerobotics.jargon.profile.ForwardMotionProfile
import org.futurerobotics.jargon.profile.TimeDistanceProfiled
import org.futurerobotics.jargon.profile.TimeProfiled
import org.futurerobotics.jargon.util.Stepper
import kotlin.math.pow

/**
 * Represents a trajectory; i.e. a [Path] paired with time/velocity info (a [ForwardMotionProfile]).
 *
 * This links to the rest of the world through implementing the [TimeProfiled] interface.
 *
 * @see generateTrajectory
 */
class Trajectory(private val path: Path, private val profile: ForwardMotionProfile) :
    TimeDistanceProfiled<MotionState<Pose2d>> {

    init {
        require(path.length epsEq profile.distance) {
            "Path length ${path.length} and profile length ${profile.distance} must match"
        }
    }

    /**
     * The duration of time to traverse this [Trajectory] (ideally)
     *
     * _in a perfect world where friction and entropy and floating-point errors and capacitance and noise and delay
     * and approximation errors and internal resistance and model error and unmodeled dynamics
     * and time and space and life don't exist._
     * */
    override val duration: Double get() = profile.duration

    /**
     * The total length of this trajectory; i.e, the original path's length
     * @see Path
     */
    override val distance: Double get() = path.length

    override fun timeAtDistance(distance: Double): Double = profile.timeAtDistance(distance)

    /**
     * Gets the [MotionState] of Poses after the specified [time] traversing this trajectory.
     */
    override fun atTime(time: Double): MotionState<Pose2d> {
        val state = profile.atTime(time)
        val point = path.pointAt(state.s)
        return getState(state, point)
    }

    override fun atDistance(distance: Double): MotionState<Pose2d> {
        val state = profile.atDistance(distance)
        val point = path.pointAt(state.s)
        return getState(state, point)
    }

    override fun stepper(): Stepper<MotionState<Pose2d>> {
        val pathStepper = path.stepper()
        val profileStepper = profile.stepper()
        return Stepper { t ->
            val state = profileStepper.stepTo(t)
            val point = pathStepper.stepTo(state.s)
            getState(state, point)
        }
    }

    private fun getState(state: RealMotionState, point: PathPoint): MotionState<Pose2d> {
        val pose = point.pose
        val poseDeriv = point.poseDeriv
        val poseSecondDeriv = point.poseSecondDeriv
        return MotionState(
            pose,
            poseDeriv * state.v,
            poseSecondDeriv * state.v.pow(2) + poseDeriv * state.a
        )
    }
}
