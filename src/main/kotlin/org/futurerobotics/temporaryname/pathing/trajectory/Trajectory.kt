package org.futurerobotics.temporaryname.pathing.trajectory

import org.futurerobotics.temporaryname.math.epsEq
import org.futurerobotics.temporaryname.motionprofile.MotionProfile
import org.futurerobotics.temporaryname.motionprofile.MotionProfiled
import org.futurerobotics.temporaryname.motionprofile.MotionState1d
import org.futurerobotics.temporaryname.motionprofile.PoseMotionState
import org.futurerobotics.temporaryname.pathing.Path
import org.futurerobotics.temporaryname.pathing.PathPoint
import org.futurerobotics.temporaryname.pathing.pose
import org.futurerobotics.temporaryname.pathing.poseDeriv
import org.futurerobotics.temporaryname.util.Stepper

/**
 * Represents a trajectory; that is a Path paired with time/velocity info on traversal (a [MotionProfiled]).
 *
 * @see TrajectoryGenerator
 */
class Trajectory(private val path: Path, internal val profile: MotionProfile) : MotionProfiled<PoseMotionState> {

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
    override val length: Double get() = path.length

    init {
        require(path.length epsEq profile.length) {
            "Path length ${path.length} and profile length ${profile.length} must match"
        }
    }

    /**
     * Gets the [PoseMotionState] after the specified [time] traversing this trajectory.
     */
    override fun atTime(time: Double): PoseMotionState {
        return stateFrom(profile.atTime(time))
    }

    /**
     * Gets the the [PoseMotionState] at the specified [distance] along this trajectory.
     */
    override fun atDistance(distance: Double): PoseMotionState {
        return stateFrom(profile.atDistance(distance))
    }

    override fun timeStepper(): Stepper<Double, PoseMotionState> {
        return stepperFrom(profile.timeStepper())
    }

    override fun distanceStepper(): Stepper<Double, PoseMotionState> {
        return stepperFrom(profile.distanceStepper())
    }

    private fun stepperFrom(profileStepper: Stepper<Double, MotionState1d>): Stepper<Double, PoseMotionState> {
        val pathStepper = path.stepper()
        return Stepper {
            val state = profileStepper.stepTo(it)
            val point = pathStepper.stepTo(state.x)
            getState(state, point)
        }
    }

    private fun stateFrom(state: MotionState1d): PoseMotionState {
        val point = path.pointAt(state.x)
        return getState(state, point)
    }

    private fun getState(state: MotionState1d, point: PathPoint): PoseMotionState {
        return PoseMotionState(point.pose, point.poseDeriv * state.v)
    }
}
