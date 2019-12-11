package org.futurerobotics.jargon.pathing.trajectory

import org.futurerobotics.jargon.math.*
import org.futurerobotics.jargon.pathing.Path
import org.futurerobotics.jargon.pathing.PathPoint
import org.futurerobotics.jargon.profile.MotionProfile
import org.futurerobotics.jargon.profile.MotionProfiled
import org.futurerobotics.jargon.util.Stepper
import java.io.Serializable
import kotlin.math.pow

/**
 * Represents a trajectory; that is a Path paired with time/velocity info (a [MotionProfile]).
 *
 * This links to the rest of the world through implementing the [MotionProfiled] interface.
 *
 * @see generateTrajectory
 */
class Trajectory(private val path: Path, private val profile: MotionProfile) : MotionProfiled<MotionState<Pose2d>>,
                                                                               Serializable {

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
     * @see [Path]
     */
    val distance: Double get() = path.length

    /**
     * Gets the time along the profile at a given [distance].
     */
    fun timeAtDistance(distance: Double) = profile.timeAtDistance(distance)

    /**
     * Gets the [MotionState] of Poses after the specified [time] traversing this trajectory.
     */
    override fun atTime(time: Double): MotionState<Pose2d> {
        val state = profile.atTime(time)
        val point = path.pointAt(state.value)
        return getState(state, point)
    }

    override fun stepper(): Stepper<MotionState<Pose2d>> {
        val pathStepper = path.stepper()
        val profileStepper = profile.stepper()
        return Stepper {
            val state = profileStepper.stepTo(it)
            val point = pathStepper.stepTo(state.value)
            getState(state, point)
        }
    }

    private fun getState(state: LinearMotionState, point: PathPoint): MotionState<Pose2d> {
        val pose = point.pose
        val poseDeriv = point.poseDeriv
        val poseSecondDeriv = point.poseSecondDeriv
        return ValueMotionState(
            pose,
            poseDeriv * state.deriv,
            poseSecondDeriv * state.deriv.pow(2) + poseDeriv * state.secondDeriv
        )
    }

    companion object {
        private const val serialVersionUID: Long = 5481952978810220011
    }
}
