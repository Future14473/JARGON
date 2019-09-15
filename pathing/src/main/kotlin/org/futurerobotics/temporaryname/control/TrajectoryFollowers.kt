package org.futurerobotics.temporaryname.control

import org.futurerobotics.temporaryname.math.Pose2d
import org.futurerobotics.temporaryname.mechanics.State
import org.futurerobotics.temporaryname.mechanics.ValueState

/**
 * Base implementation of a Trajectory follower, that uses [tolerances] to estimate if done.
 *
 * @param tolerances the [PoseTolerance] to use to indicate if we can safely turn off the control system.
 */
abstract class BaseTrajectoryFollower(var tolerances: PoseTolerance = PoseTolerance.NONE) :
    MotionProfileFollower<Pose2d, State<Pose2d>>() {

    override fun getIdleReference(pastReference: State<Pose2d>?, currentState: Pose2d): State<Pose2d> =
        pastReference ?: ValueState(currentState, Pose2d.ZERO, Pose2d.ZERO)

    override val isDone: Boolean
        get() = reference.v epsEq Pose2d.ZERO &&
                reference.a epsEq Pose2d.ZERO &&
                pastState.let { it != null && tolerances.areSatisfied(it, reference.s) }
}

/**
 * A TrajectoryFollower that only uses measured time to step forward.
 */
class TimeOnlyTrajectoryFollower(tolerances: PoseTolerance = PoseTolerance.NONE) :
    BaseTrajectoryFollower(tolerances) {

    override fun getNextTime(
        pastState: Pose2d,
        currentState: Pose2d,
        pastOutput: State<Pose2d>,
        pastVirtualTime: Double,
        elapsedSeconds: Double
    ): Double {
        return pastVirtualTime + elapsedSeconds
    }
}

/**
 * Uses a simple heuristic to estimate how fast the bot is moving relative to an actual Trajectory.
 *
 * This estimates the velocity of the robot, then projects it onto the reference velocity to estimate
 * the virtual time elapsed, also moving slower the further away the bot is from  If the actual velocity
 * of the reference is below [minimumSpeed], then the clock will be used instead.
 */
//TODO
private class AdaptiveTrajectoryFollower(
    private val minimumSpeed: Double,
    tolerances: PoseTolerance = PoseTolerance.NONE
) :
    BaseTrajectoryFollower(tolerances) {

    override fun getNextTime(
        pastState: Pose2d,
        currentState: Pose2d,
        pastOutput: State<Pose2d>,
        pastVirtualTime: Double,
        elapsedSeconds: Double
    ): Double {
        TODO()
    }
}