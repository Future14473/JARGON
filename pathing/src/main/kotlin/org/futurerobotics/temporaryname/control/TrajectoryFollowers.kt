package org.futurerobotics.temporaryname.control

import org.futurerobotics.temporaryname.math.Pose2d
import org.futurerobotics.temporaryname.mechanics.PoseMotionState

/**
 * Base implementation of a Trajectory follower, that uses [tolerances] to estimate if done.
 */
abstract class BaseTrajectoryFollower(var tolerances: PoseTolerance = PoseTolerance.NONE) :
    MotionProfileFollower<Pose2d, PoseMotionState>() {

    override fun getIdleReference(pastReference: PoseMotionState?, currentState: Pose2d): PoseMotionState =
        pastReference ?: PoseMotionState(currentState, Pose2d.ZERO)

    override val isDone: Boolean
        get() = pastState.let { it != null && tolerances.areSatisifed(it, reference.s) }
}

/**
 * A time based TrajectoryFollower that uses [Pose2d] as it's state.
 *
 * Can be "chained" with other mappings to transform [Pose2d] into other state representations.
 */
class TimeOnlyTrajectoryFollower(tolerances: PoseTolerance = PoseTolerance.NONE) :
    BaseTrajectoryFollower(tolerances) {

    override fun getNextTime(
        pastState: Pose2d,
        currentState: Pose2d,
        pastOutput: PoseMotionState,
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
 * the virtual time elapsed, also moving slower the further away the bot is from  If the actual velocity of the reference is below [minimumSpeed], then
 * the clock will be used instead.
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
        pastOutput: PoseMotionState,
        pastVirtualTime: Double,
        elapsedSeconds: Double
    ): Double {
        TODO()
    }
}