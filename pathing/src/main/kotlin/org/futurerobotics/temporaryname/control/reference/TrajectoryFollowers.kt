package org.futurerobotics.temporaryname.control.reference

import org.futurerobotics.temporaryname.control.MotionProfileFollower
import org.futurerobotics.temporaryname.control.TimeOnlyProfileFollower
import org.futurerobotics.temporaryname.math.Pose2d
import org.futurerobotics.temporaryname.motionprofile.PoseMotionState

//TODO: THE ENTIRE THING
//POSSIBLY MOVE TO Pathing INSTEAD

/**
 * A time based TrajectoryFollower that uses [Pose2d] as it's state.
 *
 * Can be "chained" with other mappings to transform [Pose2d] into other state representations.
 */
class TimeBasedTrajectoryFollower(initialPose: Pose2d) :
    TimeOnlyProfileFollower<PoseMotionState, Pose2d, Pose2d>(initialPose, Pose2d.ZERO) {
    override fun mapState(profileState: PoseMotionState): Pair<Pose2d, Pose2d?> {
        return profileState.pose to profileState.vel
    }
}

/**
 * Uses a simple heuristic to estimate how fast the bot is moving relative to an actual Trajectory.
 *
 * This estimates the velocity of the robot, then projects it onto the reference velocity to estimate
 * the virtual time elapsed, also moving slower the further away the bot is from  If the actual velocity of the reference is below [minimumSpeed], then
 * the clock will be used instead.
 */
class AdaptiveTrajectoryFollower(initialPose: Pose2d, private val minimumSpeed: Double) :
    MotionProfileFollower<PoseMotionState, Pose2d, Pose2d>(initialPose, Pose2d.ZERO) {
    override fun getNextTime(
        pastOutput: Pair<Pose2d, Pose2d?>,
        pastState: Pose2d,
        currentState: Pose2d,
        pastVirtualTime: Double,
        elapsedNanos: Long
    ): Double {
        val estimatedDeriv = currentState.vec - pastState.vec
        TODO()
    }

    override fun mapState(profileState: PoseMotionState): Pair<Pose2d, Pose2d?> {
        return profileState.pose to profileState.vel
    }
}