/**
 * Holds the trajectory generation method.
 */
@file:JvmName("TrajectoryGenerator")

package org.futurerobotics.jargon.pathing.trajectory

import org.futurerobotics.jargon.pathing.Path
import org.futurerobotics.jargon.profile.MotionProfileGenerator

/**
 * Generates a approximate-time optimal trajectory given the [path] and [constraints].
 * [targetStartVel] and [targetEndVel] indicate the endpoints
 *
 * @see MotionProfileGenerator
 */
fun generateTrajectory(
    path: Path,
    constraints: MotionConstraintSet,
    targetStartVel: Double = 0.0,
    targetEndVel: Double = 0.0,
    segmentSize: Double = 0.01
): Trajectory {
    val profileConstraint = TrajectoryConstraint(path, constraints)
    val profile = MotionProfileGenerator.generateDynamicProfile( //checks done here...
        profileConstraint, path.length, targetStartVel, targetEndVel, segmentSize
    )
    return Trajectory(path, profile)
}