package org.futurerobotics.temporaryname.pathing.trajectory

import org.futurerobotics.temporaryname.pathing.Path
import org.futurerobotics.temporaryname.pathing.constraint.MotionConstraintSet
import org.futurerobotics.temporaryname.profile.MotionProfileGenerator

/**
 * Generates [Trajectory] from a [Path] and a [MotionConstraintSet] via [MotionProfileGenerator]
 */
object TrajectoryGenerator {

    /**
     * Generates a approximate-time optimal trajectory given the [path] and [constraints].
     * [targetStartVel] and [targetEndVel] indicate the endpoints
     *
     * @see MotionProfileGenerator
     */
    @JvmStatic
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
}
