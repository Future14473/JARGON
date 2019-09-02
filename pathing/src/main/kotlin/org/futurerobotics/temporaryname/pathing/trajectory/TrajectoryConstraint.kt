package org.futurerobotics.temporaryname.pathing.trajectory

import org.futurerobotics.temporaryname.math.Interval
import org.futurerobotics.temporaryname.pathing.Path
import org.futurerobotics.temporaryname.pathing.PathPoint
import org.futurerobotics.temporaryname.pathing.constraint.MotionConstraintSet
import org.futurerobotics.temporaryname.profile.MotionProfileConstrainer
import org.futurerobotics.temporaryname.profile.PointConstraint
import org.futurerobotics.temporaryname.util.Stepper

/**
 * The bridge between [MotionConstraintSet] and [MotionProfileConstrainer],
 * by applying the constraints on a path.
 *
 * This is separate from [MotionConstraintSet] since the path can differ along the same constraints.
 *
 * Used in [TrajectoryGenerator]
 */
class TrajectoryConstraint(
    private val path: Path, motionConstraintSet: MotionConstraintSet
) : MotionProfileConstrainer {

    private val velConstraints = motionConstraintSet.velocityConstraints
    private val accelConstrains = motionConstraintSet.accelConstraints
    private fun getMaxVel(point: PathPoint): Double {
        return velConstraints.map { it.maxVelocity(point) }.min()!!
    }

    private fun getMaxAccelBy(point: PathPoint, curVelocity: Double): Interval {
        return accelConstrains.map {
            it.maxAccelRange(point, curVelocity)
        }.reduce(Interval::intersect)
    }

    override fun stepper(): Stepper<Double, PointConstraint> {
        val pathStepper = path.stepper()
        return Stepper { x ->
            val point = pathStepper.stepTo(x)
            val maxVel = getMaxVel(point)
            return@Stepper PointConstraint(maxVel) { curVel ->
                getMaxAccelBy(point, curVel)
            }
        }
    }
}
