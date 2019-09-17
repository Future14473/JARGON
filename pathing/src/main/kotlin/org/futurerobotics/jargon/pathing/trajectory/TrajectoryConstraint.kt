package org.futurerobotics.jargon.pathing.trajectory

import org.futurerobotics.jargon.math.Interval
import org.futurerobotics.jargon.pathing.Path
import org.futurerobotics.jargon.pathing.PathPoint
import org.futurerobotics.jargon.profile.MotionProfileConstrainer
import org.futurerobotics.jargon.profile.PointConstraint
import org.futurerobotics.jargon.util.Stepper

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
