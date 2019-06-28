package org.futurerobotics.temporaryname.pathing.trajectory

import org.futurerobotics.temporaryname.math.Interval
import org.futurerobotics.temporaryname.motionprofile.MaxAccelGetter
import org.futurerobotics.temporaryname.motionprofile.ProfileConstraint
import org.futurerobotics.temporaryname.pathing.constraint.MotionConstraintSet
import org.futurerobotics.temporaryname.pathing.path.Path
import org.futurerobotics.temporaryname.pathing.path.PathPointInfo

/**
 * Provides the bridge between [MotionConstraintSet] and [ProfileConstraint], by applying the constraints on a path.
 * Used in [TrajectoryGenerator]
 */
class TrajectoryConstraint(
    private val path: Path, motionConstraintSet: MotionConstraintSet
) : ProfileConstraint {
    private val velConstraints = motionConstraintSet.velocityConstraints
    private val accelConstrains = motionConstraintSet.accelConstraints
    override fun getMaxVelocity(x: Double): Double {
        return getMaxVelBy(path.getPointInfo(x))
    }

    override fun getMaxAccel(x: Double, curVelocity: Double, reversed: Boolean): Double {
        return getMaxAccelBy(path.getPointInfo(x), curVelocity, reversed)
    }

    override fun getAllVelsAndAccels(allX: List<Double>): Pair<List<Double>, List<MaxAccelGetter>> {
        val points = path.getAllPointInfo(allX)
        val vels = points.map { getMaxVelBy(it) }
        val accels = points.map {
            MaxAccelGetter { curVelocity, reversed ->
                getMaxAccelBy(it, curVelocity, reversed)
            }
        }
        return vels to accels
    }

    private fun getMaxVelBy(point: PathPointInfo) = velConstraints.map { it.maxVelocity(point) }.min()!!

    private fun getMaxAccelBy(point: PathPointInfo, curVelocity: Double, reversed: Boolean): Double {
        val interval = accelConstrains.map {
            it.maxAccelRange(
                point, curVelocity, reversed
            )
        }.reduce(Interval::intersect)
        return if (reversed) -interval.a else interval.b
    }
}