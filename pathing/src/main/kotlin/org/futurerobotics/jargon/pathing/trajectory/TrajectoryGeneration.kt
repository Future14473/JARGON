package org.futurerobotics.jargon.pathing.trajectory

import org.futurerobotics.jargon.math.Interval
import org.futurerobotics.jargon.pathing.Path
import org.futurerobotics.jargon.pathing.PathPoint
import org.futurerobotics.jargon.profile.MotionProfileConstrainer
import org.futurerobotics.jargon.profile.PointConstraint
import org.futurerobotics.jargon.profile.generateDynamicProfile
import org.futurerobotics.jargon.util.Stepper

/**
 * A collection of [VelConstraint]s, [AccelConstraint]s, and (flattened) [MultipleConstraint] used to construct
 * a [TrajectoryConstrainer] when paired with a Path for dynamic motion profile generation.
 *
 * This class is here because it can be reused when creating trajectories; however since the path may change
 * the constraint cannot be reused.
 *
 * Automatically extracts constraints from [MultipleConstraint] and removes duplicates as defined in [SingleConstraint]
 * If there are no VelocityConstraints or AccelConstraints, Fallback constraints will be used that have a flat maximum
 * of a 100,000 so that the algorithm doesn't die. This is usually not ideal, so put at least one constraint.
 */
class MotionConstraintSet(
    velConstraints: Iterable<VelConstraint>, accelConstraints: Iterable<AccelConstraint>,
    multipleConstraints: Iterable<MultipleConstraint> = emptyList()
) {

    /** This set's velocity constraints */
    val velConstraints: List<VelConstraint> =
        (velConstraints + multipleConstraints.flatMap { it.velConstraints })
            .removeRedundant()
            .takeIf { it.isNotEmpty() } ?: FALLBACK_VELOCITY_CONSTRAINTS

    /** This set's acceleration constraints */
    val accelConstraints: List<AccelConstraint> =
        (accelConstraints + multipleConstraints.flatMap { it.accelConstraints })
            .removeRedundant()
            .takeIf { it.isNotEmpty() } ?: FALLBACK_ACCEL_CONSTRAINTS

    constructor(constraints: Iterable<MotionConstraint>) : this(
        constraints.filterIsInstance<VelConstraint>(),
        constraints.filterIsInstance<AccelConstraint>(),
        constraints.filterIsInstance<MultipleConstraint>()
    )

    constructor(vararg constraints: MotionConstraint) : this(constraints.asIterable())

    /**
     * Generates a trajectory using the given [path] and this set of constraints.
     */
    fun generateTrajectory(
        path: Path, targetStartVel: Double = 0.0,
        targetEndVel: Double = 0.0,
        segmentSize: Double = 0.01
    ): Trajectory = generateTrajectory(path, this, targetStartVel, targetEndVel, segmentSize)

    private fun <T : SingleConstraint> Iterable<T>.removeRedundant(): List<T> {
        val newConstraints = toMutableList()
        forEach { cur ->
            newConstraints.removeIf { it !== cur && cur.otherIsRedundant(it) }
        }
        return newConstraints
    }
}

/**
 * The bridge between [MotionConstraintSet] and [MotionProfileConstrainer],
 * by applying the constraints on a path.
 *
 * This is separate from [MotionConstraintSet], which is (supposed to be) immutable,
 * since the path can differ while using the same constraints.
 */
class TrajectoryConstrainer(
    private val path: Path, motionConstraintSet: MotionConstraintSet
) : MotionProfileConstrainer {

    private val velConstraints = motionConstraintSet.velConstraints
    private val accelConstrains = motionConstraintSet.accelConstraints

    private fun getMaxVel(point: PathPoint): Double = velConstraints.map { it.maxVelocity(point) }.min()!!

    private fun getMaxAccel(point: PathPoint, curVelocity: Double): Interval {
        return accelConstrains.map {
            it.accelRange(point, curVelocity)
        }.reduce(Interval::intersect)
    }

    override fun stepper(): Stepper<Double, PointConstraint> {
        val pathStepper = path.stepper()
        return Stepper { x ->
            val point = pathStepper.stepTo(x)
            object : PointConstraint {
                override val maxVelocity: Double = getMaxVel(point)
                override fun accelRange(currentVelocity: Double): Interval = getMaxAccel(point, currentVelocity)
            }
        }
    }
}

/**
 * Generates a approximate-time optimal trajectory given the [path] and [constraints].
 * [targetStartVel] and [targetEndVel] indicate the endpoints
 *
 * @see generateDynamicProfile
 */
fun generateTrajectory(
    path: Path,
    constraints: MotionConstraintSet,
    targetStartVel: Double = 0.0,
    targetEndVel: Double = 0.0,
    segmentSize: Double = 0.01
): Trajectory {
    val profileConstraint = TrajectoryConstrainer(path, constraints)
    val profile = generateDynamicProfile( //checks done here...
        profileConstraint, path.length, targetStartVel, targetEndVel, segmentSize
    )
    return Trajectory(path, profile)
}
