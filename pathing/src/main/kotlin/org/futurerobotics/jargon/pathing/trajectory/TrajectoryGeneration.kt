package org.futurerobotics.jargon.pathing.trajectory

import org.futurerobotics.jargon.math.Interval
import org.futurerobotics.jargon.pathing.Path
import org.futurerobotics.jargon.pathing.PathPoint
import org.futurerobotics.jargon.profile.MotionProfileConstrainer
import org.futurerobotics.jargon.profile.MotionProfileGenParams
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
            .also { require(it.isNotEmpty()) { "Need at least one velocity constraint." } }

    /** This set's acceleration constraints */
    val accelConstraints: List<AccelConstraint> =
        (accelConstraints + multipleConstraints.flatMap { it.accelConstraints })
            .removeRedundant()

    constructor(constraints: Iterable<MotionConstraint>) : this(
        constraints.filterIsInstance<VelConstraint>(),
        constraints.filterIsInstance<AccelConstraint>(),
        constraints.filterIsInstance<MultipleConstraint>()
    )

    constructor(vararg constraints: MotionConstraint) : this(constraints.asIterable())

    /**
     * Generates a trajectory using the current constraints, the given [path], and motion profile generation [params].
     */
    fun generateTrajectory(
        path: Path, params: MotionProfileGenParams
    ): Trajectory = generateTrajectory(path, this, params)

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

    override val requiredPoints: Set<Double>
        get() = path.criticalPoints

    private fun getMaxVel(point: PathPoint): Double = velConstraints.map { it.maxVelocity(point) }.min()!!

    private fun getMaxAccel(point: PathPoint, curVelocity: Double): Interval =
        if (accelConstrains.isEmpty()) Interval.REAL else
            accelConstrains.map {
                it.accelRange(point, curVelocity)
            }.reduce(Interval::intersect)

    override fun stepper(): Stepper<PointConstraint> {
        val pathStepper = path.stepper()
        return Stepper { x ->
            val point = pathStepper.stepTo(x)
            val maxVelocity = if (x in path.stopPoints) 0.0 else getMaxVel(point)
            object : PointConstraint {
                override val maxVelocity: Double = maxVelocity
                override fun accelRange(currentVelocity: Double): Interval = getMaxAccel(point, currentVelocity)
            }
        }
    }
}

/**
 * Generates a approximate-time optimal trajectory given the [path] and [constraints], and motion profile generation
 * [params].
 *
 * @see generateDynamicProfile
 */
fun generateTrajectory(
    path: Path,
    constraints: MotionConstraintSet,
    params: MotionProfileGenParams
): Trajectory {
    val profileConstraint = TrajectoryConstrainer(path, constraints)
    val profile = generateDynamicProfile( //checks done here...
        profileConstraint, path.length, params
    )
    return Trajectory(path, profile)
}
