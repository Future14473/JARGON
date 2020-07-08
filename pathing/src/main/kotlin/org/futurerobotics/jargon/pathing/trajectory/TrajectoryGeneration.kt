package org.futurerobotics.jargon.pathing.trajectory

import org.futurerobotics.jargon.math.Interval
import org.futurerobotics.jargon.pathing.Path
import org.futurerobotics.jargon.pathing.PathPoint
import org.futurerobotics.jargon.profile.MotionProfileConstrainer
import org.futurerobotics.jargon.profile.MotionProfileGenParams
import org.futurerobotics.jargon.profile.PointConstraint
import org.futurerobotics.jargon.profile.generateDynamicProfile
import org.futurerobotics.jargon.util.Stepper
import org.futurerobotics.jargon.util.asUnmodifiableSet
import kotlin.math.min

/**
 * A collection of [VelocityConstraint]s, [AccelerationConstraint]s, and (flattened) [MultipleConstraint] used to construct
 * a [TrajectoryConstrainer] when paired with a Path for dynamic motion profile generation.
 *
 * This class is here because it can be reused when creating trajectories; however since the path may change
 * the constraint cannot be reused.
 *
 * Automatically extracts constraints from [MultipleConstraint] and removes duplicates as defined in [SingleConstraint]
 * If there are no VelocityConstraints or AccelConstraints, Fallback constraints will be used that have a flat maximum
 * of a 100,000 so that the algorithm doesn't die. This is usually not ideal, so put at least one constraint.
 */
class MotionConstraintSet
private constructor(
    allConstraints: Set<SingleConstraint>
) : Set<SingleConstraint> by allConstraints {

    /** This set's velocity constraints */
    val velocityConstraints: Set<VelocityConstraint> =
        allConstraints.filterIsInstanceTo(hashSetOf<VelocityConstraint>()).asUnmodifiableSet()

    /** This set's acceleration constraints */
    val accelerationConstraints: Set<AccelerationConstraint> =
        allConstraints.filterIsInstanceTo(hashSetOf<AccelerationConstraint>()).asUnmodifiableSet()

    constructor(constraints: Collection<MotionConstraint>) : this(
        sequence<SingleConstraint> {
            constraints.forEach {
                when (it) {
                    is SingleConstraint -> yield(it)
                    is MultipleConstraint -> {
                        yieldAll(it.velocityConstraints)
                        yieldAll(it.accelerationConstraints)
                    }
                }
            }
        }.toList().removeRedundantToSet()
    )

    constructor(vararg constraints: MotionConstraint) : this(constraints.asList())

    /**
     * Generates a trajectory using the current constraints, the given [path], and motion profile generation [params].
     */
    fun generateTrajectory(path: Path, params: MotionProfileGenParams): Trajectory =
        generateTrajectory(path, this, params)

    companion object {
        private fun List<SingleConstraint>.removeRedundantToSet(): Set<SingleConstraint> {
            val set = toHashSet()
            this.forEach { cur ->
                set.removeIf { cur !== it && cur.otherIsRedundant(it) }
            }
            return set
        }
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

    private val velConstraints = motionConstraintSet.velocityConstraints
    private val accelConstrains = motionConstraintSet.accelerationConstraints

    override val requiredPoints: Set<Double> get() = path.stopPoints

    private fun getMaxVel(point: PathPoint): Double =
        velConstraints.fold(Double.MAX_VALUE) { vel, it ->
            min(vel, it.maxVelocity(point))
        }

    private fun getMaxAccel(point: PathPoint, curVelocity: Double): Interval =
        accelConstrains.fold(Interval.REAL) { accel, it ->
            accel.intersect(it.accelRange(point, curVelocity))
        }

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
    val constrainer = TrajectoryConstrainer(path, constraints)
    val profile = generateDynamicProfile(
        constrainer, path.length, params
    )
    return Trajectory(path, profile)
}
