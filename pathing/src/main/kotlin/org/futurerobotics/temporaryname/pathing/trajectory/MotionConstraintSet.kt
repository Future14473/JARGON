package org.futurerobotics.temporaryname.pathing.trajectory

import org.futurerobotics.temporaryname.pathing.Path

/**
 * A collection of [VelocityConstraint]s, [AccelConstraint]s, and (flattened) [MultipleConstraint] used to construct
 * a [TrajectoryConstraint] when paired with a Path for dynamic motion profile generation.
 *
 * This class is here because it can be reused when creating trajectories; however since the path may change
 * the constraint cannot be reused.
 *
 * Automatically extracts constraints from [MultipleConstraint] and removes duplicates as defined in [SingleConstraint]
 * If there are no VelocityConstraints or AccelConstraints, Fallback constraints will be used that have a flat maximum
 * of a 100,000 so that the algorithm doesn't die. This is usually not ideal, so put at least one constraint.
 */
class MotionConstraintSet(
    velocityConstraints: Iterable<VelocityConstraint>, accelConstraints: Iterable<AccelConstraint>
) {

    private val _velocityConstraints: List<VelocityConstraint>
    private val _accelConstraints: List<AccelConstraint>

    /** This set's velocity constraints */
    val velocityConstraints: List<VelocityConstraint> get() = _velocityConstraints
    /** This set's acceleration constraints */
    val accelConstraints: List<AccelConstraint> get() = _accelConstraints

    init {
        //        val velocityConstraints = velocityConstraints + multipleConstraints.flatMap { it.velocityConstraints }
        //        val accelConstraints = accelConstraints + multipleConstraints.flatMap { it.accelConstraints }
        _velocityConstraints = velocityConstraints.removeRedundant()
            .takeIf { it.isNotEmpty() } ?: FALLBACK_VELOCITY_CONSTRAINTS

        _accelConstraints = accelConstraints.removeRedundant()
            .takeIf { it.isNotEmpty() } ?: FALLBACK_ACCEL_CONSTRAINTS
    }

    constructor(
        velocityConstraints: Iterable<VelocityConstraint>,
        accelConstraints: Iterable<AccelConstraint>,
        multipleConstraints: Iterable<MultipleConstraint> = emptyList()
    ) : this(velocityConstraints + multipleConstraints.flatMap { it.velocityConstraints },
        accelConstraints + multipleConstraints.flatMap { it.accelConstraints })


    constructor(constraints: Iterable<MotionConstraint>) : this(
        constraints.filterIsInstance<VelocityConstraint>(),
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

    private fun <T : SingleConstraint> Iterable<T>.removeRedundant(): MutableList<T> {
        val newConstraints = toMutableList()
        forEach { cur ->
            newConstraints.removeIf { it !== cur && cur.otherIsRedundant(it) }
        }
        return newConstraints
    }


}
