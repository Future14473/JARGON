package org.futurerobotics.temporaryname.pathing.constraint

import org.futurerobotics.temporaryname.pathing.trajectory.TrajectoryConstraint

/**
 * A collection of both [VelocityConstraint]s and [AccelConstraint]s, used to construct a [TrajectoryConstraint] when
 * paired with a Path for dynamic motion profile generation.
 *
 * Automatically extracts constraints from [MultipleConstraint] and removes duplicates as defined in [SingleConstraint]
 * If there are no VelocityConstraints or AccelConstraints, Fallback constraints will be used that have a flat maximum
 * of a 100,000. This is usually not ideal, so put at least one constraint.
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
        _velocityConstraints = velocityConstraints.removeDupes()
            .takeIf { it.isNotEmpty() } ?: FALLBACK_VELOCITY_CONSTRAINTS

        _accelConstraints = accelConstraints.removeDupes()
            .takeIf { it.isNotEmpty() } ?: FALLBACK_ACCEL_CONSTRAINTS
    }

    private fun <T : SingleConstraint> Iterable<T>.removeDupes(): MutableList<T> {
        val newConstraints = toMutableList()
        forEach { cur ->
            newConstraints.removeIf { it !== cur && it.compareConstraints(cur) < 0 }
        }
        return newConstraints
    }

    constructor(
        velocityConstraints: Iterable<VelocityConstraint>,
        accelConstraints: Iterable<AccelConstraint>,
        multipleConstraints: Iterable<MultipleConstraint> = emptyList()
    ) : this(velocityConstraints + multipleConstraints.flatMap { it.velocityConstraints },
        accelConstraints + multipleConstraints.flatMap { it.accelConstraints })

    constructor(vararg constraints: MotionConstraint) : this(
        constraints.filterIsInstance<VelocityConstraint>(),
        constraints.filterIsInstance<AccelConstraint>(),
        constraints.filterIsInstance<MultipleConstraint>()
    )

    constructor(constraints: Iterable<MotionConstraint>) : this(
        constraints.filterIsInstance<VelocityConstraint>(),
        constraints.filterIsInstance<AccelConstraint>(),
        constraints.filterIsInstance<MultipleConstraint>()
    )
}
