package org.futurerobotics.temporaryname.pathing.constraint

import org.futurerobotics.temporaryname.pathing.trajectory.TrajectoryConstraint


/**
 * A collection of both [VelocityConstraint]s and [AccelConstraint], used to construct a [TrajectoryConstraint].
 * Automatically extracts constraints from [MultipleConstraint]
 * If there are no VelocityConstraints or AccelConstraints, Fallback constraints will be used that have a flat maximum
 * of a 100,000. This is usually not ideal, so put at least one constraint.
 */
class MotionConstraintSet private constructor(
    velocityConstraints: List<VelocityConstraint>, accelConstraints: List<AccelConstraint>
) {
    /** The [VelocityConstraint]s */
    val velocityConstraints: List<VelocityConstraint> =
        velocityConstraints.takeIf { it.isNotEmpty() }?.toList() ?: FALLBACK_VELOCITY_CONSTRAINTS
    /** A sequence of the [AccelConstraint]s */
    val accelConstraints: List<AccelConstraint> =
        accelConstraints.takeIf { it.isNotEmpty() }?.toList() ?: FALLBACK_ACCEL_CONSTRAINTS

    companion object {
        /**
         * Constructs a [MotionConstraintSet] from lists of velocity constraints, accel constraints, and multipleConstraints.
         */
        @JvmOverloads
        @JvmStatic
        fun of(
            velocityConstraints: Collection<VelocityConstraint>,
            accelConstraints: Collection<AccelConstraint>,
            multipleConstraint: Collection<MultipleConstraint> = emptyList()
        ): MotionConstraintSet {
            val velCount = velocityConstraints.size + multipleConstraint.sumBy { it.velocityConstraints.size }
            val accelCount = accelConstraints.size + multipleConstraint.sumBy { it.accelConstraints.size }
            val myVelConstraints = ArrayList<VelocityConstraint>(velCount)
            val myAccelConstraints = ArrayList<AccelConstraint>(accelCount)
            myVelConstraints += velocityConstraints
            myAccelConstraints += accelConstraints
            multipleConstraint.forEach {
                myVelConstraints += it.velocityConstraints
                myAccelConstraints += it.accelConstraints
            }
            return MotionConstraintSet(myVelConstraints, myAccelConstraints)
        }

        /**
         * Constructs a [MotionConstraintSet] from a list of [MotionConstraint]s
         * @param constraints the constraints
         */
        @JvmStatic
        fun of(constraints: List<MotionConstraint>): MotionConstraintSet {
            return getFromIterator(constraints.iterator())
        }

        /**
         * Constructs a [MotionConstraintSet] from the given [MotionConstraint]s
         * @param constraints the constraints
         */
        @JvmStatic
        fun of(vararg constraints: MotionConstraint): MotionConstraintSet {
            return getFromIterator(constraints.iterator())
        }

        private fun getFromIterator(constraints: Iterator<MotionConstraint>): MotionConstraintSet {
            val velConstraints = mutableListOf<VelocityConstraint>()
            val accelConstraints = mutableListOf<AccelConstraint>()
            constraints.forEach {
                when (it) {
                    is VelocityConstraint -> velConstraints += it
                    is AccelConstraint -> accelConstraints += it
                    is MultipleConstraint -> {
                        velConstraints += it.velocityConstraints
                        accelConstraints += it.accelConstraints
                    }
                }
            }
            return MotionConstraintSet(velConstraints, accelConstraints)
        }
    }
}
