package org.futurerobotics.temporaryname.pathing

import org.futurerobotics.temporaryname.Debug
import org.futurerobotics.temporaryname.math.DoubleProgression
import org.futurerobotics.temporaryname.math.distTo
import org.futurerobotics.temporaryname.math.extendingDoubleSearch
import org.futurerobotics.temporaryname.math.function.QuinticSpline
import org.futurerobotics.temporaryname.math.randomVectorDerivatives
import org.futurerobotics.temporaryname.pathing.constraint.MotionConstraintSet
import org.futurerobotics.temporaryname.pathing.constraint.RobotAngularAccelConstraint
import org.futurerobotics.temporaryname.pathing.constraint.TangentVelocityConstraint
import org.futurerobotics.temporaryname.pathing.constraint.TotalAccelConstraint
import org.futurerobotics.temporaryname.pathing.path.Path
import org.futurerobotics.temporaryname.pathing.path.TangentHeading
import org.futurerobotics.temporaryname.pathing.path.addHeading
import org.futurerobotics.temporaryname.pathing.path.reparam.reparamByIntegration
import org.futurerobotics.temporaryname.pathing.trajectory.TrajectoryConstraint
import org.junit.Test
import kotlin.random.Random

internal class ConstraintTest {
    private val range = 12.0
    private val random = Random(2349)
    private val path = List(10) { randomVectorDerivatives(random, range) }.zipWithNext { start, end ->
        QuinticSpline.fromDerivatives(start, end).reparamByIntegration().addHeading(TangentHeading)
    }
        .let { Path(it) }
    private val motionConstraints = MotionConstraintSet.of(
        TangentVelocityConstraint(10.0),
        TotalAccelConstraint(1.0),
        RobotAngularAccelConstraint(0.2)
    )
    private val constraint = TrajectoryConstraint(path, motionConstraints)
    private val steps = 1_000

    @Test
    fun `Something Fishy About Angular Accel Constraint`() {
        val points = DoubleProgression.fromNumSegments(0.0, path.length, steps).toList()
        val (_, accels) = constraint.getAllVelsAndAccels(points)
        val vels = DoubleProgression.fromNumSegments(0.0, 5.0, steps)
        val step = vels.step
        repeat(steps + 1) { i ->
            Debug.breakIf(i == 16)
            val getter = accels[i]
            var wasValid = true
            var partitionPoint = 5.0
            vels.forEach { v ->
                val valid = !getter.getMaxAccel(v, false).isNaN()
                if (!wasValid && valid) {
                    println("ISSUES")
                    Debug.breakpoint()
                }
                if (wasValid && !valid) {
                    partitionPoint = v
                }
                wasValid = valid
            }
            val partitionPointBySearch = extendingDoubleSearch(0.0, 5.0, step) {
                getter.getMaxAccel(it, false).isNaN()
            }
            assert(partitionPointBySearch distTo partitionPoint <= 2 * step) {
                "at $i: partitionPoint $partitionPoint, search got $partitionPointBySearch"
            }
        }
    }
}