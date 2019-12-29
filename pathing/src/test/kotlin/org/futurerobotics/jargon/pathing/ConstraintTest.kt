package org.futurerobotics.jargon.pathing

import org.futurerobotics.jargon.Debug
import org.futurerobotics.jargon.math.DoubleProgression
import org.futurerobotics.jargon.math.distTo
import org.futurerobotics.jargon.math.function.QuinticSpline
import org.futurerobotics.jargon.math.randomVectorDerivatives
import org.futurerobotics.jargon.pathing.reparam.reparameterizeToCurve
import org.futurerobotics.jargon.pathing.trajectory.*
import org.futurerobotics.jargon.util.extendingDoubleSearch
import org.futurerobotics.jargon.util.stepToAll
import org.junit.Test
import kotlin.random.Random

internal class ConstraintTest {
    private val range = 12.0
    private val random = Random(2349)
    private val path = List(10) {
        randomVectorDerivatives(
            random,
            range
        )
    }.zipWithNext { start, end ->
        QuinticSpline.fromDerivatives(start, end).reparameterizeToCurve().addHeading(TangentHeading)
    }.let { multiplePath(it) }
    private val motionConstraints = MotionConstraintSet(
        MaxTangentVelocity(10.0),
        MaxTotalAcceleration(1.0),
        MaxAngularAcceleration(0.2)
    )
    private val constraint = TrajectoryConstrainer(path, motionConstraints)
    private val steps = 1_000
    @Test
    fun `Something Fishy About Angular Accel Constraint`() {
        val points = DoubleProgression.fromNumSegments(0.0, path.length, steps).toList()
        val pointConstraints = constraint.stepToAll(points)
        val vels = DoubleProgression.fromNumSegments(0.0, 5.0, steps)
        val step = vels.step
        repeat(steps + 1) { i ->
            Debug.breakIf(i == 16)
            val pointConstraint = pointConstraints[i]
            var wasValid = true
            var partitionPoint = 5.0
            vels.forEach { v ->
                val valid = !pointConstraint.accelRange(v).isEmpty()
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
                pointConstraint.accelRange(it).isEmpty()
            }
            assert(partitionPointBySearch distTo partitionPoint <= 2 * step) {
                "at $i: partitionPoint $partitionPoint, search got $partitionPointBySearch"
            }
        }
    }
}
