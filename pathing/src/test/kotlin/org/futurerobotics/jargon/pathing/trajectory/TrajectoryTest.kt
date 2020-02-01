package org.futurerobotics.jargon.pathing.trajectory

import org.futurerobotics.jargon.Debug
import org.futurerobotics.jargon.errorTo
import org.futurerobotics.jargon.math.MotionState
import org.futurerobotics.jargon.math.Pose2d
import org.futurerobotics.jargon.pathing.*
import org.futurerobotics.jargon.profile.MotionProfileGenParams
import org.futurerobotics.jargon.reportError
import org.junit.Assert
import org.junit.Test
import org.junit.runner.RunWith
import org.junit.runners.Parameterized
import kotlin.random.Random

@RunWith(Parameterized::class)
class TrajectoryTest(private val trajectory: Trajectory) {

    @Test
    fun `time deriv inspect`() {
        val duration = trajectory.duration
        reportError {
            stepT { i, t ->
                val time = t * duration
                val direct: MotionState<Pose2d> = trajectory.atTime(time)
                val approx =
                    (trajectory.atTime(time + epsilon).value - trajectory.atTime(time - epsilon).value) / (2 * epsilon)
                addError(approx errorTo direct.deriv) {
                    "at $i, approx deriv was $approx, returned was ${direct.deriv}"
                }
                //                addError((deriv.poseDeriv errorTo getDirect.poseSecondDeriv)) {
                //                    "at $i, approx second deriv was ${deriv.poseDeriv}, returned was ${getDirect.poseSecondDeriv}"
                //                }
            }
        }.also {
            println(it.report())
            val errorOk = it.averageError < maxError
            Debug.breakIf(!errorOk)
            Assert.assertTrue(errorOk)
        }
    }

    private inline fun stepT(block: (Int, Double) -> Unit) {
        for (i in 1 until steps) {
            val t = i.toDouble() / steps
            block(i, t)
        }
    }

    companion object {
        private const val epsilon = 1e-9
        private const val steps = 10_000
        private const val range = 12.0
        private const val maxError = 0.01
        private val random = Random(234828)
        private val constraints = MotionConstraintSet(
            MaxTangentVelocity(2.0),
            MaxPathAngularVelocity(1.5),
            MaxCentripetalAccel(0.9),
            MaxTangentAcceleration(0.9),
            MaxTotalAcceleration(1.0),
            MaxAngularAcceleration(1.0)
        )

        @JvmStatic
        @Parameterized.Parameters
        fun trajectories(): Iterable<Array<Trajectory>> = List(20) {
            generateSequence {
                randomVectorDerivatives(
                    random,
                    range
                )
            }.take(6).zipWithNext { a, b ->
                QuinticSpline.fromDerivatives(a, b)
                    .reparameterizeToCurve()
                    .addHeading(TangentHeading)
            }.let {
                multiplePath(it.toList())
            }.let {
                constraints.generateTrajectory(it, MotionProfileGenParams())
            }.let {
                arrayOf(it)
            }
        }
    }
}
