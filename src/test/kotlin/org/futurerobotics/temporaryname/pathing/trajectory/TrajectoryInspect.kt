package org.futurerobotics.temporaryname.pathing.trajectory

import org.futurerobotics.temporaryname.Debug
import org.futurerobotics.temporaryname.math.errorTo
import org.futurerobotics.temporaryname.math.function.QuinticSpline
import org.futurerobotics.temporaryname.math.randomVectorDerivatives
import org.futurerobotics.temporaryname.pathing.MultiplePath
import org.futurerobotics.temporaryname.pathing.TangentHeading
import org.futurerobotics.temporaryname.pathing.addHeading
import org.futurerobotics.temporaryname.pathing.constraint.*
import org.futurerobotics.temporaryname.pathing.reparam.reparamByIntegration
import org.futurerobotics.temporaryname.reportError
import org.junit.Assert
import org.junit.Test
import org.junit.runner.RunWith
import org.junit.runners.Parameterized
import kotlin.random.Random

@RunWith(Parameterized::class)
class TrajectoryInspect(private val trajectory: Trajectory) {
    @Test
    fun `time deriv inspect`() {
        val duration = trajectory.duration
        reportError {
            stepT { i, t ->
                val time = t * duration
                val direct = trajectory.atTime(time)
                val approx =
                    (trajectory.atTime(time + epsilon).pose - trajectory.atTime(time - epsilon).pose) / (2 * epsilon)
                addError(approx errorTo direct.vel) {
                    "at $i, approx deriv was $approx, returned was ${direct.vel}"
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

    @Test
    fun `dist deriv inspect`() {
        val length = trajectory.length
        reportError {
            stepT { i, t ->
                val dist = t * length
                val direct = trajectory.atDistance(dist)
                val approx =
                    (trajectory.atDistance(dist + epsilon).pose - trajectory.atDistance(dist - epsilon).pose) / //dp/ds
                            (2 * epsilon) * trajectory.profile.atDistance(dist).v //ds/dt
                addError(approx errorTo direct.vel) {
                    "at $i, approx deriv was $approx, returned was ${direct.vel}"
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
        private const val maxError = 0.005
        private val random = Random(23934827)
        private val constraints = MotionConstraintSet(
            TangentVelocityConstraint(2.0),
            PathAngularVelocityConstraint(1.5),
            CentripetalAccelConstraint(0.9),
            TangentAccelConstraint(0.9),
            TotalAccelConstraint(1.0),
            RobotAngularAccelConstraint(1.0)
        )

        @JvmStatic
        @Parameterized.Parameters
        fun trajectories(): List<Array<Trajectory>> {
            return List(30) {
                List(6) {
                    randomVectorDerivatives(random, range)
                }.zipWithNext { a, b ->
                    QuinticSpline.fromDerivatives(a, b)
                        .reparamByIntegration()
                        .addHeading(TangentHeading)
                }.let {
                    MultiplePath(it)
                }.let {
                    TrajectoryGenerator.generateTrajectory(it, constraints)
                }.let {
                    arrayOf(it)
                }
            }
        }
    }
}
