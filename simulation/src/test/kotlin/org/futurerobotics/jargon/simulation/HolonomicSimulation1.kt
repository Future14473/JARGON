package org.futurerobotics.jargon.simulation

import org.futurerobotics.jargon.blocks.control.PIDCoefficients
import org.futurerobotics.jargon.blocks.control.PosePIDController
import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.math.Interval
import org.futurerobotics.jargon.math.ValueDerivatives
import org.futurerobotics.jargon.math.Vector2d
import org.futurerobotics.jargon.math.convert.*
import org.futurerobotics.jargon.math.function.QuinticSpline
import org.futurerobotics.jargon.math.randomVectorDerivatives
import org.futurerobotics.jargon.pathing.*
import org.futurerobotics.jargon.pathing.reparam.reparamByIntegration
import org.futurerobotics.jargon.pathing.trajectory.*
import org.futurerobotics.jargon.saveGraph
import org.futurerobotics.jargon.statespace.QRCost
import org.futurerobotics.jargon.system.looping.FixedTestClock
import org.futurerobotics.jargon.system.looping.LoopAsFastAsPossible
import org.futurerobotics.jargon.system.looping.LoopMaxTimes
import org.futurerobotics.jargon.system.looping.LoopSystemRunner
import org.junit.jupiter.api.Test
import java.util.*
import kotlin.math.roundToLong
import kotlin.random.asKotlinRandom

internal fun randomTrajectory(
    random: kotlin.random.Random,
    constraints: MotionConstraintSet
): Trajectory {
    val segs =
        (listOf(ValueDerivatives(Vector2d.ZERO, Vector2d.polar(1.0, -74 * deg), Vector2d.ZERO)) +
                List(4) {
                    randomVectorDerivatives(random, 5.0)
                }).zipWithNext { a, b ->
            QuinticSpline.fromDerivatives(a, b).reparamByIntegration().addHeading(OffsetTangentHeading(74 * deg))
        }
    val path = MultiplePath(segs)
    return constraints.generateTrajectory(path)
}

internal class HolonomicSimulation1 : PoseVelocityControllingSimulation(
    SomeModels.mecanum,
    SimulatedFixedDrive(
        SomeModels.mecanum,
        FixedWheelDriveModelPerturb(
            0.005, 0.005, FixedWheelModelPerturb(
                0.0005, 0.00001, 0.005, TransmissionModelPerturb(
                    0.005, 0.1, 0.005, DcMotorModelPerturb(0.001)
                )
            )
        ),
        Random("Holonomic Simulation 1".hashCode().toLong()),
        idenMat(4) * 0.05,
        idenMat(4) * 0.05,
        0.005
    ),
    1.0 / 20,
    PosePIDController(coeff, coeff, headingCoeff),
    QRCost(idenMat(3) * 4.0, idenMat(4)),
    idenMat(3) * 0.05,
    idenMat(4) * 0.05
) {

    private val constraints1 = MotionConstraintSet(
        MaxVelConstraint(0.5),
        MaxAngularVelConstraint(0.6),
        MaxTotalAccelConstraint(0.6),
        MaxAngularAccelConstraint(0.6)
    )

    private val constraints2 = MotionConstraintSet(
        MaxMotorVoltage(driveModel, 8.0),
        MaxMotorTorque(driveModel, 250 * ozf * `in`),
        MaxVelConstraint(1.0),
        MaxAngularVelConstraint(1.0),
        MaxTotalAccelConstraint(1.0),
        MaxAngularAccelConstraint(1.0)
    )

    @Test
    fun simulation1() {
        val path = Line(Vector2d.ZERO, Vector2d(2, 0)).addHeading(TangentHeading)
        val trajectory = constraints1.generateTrajectory(path)
        trajectories.add(trajectory)

        val runner = LoopSystemRunner(
            system, LoopMaxTimes(5000, LoopAsFastAsPossible(FixedTestClock((1e9 * period).roundToLong())))
        )
        runner.run()

        recordings.getAllGraphs().forEach { (name, graph) ->
            graph.saveGraph("${this.javaClass.simpleName}/simulation1/$name", 300)
        }
    }

    @Test
    fun simulation2() {
        randomPaths(Random("Don't get unlucky".hashCode().toLong()).asKotlinRandom(), constraints1)
    }

    @Test
    fun simulation3() {
        randomPaths(Random("It's not working!!!".hashCode().toLong()).asKotlinRandom(), constraints2)
    }

    private fun randomPaths(
        random: kotlin.random.Random, constraints: MotionConstraintSet
    ) {
        val trajectory = randomTrajectory(random, constraints)
        trajectories.add(trajectory)

        val driver = LoopSystemRunner(
            system, LoopMaxTimes(5000, LoopAsFastAsPossible(FixedTestClock((1e9 * period).roundToLong())))
        )
        driver.run()
        val method = Thread.currentThread().stackTrace[2].methodName
        recordings.getAllGraphs().forEach { (name, graph) ->
            graph.saveGraph("${this.javaClass.simpleName}/$method/$name", 300)
        }
    }

    companion object {
        val coeff = PIDCoefficients(
            3.0, 0.0, 0.001,
            errorBounds = Interval.symmetric(0.5)
        )

        val headingCoeff = PIDCoefficients(
            1.0, 0.0, 0.001,
            errorBounds = Interval.symmetric(0.5)
        )
    }
}

