package org.futurerobotics.jargon.simulation

import org.futurerobotics.jargon.blocks.control.PidCoefficients
import org.futurerobotics.jargon.blocks.control.PosePidController
import org.futurerobotics.jargon.interruptAfter
import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.math.Interval
import org.futurerobotics.jargon.math.Vector2d
import org.futurerobotics.jargon.math.convert.*
import org.futurerobotics.jargon.pathing.Line
import org.futurerobotics.jargon.pathing.TangentHeading
import org.futurerobotics.jargon.pathing.addHeading
import org.futurerobotics.jargon.pathing.trajectory.*
import org.futurerobotics.jargon.profile.MotionProfileGenParams
import org.futurerobotics.jargon.running.FixedTestClock
import org.futurerobotics.jargon.running.LoopSystemRunner
import org.futurerobotics.jargon.running.UnregulatedRegulator
import org.futurerobotics.jargon.saveGraph
import org.futurerobotics.jargon.statespace.NoiseCovariance
import org.futurerobotics.jargon.statespace.QRCost
import org.junit.jupiter.api.Test
import java.util.*
import java.util.concurrent.TimeUnit
import kotlin.math.roundToLong
import kotlin.random.asKotlinRandom

internal class HolonomicSimulation1 : PoseVelocityControllingSimulation(
    SomeModels.mecanum,
    SimulatedFixedDrive(
        SomeModels.mecanum,
        Random("Holonomic Simulation 1".hashCode().toLong()),
        idenMat(4) * 0.05,
        idenMat(4) * 0.05,
        0.005
    ),
    1.0 / 20,
    PosePidController(coeff, coeff, headingCoeff),
    QRCost(idenMat(3) * 2.1, idenMat(4)),
    NoiseCovariance(idenMat(3) * 0.05, idenMat(4) * 0.05)
) {

    private val constraints1 = MotionConstraintSet(
        MaxVelConstraint(0.5),
        MaxAngularVelConstraint(0.6),
        MaxTotalAccelConstraint(0.6),
        MaxAngularAccelConstraint(0.6)
    )

    private val constraints2 = MotionConstraintSet(
        MaxMotorVoltage(driveModel, driveModel, 8.0),
        MaxMotorTorque(driveModel.transmissions.map { it.motor }, driveModel, driveModel, 250 * ozf * `in`),
        MaxVelConstraint(1.0),
        MaxAngularVelConstraint(1.0),
        MaxTotalAccelConstraint(1.0),
        MaxAngularAccelConstraint(1.0)
    )

    @Test
    fun simulation1() {
        val path = Line(Vector2d.ZERO, Vector2d(2, 0)).addHeading(TangentHeading)
        val trajectory = generateTrajectory(path, constraints1, MotionProfileGenParams())
        trajectories.add(trajectory)

        val runner = LoopSystemRunner(
            system,
            UnregulatedRegulator(
                FixedTestClock((1e9 * period).roundToLong())
            )
        )
        interruptAfter(5, TimeUnit.SECONDS) {
            runner.run()
        }


        recordings.getAllGraphs().forEach { (name, graph) ->
            graph.saveGraph("${this.javaClass.simpleName}/simulation1/$name", 200)
        }
    }

    @Test
    fun simulation2() {
        randomPaths(Random("Don't get unlucky".hashCode().toLong()).asKotlinRandom(), constraints1)
    }

    @Test
    fun simulation3() {
        randomPaths(Random("It's not working!!".hashCode().toLong()).asKotlinRandom(), constraints2)
    }

    private fun randomPaths(
        random: kotlin.random.Random, constraints: MotionConstraintSet
    ) {
        val trajectory = randomTrajectory(random, constraints)
        trajectories.add(trajectory)

        val runner = LoopSystemRunner(
            system,
            UnregulatedRegulator(
                FixedTestClock((1e9 * period).roundToLong())
            )
        )
        interruptAfter(5, TimeUnit.SECONDS) {
            runner.run()
        }

        val method = Thread.currentThread().stackTrace[2].methodName
        recordings.getAllGraphs().forEach { (name, graph) ->
            graph.saveGraph("${this.javaClass.simpleName}/$method/$name", 200)
        }
    }

    companion object {
        val coeff = PidCoefficients(
            3.0, 0.0, 0.001,
            errorBounds = Interval.symmetric(0.5)
        )

        val headingCoeff = PidCoefficients(
            1.0, 0.0, 0.001,
            errorBounds = Interval.symmetric(0.5)
        )
    }
}

