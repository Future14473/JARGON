package org.futurerobotics.jargon.simulation

import org.futurerobotics.jargon.blocks.*
import org.futurerobotics.jargon.blocks.control.FeedForwardController
import org.futurerobotics.jargon.blocks.control.FixedDriveMotorToBotDelta
import org.futurerobotics.jargon.blocks.control.PIDCoefficients
import org.futurerobotics.jargon.blocks.control.PosePIDController
import org.futurerobotics.jargon.blocks.functional.CreateMotionState
import org.futurerobotics.jargon.blocks.functional.ExternalQueue
import org.futurerobotics.jargon.blocks.functional.SplitMotionOnly
import org.futurerobotics.jargon.blocks.motion.GlobalPoseTrackerFromDelta
import org.futurerobotics.jargon.blocks.motion.GlobalToBotMotionReference
import org.futurerobotics.jargon.blocks.motion.TimeOnlyMotionProfileFollower
import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.math.*
import org.futurerobotics.jargon.math.function.QuinticSpline
import org.futurerobotics.jargon.mechanics.*
import org.futurerobotics.jargon.pathing.MultiplePath
import org.futurerobotics.jargon.pathing.OffsetTangentHeading
import org.futurerobotics.jargon.pathing.addHeading
import org.futurerobotics.jargon.pathing.reparam.reparamByIntegration
import org.futurerobotics.jargon.pathing.trajectory.*
import org.futurerobotics.jargon.saveGraph
import org.futurerobotics.jargon.statespace.*
import org.futurerobotics.jargon.system.*
import org.futurerobotics.jargon.util.after
import org.futurerobotics.jargon.util.stepToAll
import org.junit.jupiter.api.Test
import org.knowm.xchart.XYChartBuilder
import org.knowm.xchart.style.markers.None
import java.util.*
import kotlin.collections.ArrayList
import kotlin.math.max
import kotlin.math.roundToLong
import kotlin.random.asKotlinRandom

internal class ASimulation {
    private val trajectories = ExternalQueue<Trajectory>()
    private val driveModel: FixedDriveModel = DriveModels.mecanumLike(
        5 * lbs, 10.0, TransmissionModel.fromTorqueLosses(
            DcMotorModel.fromMotorData(
                12 * volts, 350 * ozf * `in`, 11.5 * A, 160 * rev / min, 0.5 * A
            ),
            1.0, 1.0, 0.9
        ), 2 * `in`, 8 * `in`, 8 * `in`
    )
    private val actualDrive: SimulatedFixedDrive
    private val estimatedPose: Monitor<Pose2d>
    private val toGraph = ArrayList<Pair<String, Monitor<Double>>>()

    private val system: BlocksSystem
    private val constraints = MotionConstraintSet(
        MaxVelocityConstraint(0.5),
        MaxAngularVelocityConstraint(0.6),
        MaxTotalAccelConstraint(0.6),
        MaxAngularAccelConstraint(0.6)
    )
    private val period = 1.0 / 20

    init {

        val continuous = LinearDriveModels.poseVelocityController(driveModel)
        val kGain = continuousLQR(continuous, QRCost(eye(3) * 3.0, eye(4)))

        val ssModel: DiscreteLinSSModel = continuous.discretize(period)
        actualDrive =
            SimulatedFixedDrive(
                driveModel,
                FixedDriveModelPerturber(
                    0.005, 0.005, FixedWheelModelPerturb(
                        0.0005, 0.0, 0.005, TransmissionModelPerturb(
                            0.001, 0.1, 0.001, DcMotorModelPerturb(0.0001)
                        )
                    )
                ),
                Random("a simulation".hashCode().toLong()),
                eye(4) * 0.02,
                eye(4) * 0.05,
                0.005
            )
        val coeff = PIDCoefficients(
            2.0, 0.0, 0.0,
            errorBounds = Interval.symmetric(0.5)
        )
        system = buildBlocksSystem {
            val queue = trajectories
            val follower =
                TimeOnlyMotionProfileFollower<MotionState<Pose2d>>(ValueMotionState.ofAll(Pose2d.ZERO)).apply {
                    profileInput from queue
                }

            val positionController =
                FeedForwardController.withAdder(PosePIDController(coeff, coeff, coeff), Pose2d::plus).apply {
                    reference from follower.output
                }
            toGraph += "X reference pos" to follower.output.pipe { s.x }.monitor()
            toGraph += "Y reference pos" to follower.output.pipe { s.y }.monitor()
//            toGraph += "X reference vel" to follower.output.pipe { v.x }.monitor()
//            toGraph += "Y reference vel" to follower.output.pipe { v.y }.monitor()
//            toGraph += "signals reference y" to follower.output.pipe { v.y }.monitor()
//            toGraph += "signals velocity signal x" to positionController.pipe { v.x }.monitor()
//            toGraph += "signals velocity signal y" to positionController.pipe { v.y }.monitor()

            val botMotion = GlobalToBotMotionReference().apply { reference from positionController }
            toGraph += "X vel signal" to botMotion.pipe { v.x }.monitor()
            toGraph += "Y vel signal" to botMotion.pipe { v.y }.monitor()
            val (refVPose, refAPose) = SplitMotionOnly<Pose2d>().apply { this from botMotion }
            val ref = CreateMotionState<Vec>().apply {
                value from refVPose.pipe { toVector() }
                vel from refAPose.pipe { toVector() }
                accel from Constant(zeroVec(3))
            }
//            toGraph += "signals vec vel signal x" to ref.pipe { s[0] }.monitor()
//            toGraph += "signals vec vel signal y" to ref.pipe { s[1] }.monitor()
            val ssController = SSControllerWithFF(ssModel, kGain).apply {
                reference from ref
            }


            val simulated = SimulatedDriveInput(actualDrive).apply {
                this from ssController.pipe { asList() }
                    .also {
                        repeat(4) { i ->
                            toGraph += "signals voltage signal $i" to it.pipe { this[i] }.monitor()
                        }
                    }
                repeat(4) { i ->
                    toGraph += "signals real wheel vel $i" to motorVel.pipe { this[i] }.monitor()
                }
            }

            KalmanFilter(ssModel, eye(3) * 0.03, eye(4) * 0.05).apply {
                measurement from simulated.motorVel.pipe { toVec() }
                signal from ssController.delay(zeroVec(4))
                ssController.state from this
            }
//                simulated.actualPose.monitor()
            val tracker = GlobalPoseTrackerFromDelta().apply {
                deltaIn from simulated.motorPos.pipe(FixedDriveMotorToBotDelta(driveModel))
                this into positionController.state
                this into botMotion.globalPose
                estimatedPose = this.monitor()


                toGraph += "X measured pos" to estimatedPose.source()!!.pipe { x }.monitor()
                toGraph += "Y measured pos" to estimatedPose.source()!!.pipe { y }.monitor()

            }
            val error = tracker.combine(follower.output.pipe { s }) {
                max(this.vec distTo it.vec, (this.heading distTo it.heading))
            }
            Shutdown() from follower.isFollowing.combine(error) { !this && it < 0.03 }

//            simulated.actualPose.let {
//                positionController.state from it
//                botMotion.globalPose from it
//            }
        }
    }

    private val systemTracker = object : LoopSystem {
        val actualPositions = ArrayList<Vector2d>()
        val predictedPositions = ArrayList<Vector2d>()
        val toGraphValues = List(toGraph.size) { ArrayList<Double>() }
        override fun loop(loopTimeInNanos: Long): Boolean = false.after {
            actualPositions += actualDrive.curGlobalPose.vec
            predictedPositions += estimatedPose.value!!.vec
            toGraph.forEachIndexed { i, (_, it) ->
                toGraphValues[i] += it.value!!
            }
        }

        override fun init() {
        }

        override fun stop() {
            actualPositions.trimToSize()
            predictedPositions.trimToSize()
        }

    }

    @Test
    fun `simulation 1`() {
        val random = Random("The first simulation".hashCode().toLong()).asKotlinRandom()
        val segs =
            (listOf(ValueDerivatives(Vector2d.ZERO, Vector2d(1, 0), Vector2d.ZERO)) +
                    List(4) {
                        randomVectorDerivatives(random, 5.0)
                    }).zipWithNext { a, b ->
                QuinticSpline.fromDerivatives(a, b).reparamByIntegration().addHeading(OffsetTangentHeading(74 * deg))
            }
        val path = MultiplePath(segs)
        val traj = generateTrajectory(path, constraints)
        trajectories.add(traj)
        val systems = CompositeLoopSystem(system, systemTracker)
        val driver = LimitedLoopSystemDriver(4_000, LoopAsFastAsPossible(FixedTestClock((1e9 * period).roundToLong())))

        driver.run(systems)

        XYChartBuilder().apply {
            title = "Some simulation"
        }.build().apply {
            styler.apply {
                //                xAxisMin = -1.0
//                xAxisMax = 5.0
//                yAxisMin = -3.0
//                yAxisMax = 3.0
            }


            val div = DoubleProgression.fromClosedRange(0.0, traj.duration, 0.05)
            val trajPoints = traj.stepToAll(div).map { it.s.vec }
            val stepper = traj.stepper()
            addSeries("Trajectory", trajPoints.map { it.x }, trajPoints.map { it.y }).apply {
                marker = None()
            }
            addSeries(
                "Actual positions",
                systemTracker.actualPositions.map { it.x },
                systemTracker.actualPositions.map { it.y }
            ).apply { marker = None() }
            addSeries("Predicted positions",
                systemTracker.predictedPositions.map { it.x },
                systemTracker.predictedPositions.map { it.y }
            ).apply { marker = None() }
        }.saveGraph("Path")
        toGraph.map { it.first }.zip(systemTracker.toGraphValues)
            .groupBy { (it) -> it.substring(0, it.indexOf(' ')) }
            .forEach { (groupName, list) ->
                XYChartBuilder().apply {
                    title = groupName
                }.build().apply {
                    list.forEach { (name, values) ->
                        addSeries(name.substring(name.indexOf(' ')), values).apply {
                            marker = None()
                        }
                    }
                }.saveGraph(groupName)
            }
    }
}
