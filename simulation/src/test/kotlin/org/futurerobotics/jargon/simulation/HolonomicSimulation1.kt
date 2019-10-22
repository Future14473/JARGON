package org.futurerobotics.jargon.simulation

import org.futurerobotics.jargon.blocks.BlocksSystem
import org.futurerobotics.jargon.blocks.Shutdown
import org.futurerobotics.jargon.blocks.control.*
import org.futurerobotics.jargon.blocks.functional.ExternalQueue
import org.futurerobotics.jargon.blocks.functional.MapMotionOnly
import org.futurerobotics.jargon.blocks.functional.ShiftMotionOnlyToState
import org.futurerobotics.jargon.blocks.motion.GlobalPoseTrackerFromDeltaAndGyro
import org.futurerobotics.jargon.blocks.motion.GlobalToBotMotion
import org.futurerobotics.jargon.blocks.motion.TimeOnlyMotionProfileFollower
import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.math.*
import org.futurerobotics.jargon.math.function.QuinticSpline
import org.futurerobotics.jargon.mechanics.FixedDriveModel
import org.futurerobotics.jargon.mechanics.MotionState
import org.futurerobotics.jargon.mechanics.ValueMotionState
import org.futurerobotics.jargon.pathing.*
import org.futurerobotics.jargon.pathing.reparam.reparamByIntegration
import org.futurerobotics.jargon.pathing.trajectory.*
import org.futurerobotics.jargon.saveGraph
import org.futurerobotics.jargon.statespace.*
import org.futurerobotics.jargon.system.FixedTestClock
import org.futurerobotics.jargon.system.LimitedLoopSystemDriver
import org.futurerobotics.jargon.system.LoopAsFastAsPossible
import org.junit.jupiter.api.Test
import java.util.*
import kotlin.math.max
import kotlin.math.roundToLong
import kotlin.random.asKotlinRandom

internal abstract class PoseVelocityControllingSimulation(
    protected val driveModel: FixedDriveModel,
    simulatedDrive: SimulatedFixedDrive,
    val period: Double,
    nonFFController: PosePIDController,
    lqrCost: QRCost,
    kfilterQ: Mat,
    kfilterR: Mat
) {
    protected val trajectories: ExternalQueue<Trajectory> = ExternalQueue()
    protected val system: BlocksSystem
    protected val recordings: Recordings

    init {
        val numWheels = driveModel.numWheels
        val motorsBlock = SimulatedDriveBlock(simulatedDrive)
        val gyro = GyroBlock(SimulatedGyro(simulatedDrive))

        val continuous = LinearDriveModels.poseVelocityController(driveModel)
        val kGain = continuousLQR(continuous, lqrCost)

        val ssModel = continuous.discretize(period)


        val (system, recordings) = buildRecordingBlocksSystem {
            val queue = trajectories
            val follower =
                TimeOnlyMotionProfileFollower<MotionState<Pose2d>>(ValueMotionState.ofAll(Pose2d.ZERO)).apply {
                    profileInput from queue
                }

            val positionController =
                FeedForwardController.withAdder(nonFFController, Pose2d::plus).apply {
                    reference from follower.output
                }
            follower.output.pipe { s.x }.recordY("x reference", "reference value")
            follower.output.pipe { s.y }.recordY("y reference", "reference value")
            follower.output.pipe { s.heading }.recordY("heading reference", "reference value")

            follower.output.pipe { v.x }.recordY("x reference", "reference velocity")
            follower.output.pipe { v.y }.recordY("y reference", "reference velocity")
            follower.output.pipe { v.heading }.recordY("heading reference", "reference velocity")

            val botMotion = GlobalToBotMotion().apply { reference from positionController }
            botMotion.pipe { v.x }.recordY("x reference", "velocity signal")
            botMotion.pipe { v.y }.recordY("y reference", "velocity signal")
            botMotion.pipe { v.heading }.recordY("heading reference", "velocity signal")

            val poseVelRef = botMotion
                .pipe(MapMotionOnly.of<Pose2d, Vec> { it.toVector() })
                .pipe(ShiftMotionOnlyToState(zeroVec(3)))

            val ssController = SSControllerWithFF(ssModel, kGain)() { reference from poseVelRef }

            motorsBlock {
                this from ssController.pipe { asList() }
                    .apply {
                        repeat(4) {
                            pipe { this[it] }.recordY("Voltages", "Voltage $it")
                        }
                    }
            }

            KalmanFilter(ssModel, kfilterQ, kfilterR)() {
                measurement from motorsBlock.motorVelocities.pipe { toVec() }
                signal from ssController.delay(zeroVec(numWheels))
                ssController.state from this
            }

            val delta = FixedDriveMotorToBotDelta(driveModel)() {
                this from motorsBlock.motorPositions
            }

            val tracker = GlobalPoseTrackerFromDeltaAndGyro()() {
                deltaIn from delta
                gyroIn from gyro
                this into positionController.state
                this into botMotion.globalPose

                pipe { x }.recordY("x reference", "measured value")
                pipe { y }.recordY("y reference", "measured value")
                pipe { heading }.recordY("heading reference", "measured value")

            }
            val error = tracker.combine(follower.output.pipe { s }) {
                max(this.vec distTo it.vec, (this.heading distTo it.heading))
            }
            Shutdown() from follower.isFollowing.combine(error) { !this && it < 0.1 }

            follower.output.pipe { s.vec }.recordXY("Path", "Reference")
            tracker.pipe { vec }.recordXY("Path", "Estimated pose")
            motorsBlock.actualPose.pipe { vec }.recordXY("Path", "Actual pose")
            motorsBlock.actualPose.pipe { x }.recordY("x reference", "Actual value")
            motorsBlock.actualPose.pipe { y }.recordY("y reference", "Actual value")
//            motorsBlock.actualPose.pipe { heading }.recordY("heading reference", "Actual value")
            motorsBlock.actualPose.listen { }
        }
        this.system = system
        this.recordings = recordings
    }
}

internal class HolonomicSimulation1 : PoseVelocityControllingSimulation(
    SomeModels.mecanum,
    SimulatedFixedDrive(
        SomeModels.mecanum,
        FixedDriveModelPerturber(
            0.005, 0.005, FixedWheelModelPerturb(
                0.0005, 0.00001, 0.005, TransmissionModelPerturb(
                    0.005, 0.1, 0.005, DcMotorModelPerturb(0.001)
                )
            )
        ),
        Random("Holonomic Simulation 1".hashCode().toLong()),
        eye(4) * 0.05,
        eye(4) * 0.05,
        0.005
    ),
    1.0 / 20,
    PosePIDController(coeff, coeff, headingCoeff),
    QRCost(eye(3) * 3.0, eye(4)),
    eye(3) * 0.05,
    eye(4) * 0.05
) {
    private val constraints1 = MotionConstraintSet(
        MaxVelocityConstraint(0.5),
        MaxAngularVelocityConstraint(0.6),
        MaxTotalAccelConstraint(0.6),
        MaxAngularAccelConstraint(0.6)
    )

    private val constraints2 = MotionConstraintSet(
        MaxMotorVoltage(driveModel, 10.0),
        MaxWheelForce(driveModel, 50.0)
    )

    @Test
    fun simulation1() {
        val path = Line(Vector2d.ZERO, Vector2d(2, 0)).addHeading(TangentHeading)
        val trajectory = constraints1.generateTrajectory(path)
        trajectories.add(trajectory)

        val driver = LimitedLoopSystemDriver(5_000, LoopAsFastAsPossible(FixedTestClock((1e9 * period).roundToLong())))
        driver.run(system)

        recordings.getAllGraphs().forEach { (name, graph) ->
            graph.saveGraph(name, 300)
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
        random: kotlin.random.Random,
        constraints1: MotionConstraintSet
    ) {
        val segs =
            (listOf(ValueDerivatives(Vector2d.ZERO, Vector2d(1, 0), Vector2d.ZERO)) +
                    List(4) {
                        randomVectorDerivatives(random, 5.0)
                    }).zipWithNext { a, b ->
                QuinticSpline.fromDerivatives(a, b).reparamByIntegration().addHeading(OffsetTangentHeading(74 * deg))
            }
        val path = MultiplePath(segs)
        val trajectory = constraints1.generateTrajectory(path)
        trajectories.add(trajectory)

        val driver = LimitedLoopSystemDriver(5_000, LoopAsFastAsPossible(FixedTestClock((1e9 * period).roundToLong())))
        driver.run(system)

        recordings.getAllGraphs().forEach { (name, graph) ->
            graph.saveGraph(name, 300)
        }
    }


    companion object {
        val coeff = PIDCoefficients(
            3.0, 0.0, 0.0,
            errorBounds = Interval.symmetric(0.5)
        )

        val headingCoeff = PIDCoefficients(
            2.0, 0.0, 0.0,
            errorBounds = Interval.symmetric(0.5)
        )
    }
}
