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
import org.futurerobotics.jargon.mechanics.*
import org.futurerobotics.jargon.pathing.MultiplePath
import org.futurerobotics.jargon.pathing.OffsetTangentHeading
import org.futurerobotics.jargon.pathing.addHeading
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

private val motorModel = DcMotorModel.fromMotorData(
    12 * volts,
    260 * ozf * `in`,
    9.2 * A,
    435 * rev / min,
    0.25 * A
)
private val transmissionModel = TransmissionModel.fromTorqueLosses(motorModel, 2.0, 0.0, 0.9)
private const val mass = 10.8 * lbs
private val driveModel = DriveModels.mecanumLike(
    mass,
    mass / 6 * (18 * `in`).squared(),
    transmissionModel,
    2 * `in`,
    16 * `in`,
    14 * `in`
)

internal class ASimulation {

    private val period = 1.0 / 20
    private val trajectories = ExternalQueue<Trajectory>()
    private val constraints = MotionConstraintSet(
        MaxVelocityConstraint(0.5),
        MaxAngularVelocityConstraint(0.6),
        MaxTotalAccelConstraint(0.6),
        MaxAngularAccelConstraint(0.6)
    )

    private val system: BlocksSystem
    private val recordings: Recordings

    init {
        val simulatedDrive = SimulatedFixedDrive(
            driveModel,
            FixedDriveModelPerturber(
                0.005, 0.005, FixedWheelModelPerturb(
                    0.0005, 0.00001, 0.005, TransmissionModelPerturb(
                        0.005, 0.1, 0.005, DcMotorModelPerturb(0.001)
                    )
                )
            ),
            Random("a simulation".hashCode().toLong()),
            eye(4) * 0.05,
            eye(4) * 0.05,
            0.005
        )

        val motorsBlock = SimulatedDriveBlock(simulatedDrive)
        val gyro = GyroBlock(SimulatedGyro(simulatedDrive))

        val continuous = LinearDriveModels.poseVelocityController(driveModel)
        val kGain = continuousLQR(continuous, QRCost(eye(3) * 3.0, eye(4)))

        val ssModel: DiscreteLinSSModel = continuous.discretize(period)

        val coeff = PIDCoefficients(
            4.0, 0.0, 0.0,
            errorBounds = Interval.symmetric(0.5)
        )
        val (system, recordings) = buildRecordingBlocksSystem {
            val queue = trajectories
            val follower =
                TimeOnlyMotionProfileFollower<MotionState<Pose2d>>(ValueMotionState.ofAll(Pose2d.ZERO)).apply {
                    profileInput from queue
                }

            val positionController =
                FeedForwardController.withAdder(PosePIDController(coeff, coeff, coeff), Pose2d::plus).apply {
                    reference from follower.output
                }
            follower.output.pipe { s.x }.recordY("x reference", "reference value")
            follower.output.pipe { s.y }.recordY("y reference", "reference value")
            follower.output.pipe { s.heading }.recordY("heading reference", "reference value")

            follower.output.pipe { v.x }.recordY("x reference", "reference velocity")
            follower.output.pipe { v.y }.recordY("y reference", "reference velocity")
//            follower.output.pipe { v.heading }.recordY("heading reference", "reference velocity")

            val botMotion = GlobalToBotMotion().apply { reference from positionController }
            botMotion.pipe { v.x }.recordY("x reference", "velocity signal")
            botMotion.pipe { v.y }.recordY("y reference", "velocity signal")
//            botMotion.pipe { v.heading }.recordY("heading reference", "velocity signal")

            val poseVelRef = botMotion
                .pipe(MapMotionOnly.of<Pose2d, Vec> { it.toVector() })
                .pipe(ShiftMotionOnlyToState(zeroVec(3)))

            val ssController = SSControllerWithFF(ssModel, kGain)() { reference from poseVelRef }

            motorsBlock {
                this from ssController.pipe { asList() }
            }

            KalmanFilter(ssModel, eye(3) * 0.03, eye(4) * 0.05)() {
                measurement from motorsBlock.motorVelocities.pipe { toVec() }
                signal from ssController.delay(zeroVec(4))
                ssController.state from this
            }
            val delta = FixedDriveMotorToBotDelta(driveModel)() {
                this from motorsBlock.motorPositions
//                motorPositions from simulated.motorPos
//                gyro from GyroBlock(gyro)
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
            motorsBlock.actualPose.pipe { heading }.recordY("heading reference", "Actual value")
            motorsBlock.actualPose.listen { }
        }
        this.system = system
        this.recordings = recordings
    }


    @Test
    fun simulation() {
        val random = Random("The continuing simulation".hashCode().toLong()).asKotlinRandom()
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

//        val path = Line(Vector2d.ZERO, Vector2d(2, 0)).addHeading(TangentHeading)
//        val trajectory = constraints.generateTrajectory(path)
//        trajectories.add(trajectory)

        val driver = LimitedLoopSystemDriver(5_000, LoopAsFastAsPossible(FixedTestClock((1e9 * period).roundToLong())))
        driver.run(system)

        recordings.getAllGraphs().forEach { (name, graph) ->
            graph.saveGraph(name, 300)
        }
    }
}
