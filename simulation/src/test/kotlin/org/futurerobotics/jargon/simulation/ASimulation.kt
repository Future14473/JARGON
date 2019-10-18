package org.futurerobotics.jargon.simulation

import org.futurerobotics.jargon.blocks.BlocksSystem
import org.futurerobotics.jargon.blocks.Constant
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
import org.futurerobotics.jargon.system.FixedTestClock
import org.futurerobotics.jargon.system.LimitedLoopSystemDriver
import org.futurerobotics.jargon.system.LoopAsFastAsPossible
import org.junit.jupiter.api.Test
import java.util.*
import kotlin.math.max
import kotlin.math.roundToLong
import kotlin.random.asKotlinRandom

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
        val driveModel: FixedDriveModel = DriveModels.mecanumLike(
            5 * lbs, 10.0, TransmissionModel.fromTorqueLosses(
                DcMotorModel.fromMotorData(
                    12 * volts, 350 * ozf * `in`, 11.5 * A, 160 * rev / min, 0.5 * A
                ),
                1.0, 1.0, 0.9
            ), 2 * `in`, 8 * `in`, 8 * `in`
        )
        val continuous = LinearDriveModels.poseVelocityController(driveModel)
        val kGain = continuousLQR(continuous, QRCost(eye(3) * 3.0, eye(4)))

        val ssModel: DiscreteLinSSModel = continuous.discretize(period)
        val actualDrive = SimulatedFixedDrive(
            driveModel,
            FixedDriveModelPerturber(
                0.005, 0.005, FixedWheelModelPerturb(
                    0.005, 0.0, 0.05, TransmissionModelPerturb(
                        0.005, 0.1, 0.05, DcMotorModelPerturb(0.0001)
                    )
                )
            ),
            Random("a simulation".hashCode().toLong()),
            eye(4) * 0.05,
            eye(4) * 0.05,
            0.005
        )
        val coeff = PIDCoefficients(
            0.0, 0.0, 0.0,
            errorBounds = Interval.symmetric(0.5)
        )
        val (system, recordings) = buildBlocksRecordingSystem {
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

            follower.output.pipe { v.x }.recordY("x reference", "reference velocity")
            follower.output.pipe { v.y }.recordY("y reference", "reference velocity")

            val botMotion = GlobalToBotMotionReference().apply { reference from positionController }
            botMotion.pipe { v.x }.recordY("x reference", "velocity signal")
            botMotion.pipe { v.y }.recordY("y reference", "velocity signal")
            val (refVPose, refAPose) = SplitMotionOnly<Pose2d>()() { this from botMotion }

            val ref = CreateMotionState<Vec>().apply {
                value from refVPose.pipe { toVector() }
                vel from refAPose.pipe { toVector() }
                accel from Constant(zeroVec(3))
            }

            val ssController = SSControllerWithFF(ssModel, kGain)() { reference from ref }
//            ssController.pipe { println(asList()) }.monitor()

            val simulated = SimulatedDriveInput(actualDrive).apply {
                this from ssController.pipe { asList() }
                    .also {
                        //                        repeat(4) { i ->
//                            it.pipe { this[i] }.recordY("Voltage signals", "Voltage $i")
//                        }
                    }
            }

            KalmanFilter(ssModel, eye(3) * 0.03, eye(4) * 0.05).apply {
                measurement from simulated.motorVel.pipe { toVec() }
                signal from ssController.delay(zeroVec(4))
                ssController.state from this
            }
            val tracker = GlobalPoseTrackerFromDelta().apply {
                deltaIn from simulated.motorPos.pipe(FixedDriveMotorToBotDelta(driveModel))
                this into positionController.state
                this into botMotion.globalPose

                pipe { x }.recordY("x reference", "measured value")
                pipe { y }.recordY("y reference", "measured value")

            }
            val error = tracker.combine(follower.output.pipe { s }) {
                max(this.vec distTo it.vec, (this.heading distTo it.heading))
            }
//            Shutdown() from follower.isFollowing.combine(error) { !this && it < 0.03 }

            follower.output.pipe { s.vec }.recordXY("Path", "Reference")
            tracker.pipe { vec }.recordXY("Path", "Estimated pose")
            simulated.actualPose.pipe { vec }.recordXY("Path", "Actual pose")
        }
        this.system = system
        this.recordings = recordings
    }


    @Test
    fun simulation() {
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

        val driver = LimitedLoopSystemDriver(5_000, LoopAsFastAsPossible(FixedTestClock((1e9 * period).roundToLong())))
        driver.run(system)

        recordings.getAllGraphs().forEach { (name, graph) ->
            graph.saveGraph(name, 300)
        }
    }
}
