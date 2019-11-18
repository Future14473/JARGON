package org.futurerobotics.jargon.simulation

import org.futurerobotics.jargon.blocks.Block
import org.futurerobotics.jargon.blocks.BlockSystem
import org.futurerobotics.jargon.blocks.Shutdown
import org.futurerobotics.jargon.blocks.buildBlockSystem
import org.futurerobotics.jargon.blocks.control.*
import org.futurerobotics.jargon.blocks.functional.ExternalQueue
import org.futurerobotics.jargon.blocks.functional.MapMotionState
import org.futurerobotics.jargon.blocks.functional.MotionOnlyToVelocityState
import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.math.*
import org.futurerobotics.jargon.mechanics.NominalDriveModel
import org.futurerobotics.jargon.pathing.trajectory.Trajectory
import org.futurerobotics.jargon.statespace.*
import kotlin.math.abs

internal abstract class PoseVelocityControllingSimulation(
    protected val driveModel: NominalDriveModel,
    simulatedDrive: SimulatedFixedDrive,
    val period: Double,
    nonFFController: Controller<Pose2d, Pose2d, Pose2d>,
    lqrCost: QRCost,
    noiseCovariance: NoiseCovariance
) {

    protected val trajectories: ExternalQueue<Trajectory> = ExternalQueue()
    protected val system: BlockSystem
    protected val recordings: Recordings

    init {
        val numMotors = driveModel.numMotors
        val motorsBlock = SimulatedDriveBlock(simulatedDrive)
        val gyro = GyroReading(
            SimulatedGyro(simulatedDrive)
        )

        val matrices = DriveStateSpaceModels.poseVelocityController(driveModel, driveModel)

        val discMatrices = discretize(matrices, 1 / 20.0)
        val discCost = discretizeQrCost(matrices, lqrCost, 1 / 20.0)
        val runner = StateSpaceRunnerBuilder()
            .setMatrices(discMatrices)
            .addGainController(discreteLQR(discMatrices, discCost))
            .addReferenceTracking(plantInversion(discMatrices, null))
            .addKalmanFilter {
                setNoiseCovariance(noiseCovariance)
                setInitialProcessCovariance(idenMat(3) * 0.05)
            }.build()

        val recordings: Recordings
        val system = buildBlockSystem {
            recordings = Recordings(this).apply {
                fun Block.Output<Double>.recordY(s: String, s1: String) = recordY(this, s, s1)

                fun Block.Output<Vector2d>.recordXY(s: String, s1: String) = recordXY(this, s, s1)

                val profileFollower = TimeOnlyMotionProfileFollower<MotionState<Pose2d>>(
                    ValueMotionState.ofAll(Pose2d.ZERO)
                )() {
                    profileInput from trajectories.output
                }

                val refState = profileFollower.output
                val positionController = FeedForwardWrapper.withAdder(
                    nonFFController,
                    Pose2d::plus
                )() {
                    reference from refState
                }

                refState.pipe { it.value.x }.recordY("x reference", "reference value")
                refState.pipe { it.value.y }.recordY("y reference", "reference value")
                refState.pipe { it.value.heading }.recordY("heading reference", "reference value")

                refState.pipe { it.deriv.x }.recordY("x reference", "reference velocity")
                refState.pipe { it.deriv.y }.recordY("y reference", "reference velocity")
                refState.pipe { it.deriv.heading }.recordY("heading reference", "reference velocity")

                val botMotion = GlobalToBotMotion()() { globalMotion from positionController.signal }
                recordY(botMotion.output.pipe { it.vel.x }, "x reference", "velocity signal")
                recordY(botMotion.output.pipe { it.vel.y }, "y reference", "velocity signal")
                recordY(botMotion.output.pipe { it.vel.heading }, "heading reference", "velocity signal")

                val poseVelRef = botMotion.output
                    .pipe(MotionOnlyToVelocityState(Pose2d.ZERO))
                    .pipe(MapMotionState.with<Pose2d, Vec> { it.toVec() })

                val ssController = StateSpaceRunnerBlock(
                    runner, zeroVec(3)
                )() {
                    referenceMotionState from poseVelRef
                }


                motorsBlock {
                    motorVolts from ssController.signal
                    ssController.measurement from motorVelocities
                }

                ssController.signal.apply {
                    repeat(numMotors) { i ->
                        recordY(pipe { it[i] }, "Voltages", "Voltage $i")
                    }
                }

                val measured = ssController.measurement.source()!!
                repeat(numMotors) { i ->
                    recordY(measured.pipe { it[i] }, "Wheel velocities $i", "Estimated $i")
                }
                val actual = generate { simulatedDrive.curMotorVelocities }
                repeat(numMotors) { i ->
                    recordY(actual.pipe { it[i] }, "Wheel velocities $i", "Actual $i")
                }

                val delta = MotorToBotDelta(driveModel)() {
                    input from motorsBlock.motorPositions
                }

                val tracker = GlobalPoseTrackerFromDeltaAndGyro()() {
                    deltaIn from delta.output; gyroIn from gyro.output
                    output into positionController.state
                    output into botMotion.globalPose

                    recordY(output.pipe { it.x }, "x reference", "measured value")
                    recordY(output.pipe { it.y }, "y reference", "measured value")
                    recordY(output.pipe { it.heading }, "heading reference", "measured value")

                }

                val refS = refState.pipe { it.value }
                Shutdown().signal from generate {
                    !profileFollower.isFollowing.get && (tracker.output.get - refS.get).let { (vec, heading) ->
                        max(abs(vec.x), abs(vec.y), angleNorm(heading)) < 0.1
                    }
                }

                refState.pipe { it.value.vec }.recordXY("Path", "Reference")
                recordXY(tracker.output.pipe { it.vec }, "Path", "Estimated pose")
                val actualPose = motorsBlock.actualPose
                actualPose.pipe { it.vec }.recordXY("Path", "Actual pose")
                actualPose.pipe { it.x }.recordY("x reference", "Actual value")
                actualPose.pipe { it.y }.recordY("y reference", "Actual value")
                actualPose.pipe { it.heading }.recordY("heading reference", "Actual value")
                repeat(numMotors) { i ->
                    recordY(ssController.signal.pipe { it[i] }, "Wheel velocities $i", "Voltage $i")
                }
            }
        }
        this.system = system
        this.recordings = recordings
    }
}
