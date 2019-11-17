package org.futurerobotics.jargon.simulation

import org.futurerobotics.jargon.blocks.BlockSystem
import org.futurerobotics.jargon.blocks.Shutdown
import org.futurerobotics.jargon.blocks.buildBlockSystem
import org.futurerobotics.jargon.blocks.control.*
import org.futurerobotics.jargon.blocks.functional.ExternalQueue
import org.futurerobotics.jargon.blocks.functional.MapMotionOnly
import org.futurerobotics.jargon.blocks.functional.ShiftMotionOnlyToState
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
    nonFFController: PosePIDController,
    lqrCost: QRCost,
    kfilterQ: Mat,
    kfilterR: Mat
) {

    protected val trajectories: ExternalQueue<Trajectory> = ExternalQueue()
    protected val system: BlockSystem
    protected val recordings: Recordings

    init {
        val numWheels = driveModel.numMotors
        val motorsBlock = SimulatedDriveBlock(simulatedDrive)
        val gyro = GyroReading(
            SimulatedGyro(simulatedDrive)
        )

        val continuous = DriveStateSpaceModels.poseVelocityController(driveModel, driveModel)
        val kGain = continuousLQR(continuous, lqrCost)

        val ssModel = continuous.discretize(period)


        system = buildBlockSystem {
            recordings = Recordings(this).apply {
                val queue = trajectories
                val follower =
                    TimeOnlyMotionProfileFollower<MotionState<Pose2d>>(
                        ValueMotionState.ofAll(Pose2d.ZERO)
                    ).apply {
                        profileInput from queue.output
                    }

                val refState = follower.output
                val positionController = FeedForwardWrapper.withAdder(
                    nonFFController,
                    Pose2d::plus
                )() {
                    reference from refState
                }

                recordY(refState.pipe { it.value.x }, "x reference", "reference value")
                recordY(refState.pipe { it.value.y }, "y reference", "reference value")
                recordY(refState.pipe { it.value.heading }, "heading reference", "reference value")

                recordY(refState.pipe { it.deriv.x }, "x reference", "reference velocity")
                recordY(refState.pipe { it.deriv.y }, "y reference", "reference velocity")
                recordY(refState.pipe { it.deriv.heading }, "heading reference", "reference velocity")

                val botMotion =
                    GlobalToBotMotion().apply { globalMotion from positionController.signal }
                recordY(botMotion.output.pipe { it.vel.x }, "x reference", "velocity signal")
                recordY(botMotion.output.pipe { it.vel.y }, "y reference", "velocity signal")
                recordY(botMotion.output.pipe { it.vel.heading }, "heading reference", "velocity signal")

                val poseVelRef = botMotion.output
                    .pipe(MapMotionOnly.with<Pose2d, Vec> { it.toVec() })
                    .pipe(ShiftMotionOnlyToState(zeroVec(3)))

                val ssController = SSControllerWithFF(ssModel, kGain)() { reference from poseVelRef }

                motorsBlock.motorVolts from ssController.signal
                    .apply {
                        repeat(4) { i ->
                            recordY(pipe { it[i] }, "Voltages", "Voltage $i")
                        }
                    }


                LinearKalmanFilter(ssModel, kfilterQ, kfilterR)() {
                    measurement from motorsBlock.motorVelocities
                    signal from ssController.signal.delay(zeroVec(numWheels))
                    ssController.state from output
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
                    !follower.isFollowing.get && (tracker.output.get - refS.get).let { (vec, heading) ->
                        max(abs(vec.x), abs(vec.y), abs(angleNorm(heading))) < 0.1
                    }
                }


                recordXY(refState.pipe { it.value.vec }, "Path", "Reference")
                recordXY(tracker.output.pipe { it.vec }, "Path", "Estimated pose")
                val actualPose = motorsBlock.actualPose
                recordXY(actualPose.pipe { it.vec }, "Path", "Actual pose")
                recordY(actualPose.pipe { it.x }, "x reference", "Actual value")
                recordY(actualPose.pipe { it.y }, "y reference", "Actual value")
                //            motorsBlock.actualPose.pipe { heading }.recordY("heading reference", "Actual value")
            }
        }
    }
}
