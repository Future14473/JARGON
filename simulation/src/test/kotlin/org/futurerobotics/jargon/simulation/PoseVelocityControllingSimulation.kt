package org.futurerobotics.jargon.simulation

import org.futurerobotics.jargon.blocks.BlocksSystem
import org.futurerobotics.jargon.blocks.Shutdown
import org.futurerobotics.jargon.blocks.control.FeedForwardWrapper
import org.futurerobotics.jargon.blocks.control.FixedDriveMotorToBotDelta
import org.futurerobotics.jargon.blocks.control.GyroReading
import org.futurerobotics.jargon.blocks.control.PosePIDController
import org.futurerobotics.jargon.blocks.functional.ExternalQueue
import org.futurerobotics.jargon.blocks.functional.MapMotionOnly
import org.futurerobotics.jargon.blocks.functional.ShiftMotionOnlyToState
import org.futurerobotics.jargon.blocks.motion.GlobalPoseTrackerFromDeltaAndGyro
import org.futurerobotics.jargon.blocks.motion.GlobalToBotMotion
import org.futurerobotics.jargon.blocks.motion.TimeOnlyMotionProfileFollower
import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.math.Pose2d
import org.futurerobotics.jargon.math.distTo
import org.futurerobotics.jargon.mechanics.FixedDriveModel
import org.futurerobotics.jargon.mechanics.MotionState
import org.futurerobotics.jargon.mechanics.ValueMotionState
import org.futurerobotics.jargon.pathing.trajectory.Trajectory
import org.futurerobotics.jargon.statespace.*
import kotlin.math.max

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
        val gyro = GyroReading(
            SimulatedGyro(simulatedDrive)
        )

        val continuous = DriveStateSpaceModels.poseVelocityController(driveModel)
        val kGain = continuousLQR(continuous, lqrCost)

        val ssModel = continuous.discretize(period)


        val (system, recordings) = buildRecordingBlocksSystem {
            val queue = trajectories
            val follower =
                TimeOnlyMotionProfileFollower<MotionState<Pose2d>>(
                    ValueMotionState.ofAll(Pose2d.ZERO)
                ).apply {
                    profileInput from queue
                }

            val positionController = FeedForwardWrapper.withAdder(
                nonFFController,
                Pose2d::plus
            ).apply {
                reference from follower.output
            }
            follower.output.pipe { s.x }.recordY("x reference", "reference value")
            follower.output.pipe { s.y }.recordY("y reference", "reference value")
            follower.output.pipe { s.heading }.recordY("heading reference", "reference value")

            follower.output.pipe { v.x }.recordY("x reference", "reference velocity")
            follower.output.pipe { v.y }.recordY("y reference", "reference velocity")
            follower.output.pipe { v.heading }.recordY("heading reference", "reference velocity")

            val botMotion =
                GlobalToBotMotion().apply { globalMotion from positionController }
            botMotion.pipe { v.x }.recordY("x reference", "velocity signal")
            botMotion.pipe { v.y }.recordY("y reference", "velocity signal")
            botMotion.pipe { v.heading }.recordY("heading reference", "velocity signal")

            val poseVelRef = botMotion
                .pipe(MapMotionOnly.with<Pose2d, Vec> { it.toVec() })
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

            LinearKalmanFilter(ssModel, kfilterQ, kfilterR)() {
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
                max(vec distTo it.vec, (heading distTo it.heading))
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
