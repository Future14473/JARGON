@file:Suppress("GraziInspection")

package org.futurerobotics.jargon.simulation

import org.futurerobotics.jargon.blocks.BlocksSystem
import org.futurerobotics.jargon.blocks.Shutdown
import org.futurerobotics.jargon.blocks.control.*
import org.futurerobotics.jargon.blocks.functional.ExternalQueue
import org.futurerobotics.jargon.blocks.functional.ShiftMotionOnlyToState
import org.futurerobotics.jargon.blocks.motion.GlobalPoseTrackerFromDeltaAndGyro
import org.futurerobotics.jargon.blocks.motion.GlobalToBotMotion
import org.futurerobotics.jargon.blocks.motion.TimeOnlyMotionProfileFollower
import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.math.Pose2d
import org.futurerobotics.jargon.math.distTo
import org.futurerobotics.jargon.mechanics.GlobalToBot
import org.futurerobotics.jargon.mechanics.MotionState
import org.futurerobotics.jargon.mechanics.NominalFixedWheelDriveModel
import org.futurerobotics.jargon.mechanics.ValueMotionState
import org.futurerobotics.jargon.pathing.trajectory.Trajectory
import org.futurerobotics.jargon.statespace.*
import kotlin.math.max

internal abstract class DecoupWheelsSimulation(
    protected val driveModel: NominalFixedWheelDriveModel,
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
        val numWheels = driveModel.numMotors
        val motorsBlock = SimulatedDriveBlock(simulatedDrive)
        val gyro = GyroReading(
            SimulatedGyro(simulatedDrive)
        )

        val continuous = DriveStateSpaceModels.decoupledMotorVelocityController(driveModel, 0.5)
        val kGain = continuousLQR(continuous, lqrCost)

        val ssModel = continuous.discretize(period)


        val (system, recordings) = buildRecordingBlocksSystem {
            val queue = trajectories
            val follower = TimeOnlyMotionProfileFollower<MotionState<Pose2d>>(
                ValueMotionState.ofAll(Pose2d.ZERO)
            )() {
                profileInput from queue
            }

            val positionController = FeedForwardWrapper.withAdder(
                nonFFController,
                Pose2d::plus
            )() {
                reference from follower.output
            }
            follower.output.pipe { s.x }.recordY("x reference", "reference value")
            follower.output.pipe { s.y }.recordY("y reference", "reference value")
            follower.output.pipe { s.heading }.recordY("heading reference", "reference value")

            follower.output.pipe { v.x }.recordY("x reference", "reference velocity")
            follower.output.pipe { v.y }.recordY("y reference", "reference velocity")
            follower.output.pipe { v.heading }.recordY("heading reference", "reference velocity")
            val refV = follower.output.pipe { v }
            val refS = follower.output.pipe { s }
            val justFF = combine(refV, refS) { v, s ->
                GlobalToBot.velocity(v, s.heading)
            }.pipe(BotToMotorVel(driveModel))
            repeat(numWheels) { i ->
                justFF.pipe { this[i] }.recordY("Wheel velocities $i", "Just ff $i")
            }

            val botMotion = GlobalToBotMotion()() { globalMotion from positionController }
            botMotion.pipe { v.x }.recordY("x reference", "velocity signal")
            botMotion.pipe { v.y }.recordY("y reference", "velocity signal")
            botMotion.pipe { v.heading }.recordY("heading reference", "velocity signal")

            val wheelVelRef = botMotion
                .pipe(BotToMotorMotion(driveModel)).also {
                    val refs = it.pipe { v }
                    repeat(numWheels) { i ->
                        refs.pipe { this[i] }.recordY("Wheel velocities $i", "Reference $i")
                    }
                }
                .pipe(ShiftMotionOnlyToState(zeroVec(numWheels)))

            val ssController = SSControllerWithFF(ssModel, kGain)() { reference from wheelVelRef }

            motorsBlock {
                this from ssController.pipe { asList() }
                    .apply {
                        repeat(numWheels) {
                            pipe { this[it] }.recordY("Voltages", "Voltage $it")
                        }
                    }
            }

            LinearKalmanFilter(ssModel, kfilterQ, kfilterR)() {
                measurement from motorsBlock.motorVelocities.pipe { toVec() }
                signal from ssController.delay(zeroVec(numWheels))
                this into ssController.state

                val measured = this
                repeat(numWheels) { i ->
                    measured.pipe { this[i] }.recordY("Wheel velocities $i", "Estimated $i")
                }
                val actual = motorsBlock.motorVelocities
                repeat(numWheels) { i ->
                    actual.pipe { this[i] }.recordY("Wheel velocities $i", "Actual $i")
                }
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
            motorsBlock.actualPose.pipe { heading }.recordY("heading reference", "Actual value")
            repeat(numWheels) { i ->
                ssController.pipe { this[i] }.recordY("Wheel velocities $i", "Voltage $i")
            }
        }
        this.system = system
        this.recordings = recordings
    }
}
