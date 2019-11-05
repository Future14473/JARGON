@file:Suppress("GraziInspection")

package org.futurerobotics.jargon.simulation

import org.futurerobotics.jargon.blocks.Block
import org.futurerobotics.jargon.blocks.BlockSystem
import org.futurerobotics.jargon.blocks.Shutdown
import org.futurerobotics.jargon.blocks.buildBlockSystem
import org.futurerobotics.jargon.blocks.control.*
import org.futurerobotics.jargon.blocks.functional.ExternalQueue
import org.futurerobotics.jargon.blocks.functional.ShiftMotionOnlyToState
import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.math.Pose2d
import org.futurerobotics.jargon.math.Vector2d
import org.futurerobotics.jargon.math.angleNorm
import org.futurerobotics.jargon.math.max
import org.futurerobotics.jargon.mechanics.MotionState
import org.futurerobotics.jargon.mechanics.NominalFixedWheelDriveModel
import org.futurerobotics.jargon.mechanics.ValueMotionState
import org.futurerobotics.jargon.pathing.trajectory.Trajectory
import org.futurerobotics.jargon.statespace.*
import kotlin.math.abs

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
    protected val system: BlockSystem
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

        val recordings: Recordings
        val system = buildBlockSystem {
            recordings = Recordings(this).apply {
                val queue = trajectories
                val follower = TimeOnlyMotionProfileFollower<MotionState<Pose2d>>(
                    ValueMotionState.ofAll(Pose2d.ZERO)
                )() {
                    profileInput from queue.output
                }

                val refState = follower.output
                val positionController = FeedForwardWrapper.withAdder(
                    nonFFController,
                    Pose2d::plus
                )() {
                    reference from refState
                }

                fun Block.Output<Double>.recordY(s: String, s1: String) {
                    recordY(this, s, s1)
                }

                fun Block.Output<Vector2d>.recordXY(s: String, s1: String) {
                    recordXY(this, s, s1)
                }
                refState.pipe { it.s.x }.recordY("x reference", "reference value")
                refState.pipe { it.s.y }.recordY("y reference", "reference value")
                refState.pipe { it.s.heading }.recordY("heading reference", "reference value")

                refState.pipe { it.v.x }.recordY("x reference", "reference velocity")
                refState.pipe { it.v.y }.recordY("y reference", "reference velocity")
                refState.pipe { it.v.heading }.recordY("heading reference", "reference velocity")

                val botMotion = GlobalToBotMotion()() { globalMotion from positionController.signal }
                recordY(botMotion.output.pipe { it.v.x }, "x reference", "velocity signal")
                recordY(botMotion.output.pipe { it.v.y }, "y reference", "velocity signal")
                recordY(botMotion.output.pipe { it.v.heading }, "heading reference", "velocity signal")

                val wheelVelRef = botMotion.output
                    .pipe(BotToMotorMotion(driveModel)).also {
                        val refs = it.pipe { it.v }
                        repeat(numWheels) { i ->
                            recordY(refs.pipe { it[i] }, "Wheel velocities $i", "Reference $i")
                        }
                    }
                    .pipe(ShiftMotionOnlyToState(zeroVec(numWheels)))

                val ssController = SSControllerWithFF(ssModel, kGain)() { reference from wheelVelRef }



                motorsBlock.motorVolts from ssController.signal.pipe { it.asList() }
                    .apply {
                        repeat(numWheels) { i ->
                            recordY(pipe { it[i] }, "Voltages", "Voltage $i")
                        }
                    }


                LinearKalmanFilter(ssModel, kfilterQ, kfilterR)() {
                    measurement from motorsBlock.motorVelocities.pipe { it.toVec() }
                    signal from ssController.signal.delay(zeroVec(numWheels))
                    this.output into ssController.state

                    val measured = this
                    repeat(numWheels) { i ->
                        recordY(measured.output.pipe { it[i] }, "Wheel velocities $i", "Estimated $i")
                    }
                    val actual = motorsBlock.motorVelocities
                    repeat(numWheels) { i ->
                        recordY(actual.pipe { it[i] }, "Wheel velocities $i", "Actual $i")
                    }
                }

                val delta = FixedDriveMotorToBotDelta(driveModel)() {
                    input from motorsBlock.motorPositions
                }

                val tracker = GlobalPoseTrackerFromDeltaAndGyro()() {
                    deltaIn from delta.output; gyroIn from gyro.output
                    this.output into positionController.state
                    this.output into botMotion.globalPose

                    recordY(this.output.pipe { it.x }, "x reference", "measured value")
                    recordY(this.output.pipe { it.y }, "y reference", "measured value")
                    recordY(this.output.pipe { it.heading }, "heading reference", "measured value")

                }

                val refS = refState.pipe { it.s }
                Shutdown().signal from generate {
                    !follower.isFollowing.get && (tracker.output.get - refS.get).let { (vec, heading) ->
                        max(abs(vec.x), abs(vec.y), angleNorm(heading)) < 0.1
                    }
                }

                refState.pipe { it.s.vec }.recordXY("Path", "Reference")
                recordXY(tracker.output.pipe { it.vec }, "Path", "Estimated pose")
                val actualPose = motorsBlock.actualPose
                actualPose.pipe { it.vec }.recordXY("Path", "Actual pose")
                actualPose.pipe { it.x }.recordY("x reference", "Actual value")
                actualPose.pipe { it.y }.recordY("y reference", "Actual value")
                actualPose.pipe { it.heading }.recordY("heading reference", "Actual value")
                repeat(numWheels) { i ->
                    recordY(ssController.signal.pipe { it[i] }, "Wheel velocities $i", "Voltage $i")
                }
            }
        }
        this.system = system
        this.recordings = recordings
    }
}

