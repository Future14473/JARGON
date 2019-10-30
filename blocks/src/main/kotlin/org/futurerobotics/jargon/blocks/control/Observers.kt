@file:Suppress("UNCHECKED_CAST")

package org.futurerobotics.jargon.blocks.control

import org.futurerobotics.jargon.blocks.Block.Processing.IN_FIRST_LAZY
import org.futurerobotics.jargon.blocks.BlocksConfig
import org.futurerobotics.jargon.blocks.CombineBlock
import org.futurerobotics.jargon.blocks.PipeBlock
import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.math.Pose2d
import org.futurerobotics.jargon.mechanics.DriveModel

/**
 * A [PipeBlock] that takes in motor _positions_, calculates the difference, and then estimate _bot_ pose delta using
 * a drive [model].
 *
 * Maybe pass through a filter first.
 */
class FixedDriveMotorToBotDelta(private val model: DriveModel) :
    PipeBlock<List<Double>, Pose2d>(IN_FIRST_LAZY) {

    private var pastPositions: Vec? = null
    override fun pipe(input: List<Double>): Pose2d {
        val pastPositions = pastPositions
        val curPositions = input.toVec()
        this.pastPositions = curPositions
        return if (pastPositions == null) Pose2d.ZERO else {
            Pose2d(model.botVelFromMotorVel(curPositions - pastPositions))
        }
    }
}

/**
 * A [CombineBlock] block that takes in motor _positions_ and gyro readings to estimate _bot_ pose delta,
 * using a drive [model].
 *
 * Maybe pass through a filter first.
 */
class FixedDriveMotorAndGyroToBotDelta(private val model: DriveModel) :
    CombineBlock<List<Double>, Double, Pose2d>(IN_FIRST_LAZY) {

    /** Motor positions input */
    val motorPositions: BlocksConfig.Input<List<Double>> get() = firstInput
    /** Gyroscope output */
    val gyro: BlocksConfig.Input<Double> get() = secondInput
    private var pastPositions: Vec? = null
    private var pastAngle: Double = Double.NaN
    override fun combine(a: List<Double>, b: Double): Pose2d {
        val pastPositions = pastPositions
        val pastAngle = pastAngle
        val curPositions = a.toVec()
        this.pastPositions = curPositions
        this.pastAngle = b
        return if (pastPositions == null) Pose2d.ZERO else {
            Pose2d(model.botVelFromMotorVel * (curPositions - pastPositions)).copy(heading = pastAngle - b)
        }
    }
}

/**
 * A component that takes in motor _velocities_ to estimate _bot pose velocity_, using a drive [model].
 *
 * Maybe pass through a filter first.
 */
class FixedDriveMotorToBotVel(private val model: DriveModel) :
    PipeBlock<List<Double>, Pose2d>(IN_FIRST_LAZY) {

    override fun pipe(input: List<Double>): Pose2d = Pose2d((model.botVelFromMotorVel * input.toVec()))
}
