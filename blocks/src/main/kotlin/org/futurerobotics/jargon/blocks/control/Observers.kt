@file:Suppress("UNCHECKED_CAST")

package org.futurerobotics.jargon.blocks.control

import org.futurerobotics.jargon.blocks.Block.Processing.IN_FIRST_LAZY
import org.futurerobotics.jargon.blocks.BlocksConfig
import org.futurerobotics.jargon.blocks.CombineBlock
import org.futurerobotics.jargon.blocks.PipeBlock
import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.math.Pose2d
import org.futurerobotics.jargon.mechanics.FixedDriveModel


/**
 * A [PipeBlock] block that takes in motor _positions_ to estimate _bot pose difference_ with a drive [model].
 *
 * Maybe pass through a filter first.
 */
class FixedDriveMotorToBotDelta(private val model: FixedDriveModel) : PipeBlock<List<Double>, Pose2d>(IN_FIRST_LAZY) {
    private var pastPositions: Vec? = null
    override fun pipe(input: List<Double>): Pose2d {
        val pastPositions = pastPositions
        val curPositions = input.toVec()
        this.pastPositions = curPositions
        return if (pastPositions == null) Pose2d.ZERO else {
            model.getBotVelFromMotorVel(curPositions - pastPositions)
        }
    }
}

/**
 * A [CombineBlock] block that takes in motor _positions_ and _gyro readings_ to estimate _bot pose difference_
 * with a drive [model].
 *
 * Maybe pass through a filter first.
 */
class FixedDriveMotorAndGyroToBotDelta(private val model: FixedDriveModel) :
    CombineBlock<List<Double>, Double, Pose2d>(IN_FIRST_LAZY) {
    private var pastPositions: Vec? = null
    private var pastAngle: Double = Double.NaN
    override fun combine(a: List<Double>, b: Double): Pose2d {
        val pastPositions = pastPositions
        val pastAngle = pastAngle
        val curPositions = a.toVec()
        val curAngle = b
        this.pastPositions = curPositions
        this.pastAngle = curAngle
        return if (pastPositions == null) Pose2d.ZERO else {
            model.getBotVelFromMotorVel(curPositions - pastPositions).copy(heading = pastAngle - curAngle)
        }
    }

    /** Motor positions input */
    val motorPositions: BlocksConfig.Input<List<Double>> = configInput(0)
    /** Gyroscope output */
    val gyro: BlocksConfig.Input<Double> = configInput(1)
}

/**
 * A component that takes in motor _velocities_ and a drive [model] to estimate _bot pose velocity_.
 *
 * Maybe pass through a filter first.
 */
class FixedDriveMotorToBotVel(private val model: FixedDriveModel) : PipeBlock<List<Double>, Pose2d>(IN_FIRST_LAZY) {
    override fun pipe(input: List<Double>): Pose2d = model.getBotVelFromMotorVel(createVec(input))
}