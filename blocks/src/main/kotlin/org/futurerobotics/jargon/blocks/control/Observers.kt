@file:Suppress("UNCHECKED_CAST")

package org.futurerobotics.jargon.blocks.control

import org.futurerobotics.jargon.blocks.Block.Processing.IN_FIRST_LAZY
import org.futurerobotics.jargon.blocks.Pipe
import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.math.Pose2d
import org.futurerobotics.jargon.mechanics.FixedDriveModel


/**
 * A [Pipe] component that takes in motor _positions_ and a drive [model] to estimate _bot pose difference_.
 *
 * Maybe pass through a filter first.
 */
class FixedDriveMotorToBotDelta(private val model: FixedDriveModel) : Pipe<List<Double>, Pose2d>(IN_FIRST_LAZY) {
    private var pastPositions: Vec? = null
    override fun pipe(input: List<Double>): Pose2d {
        val pastPositions = pastPositions
        val input = createVec(input)
        this.pastPositions = input
        return if (pastPositions == null) Pose2d.ZERO else {
            model.getBotVelFromMotorVel(input - pastPositions)
        }
    }
}

/**
 * A component that takes in motor _velocities_ and a drive [model] to estimate _bot pose velocity_.
 *
 * Maybe pass through a filter first.
 */
class FixedDriveMotorToBotVel(private val model: FixedDriveModel) : Pipe<List<Double>, Pose2d>(IN_FIRST_LAZY) {
    override fun pipe(input: List<Double>): Pose2d = model.getBotVelFromMotorVel(createVec(input))
}