@file:Suppress("UNCHECKED_CAST")

package org.futurerobotics.jargon.control

import org.futurerobotics.jargon.math.Pose2d
import org.futurerobotics.jargon.mechanics.FixedDriveModel


/**
 * A component that takes in motor _positions_ and a drive [model] to estimate _bot pose difference_.
 *
 * Maybe pass through a filter first.
 */
class FixedDriveMotorPoseToBotDiff(private val model: FixedDriveModel) : PipeBlock<List<Double>, Pose2d>() {
    private var pastPositions: List<Double>? = null
    override fun pipe(input: List<Double>): Pose2d {
        val pastPositions = pastPositions
        this.pastPositions = input
        return if (pastPositions == null) Pose2d.ZERO else {
            val diff = input.zip(pastPositions) { cur, past -> (cur - past) }
            model.getEstimatedVelocity(diff)
        }
    }
}

/**
 * A component that takes in motor _velocities_ and a drive [model] to estimate _bot pose velocity_.
 *
 * Maybe pass through a filter first.
 */
class FixedDriveMotorToBotVel(private val model: FixedDriveModel) : PipeBlock<List<Double>, Pose2d>() {
    override fun pipe(input: List<Double>): Pose2d {
        return model.getEstimatedVelocity(input)
    }
}
