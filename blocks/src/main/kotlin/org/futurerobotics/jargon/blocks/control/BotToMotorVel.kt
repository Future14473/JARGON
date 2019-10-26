package org.futurerobotics.jargon.blocks.control

import org.futurerobotics.jargon.blocks.PipeBlock
import org.futurerobotics.jargon.blocks.functional.MapMotionOnly
import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.math.Pose2d
import org.futurerobotics.jargon.mechanics.FixedDriveModel
import org.futurerobotics.jargon.mechanics.MotionOnly

/**
 * A [PipeBlock] that converts bot velocities (pose) into wheel velocities (vec), using the given [driveModel]
 */
class BotToMotorVel(private val driveModel: FixedDriveModel) : PipeBlock<Pose2d, Vec>() {

    override fun pipe(input: Pose2d): Vec = driveModel.motorVelFromBotVel * input.toVec()
}

/**
 * A [PipeBlock] that converts bot pose [MotionOnly]  into wheel velocities (vec) [MotionOnly], using the given
 * [driveModel].
 */
class BotToMotorMotion(private val driveModel: FixedDriveModel) : MapMotionOnly<Pose2d, Vec>() {

    override fun map(value: Pose2d): Vec = driveModel.motorVelFromBotVel * value.toVec()
}
