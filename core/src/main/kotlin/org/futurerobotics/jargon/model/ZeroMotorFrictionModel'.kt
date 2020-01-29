package org.futurerobotics.jargon.model

import org.futurerobotics.jargon.linalg.*

/**
 * A [MotorFrictionModel] that models no friction.
 */
class ZeroMotorFrictionModel(override val numMotors: Int) : MotorFrictionModel {

    override val motorAccelForMotorFriction: Mat = Mat(numMotors, numMotors)
    override val voltsForMotorFriction: Mat = Mat(numMotors, numMotors)
}
