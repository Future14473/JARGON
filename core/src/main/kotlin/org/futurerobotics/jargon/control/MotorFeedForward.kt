package org.futurerobotics.jargon.control

import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.model.MotorFrictionModel
import kotlin.math.sign

/**
 * Represents motor friction feed-forward based off of a [MotorFrictionModel].
 *
 * @param activationThreshold the minimum motor angular speed needed to activate the feed-forward. If motors are
 *                          jittering when they are supposed to be not moving, try increasing this value.
 *
 */
class MotorFeedForward(
    private val motorFrictionModel: MotorFrictionModel,
    private val activationThreshold: Double
) {

    /**
     * Gets a voltage signal with this feed forward applied, given the [rawVoltageSignal] without this feed forward
     * and the current [wheelVelocities].
     */
    fun applyFeedForward(rawVoltageSignal: Vec, wheelVelocities: Vec): Vec {
        val feedForward = motorFrictionModel.voltsForMotorFriction *
                wheelVelocities.map {
                    if (it < activationThreshold) 0.0 else sign(it)
                }
        return rawVoltageSignal + feedForward
    }
}
