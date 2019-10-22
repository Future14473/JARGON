package org.futurerobotics.jargon.blocks.control

import org.futurerobotics.jargon.blocks.BlocksConfig
import org.futurerobotics.jargon.blocks.CombineBlock
import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.math.EPSILON
import org.futurerobotics.jargon.mechanics.FixedDriveModel
import kotlin.math.sign

/**
 * A block that takes in motor voltages and the wheel's current velocities, and adds the voltage required to
 * overcome
 *
 * @param driveModel the model used to get frictional forces
 */
class MotorFrictionFF(driveModel: FixedDriveModel) : CombineBlock<List<Double>, List<Double>, List<Double>>() {
    private val stallVolts: Vec = driveModel.frictionAdditionalVolts
    @Suppress("UnnecessaryVariable")
    override fun combine(a: List<Double>, b: List<Double>): List<Double> {
        val voltages = a
        val vels = b
        var i = 0
        return voltages.zip(vels) { voltage, vel ->
            val sign = if (vel <= EPSILON) sign(voltage) else sign(vel)
            voltage + sign * stallVolts[i++]
        }
    }

    /** The motor voltages input */
    val motorVoltages: BlocksConfig.Input<List<Double>> get() = firstInput

    /** The motor velocities input */
    val motorVelocities: BlocksConfig.Input<List<Double>> get() = secondInput
}