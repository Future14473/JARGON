@file:Suppress("UNCHECKED_CAST")

package org.futurerobotics.jargon.blocks.control

import org.futurerobotics.jargon.blocks.*
import org.futurerobotics.jargon.blocks.Block.Processing.OUT_FIRST_ALWAYS
import org.futurerobotics.jargon.hardware.DcMotor
import org.futurerobotics.jargon.hardware.Gyro
import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.math.EPSILON
import org.futurerobotics.jargon.mechanics.MotorVelocityModel
import kotlin.math.sign

/**
 * A block that interacts directly with motors, either real or simulated.
 *
 * Inputs:
 * - `this`: Voltage signal as a list of doubles.
 *
 * Outputs:
 * - [motorPositions]: a list of motor positions (in radians)
 * - [motorVelocities]: a list of motor velocities (in radians per second)
 */
interface MotorsBlock : Block, BlocksConfig.Input<List<Double>?> {

    /** The number of motors in this motor block. */
    val numMotors: Int
    /** An output of measured motor positions, in radians. */
    val motorPositions: BlocksConfig.Output<List<Double>>
    /** An output of the measured motor velocities, in radians per second. */
    val motorVelocities: BlocksConfig.Output<List<Double>>
}

/** A [MotorsBlock] that operates with a list of [DcMotor]s. */
class MotorList(private val motors: List<DcMotor>) : SingleInputListStoreBlock<List<Double>?>(2, OUT_FIRST_ALWAYS),
                                                     MotorsBlock {

    override val numMotors: Int get() = motors.size
    override val motorPositions: BlocksConfig.Output<List<Double>> get() = configOutput(0)
    override val motorVelocities: BlocksConfig.Output<List<Double>> get() = configOutput(1)
    private fun writeMeasurements(outputs: MutableList<Any?>) {
        outputs[0] = motors.map { it.position }
        outputs[1] = motors.map { it.velocity }
    }

    override fun init(outputs: MutableList<Any?>) {
        writeMeasurements(outputs)
    }

    override fun processInput(input: List<Double>?, systemValues: SystemValues, outputs: MutableList<Any?>) {
        if (input != null) {
            require(input.size == motors.size) { "Given voltage list is not the right size " }
            motors.forEachIndexed { index, dcMotor ->
                dcMotor.voltage = input[index]
            }
        }
        writeMeasurements(outputs)
    }

    override fun prepareAndVerify(config: BlocksConfig) {
        //do nothing.
    }
}

/** A block that outputs readings from a [Gyro]scope. */
class GyroReading(private val gyro: Gyro) : SingleOutputBlock<Double>(0, OUT_FIRST_ALWAYS) {

    override fun initialValue(): Double? = gyro.currentAngle
    override fun processOutput(inputs: List<Any?>, systemValues: SystemValues): Double = gyro.currentAngle
}

/**
 * A block that takes in motor voltages and the wheel's current velocities, and adds the voltage required to
 * overcome frictional forces.
 *
 * @param driveModel the model used
 */
class MotorFrictionFF(private val driveModel: MotorVelocityModel) :
    CombineBlock<List<Double>, List<Double>, List<Double>>() {

    /** The motor voltages input */
    val motorVoltages: BlocksConfig.Input<List<Double>> get() = firstInput
    /** The motor velocities input */
    val motorVelocities: BlocksConfig.Input<List<Double>> get() = secondInput

    @Suppress("UnnecessaryVariable")
    override fun combine(a: List<Double>, b: List<Double>): List<Double> {
        val voltages = a
        val vels = b
        val signs = voltages.zip(vels) { voltage, vel ->
            if (vel <= EPSILON) sign(voltage) else sign(vel)
        }.toVec()
        return (driveModel.motorAccelForMotorFriction * signs).toList()
    }
}
