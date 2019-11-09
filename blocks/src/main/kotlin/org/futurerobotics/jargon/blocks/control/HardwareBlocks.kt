package org.futurerobotics.jargon.blocks.control

import org.futurerobotics.jargon.blocks.Block
import org.futurerobotics.jargon.blocks.Block.Processing.LAZY
import org.futurerobotics.jargon.blocks.Block.Processing.OUT_FIRST
import org.futurerobotics.jargon.blocks.SingleInputBlock
import org.futurerobotics.jargon.blocks.SingleOutputBlock
import org.futurerobotics.jargon.hardware.DcMotor
import org.futurerobotics.jargon.hardware.Gyro
import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.math.EPSILON
import org.futurerobotics.jargon.mechanics.MotorVelocityModel
import org.futurerobotics.jargon.util.zipForEach
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
interface MotorsBlock {

    /** The number of motors in this motor block. */
    val numMotors: Int
    /** An output of measured motor positions, in radians. */
    val motorPositions: Block.Output<List<Double>>
    /** An output of the measured motor velocities, in radians per second. */
    val motorVelocities: Block.Output<List<Double>>
    /** An input of motor voltages. Optional.*/
    val motorVolts: Block.Input<List<Double>?>
}

/** A [MotorsBlock] that operates with a list of [DcMotor]s. */
class MotorList(private val motors: List<DcMotor>) : SingleInputBlock<List<Double>?>(OUT_FIRST), MotorsBlock {

    override val numMotors: Int get() = motors.size
    override val motorPositions: Output<List<Double>> = newOutput()
    override val motorVelocities: Output<List<Double>> = newOutput()
    override val motorVolts: Input<List<Double>?> get() = super.input

    private fun Context.writeMeasurements() {
        motorPositions.set = motors.map { it.position }
        motorVelocities.set = motors.map { it.velocity }
    }

    override fun Context.process(
        input: List<Double>?
    ) {
        if (input != null) {
            require(input.size == motors.size) { "Given voltage list is not the right size " }
            motors.zipForEach(input) { m, v ->
                m.voltage = v
            }
        }
        writeMeasurements()
    }
}

/** A block that outputs readings from a [Gyro]scope. */
class GyroReading(private val gyro: Gyro) : SingleOutputBlock<Double>(LAZY) {

    override fun Context.getOutput(): Double = gyro.currentAngle
}

/**
 * A block that takes in motor voltages and the wheel's current velocities, and adds the voltage required to
 * overcome frictional forces.
 *
 * @param motorVelocityModel the model used
 */
class MotorFrictionFF(private val motorVelocityModel: MotorVelocityModel) : SingleOutputBlock<List<Double>>(LAZY) {

    /** The motor voltages input */
    val motorVoltages: Input<List<Double>> = newInput()
    /** The motor velocities input */
    val motorVelocities: Input<List<Double>> = newInput()

    override fun Context.getOutput(): List<Double> {
        val voltages = motorVoltages.get
        val vels = motorVelocities.get
        val signs = voltages.zip(vels) { voltage, vel ->
            if (vel <= EPSILON) sign(voltage) else sign(vel)
        }.toVec()
        return (motorVelocityModel.motorAccelForMotorFriction * signs).toList()
    }
}
