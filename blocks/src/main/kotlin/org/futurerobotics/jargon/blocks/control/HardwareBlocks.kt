package org.futurerobotics.jargon.blocks.control

import org.futurerobotics.jargon.blocks.Block
import org.futurerobotics.jargon.blocks.Block.Processing.LAZY
import org.futurerobotics.jargon.blocks.Block.Processing.OUT_FIRST
import org.futurerobotics.jargon.blocks.BlockIndicator
import org.futurerobotics.jargon.blocks.PrincipalOutputBlock
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
interface MotorsBlock : BlockIndicator {

    /** The number of motors in this motor block. */
    val numMotors: Int
    /** An output of measured motor positions, in radians. */
    val motorPositions: Block.Output<Vec>
    /** An output of the measured motor velocities, in radians per second. */
    val motorVelocities: Block.Output<Vec>
    /** An input of motor voltages. Optional.*/
    val motorVolts: Block.Input<Vec?>
}

/** A [MotorsBlock] that operates with a list of [DcMotor]s. */
class MotorList(private val motors: List<DcMotor>) : Block(OUT_FIRST), MotorsBlock {

    override val numMotors: Int get() = motors.size
    override val motorPositions: Output<Vec> = newOutput()
    override val motorVelocities: Output<Vec> = newOutput()
    override val motorVolts: Input<Vec?> = newOptionalInput()
    override fun init() {
        motors.forEach { it.init() }
    }

    override fun Context.process() {
        val volts = motorVolts.get
        if (volts != null) {
            require(volts.size == motors.size) { "Given voltage list is not the right size " }
            motors.forEachIndexed { i, motor ->
                motor.voltage = volts[i]
            }
        }
        motorPositions.set = motors.mapToVec { it.position }
        motorVelocities.set = motors.mapToVec { it.velocity }
    }

    override fun stop() {
        motors.forEach { it.stop() }
    }
}

/** A block that outputs readings from a [Gyro]scope. */
class GyroReading(private val gyro: Gyro) : PrincipalOutputBlock<Double>(LAZY) {

    override fun Context.getOutput(): Double = gyro.currentAngle
}

/**
 * A block that takes in motor voltages and the wheel's current velocities, and adds the voltage required to
 * overcome frictional forces.
 *
 * @param motorVelocityModel the model used
 */
class MotorFrictionFF(private val motorVelocityModel: MotorVelocityModel) : PrincipalOutputBlock<Vec>(LAZY) {

    /** The motor voltages input */
    val motorVolts: Input<Vec> = newInput()
    /** The motor velocities input */
    val motorVelocities: Input<Vec> = newInput()

    override fun Context.getOutput(): Vec {
        val voltages = motorVolts.get
        val vels = motorVelocities.get
        val signs = genVec(voltages.size) {
            val vel = vels[it]
            val volts = voltages[it]
            if (vel <= EPSILON) sign(volts) else sign(vel)
        }
        return voltages + motorVelocityModel.voltsForMotorFriction * signs
    }
}
