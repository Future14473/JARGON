package org.futurerobotics.jargon.blocks.control

import org.futurerobotics.jargon.blocks.Block
import org.futurerobotics.jargon.blocks.Block.Processing.LAZY
import org.futurerobotics.jargon.blocks.Block.Processing.OUT_FIRST
import org.futurerobotics.jargon.blocks.PrincipalOutputBlock
import org.futurerobotics.jargon.hardware.Gyro
import org.futurerobotics.jargon.hardware.Motor
import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.math.EPSILON
import org.futurerobotics.jargon.math.angleNorm
import org.futurerobotics.jargon.mechanics.MotorVelocityModel
import kotlin.math.sign

/**
 * A block that interacts directly with motors, either real or simulated.
 *
 * Can input [motorVolts], and output [motorPositions] and [motorVelocities].
 */
interface MotorsBlock {

    /** The number of motors in this motor block. */
    val numMotors: Int
    /** An output of measured motor positions, in radians. */
    val motorPositions: Block.Output<Vec>
    /** An output of the measured motor velocities, in radians per second. */
    val motorVelocities: Block.Output<Vec>
    /** An input of motor voltages. Optional.*/
    val motorVolts: Block.Input<Vec?>
}

/** A [MotorsBlock] that operates with a list of [Motor]s. */
class MotorListBlock(private val motors: List<Motor>) : Block(OUT_FIRST), MotorsBlock {

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

/**
 * A block that outputs readings from a [Gyro]scope.
 *
 * An [initialHeading] can be provided, where this will calibrate to when the system first starts.
 * If the [initialHeading] is Double.NaN, no calibration will be performed.
 */
class GyroBlock @JvmOverloads constructor(
    private val gyro: Gyro, initialHeading: Double = Double.NaN
) : PrincipalOutputBlock<Double>(OUT_FIRST) {

    /** The current gyro heading reading, possibly with a constant offset. */
    val headingMeasurement: Output<Double> get() = super.output

    private var _initialheading: Double = angleNorm(initialHeading)

    /**
     * The initial heading that this will calibrate to when the system first starts.
     * If the [initialHeading] is Double.NaN, no calibration will be performed.
     *
     * If this is set when the block is running, the offset will be recalculated.
     */
    var initialHeading: Double
        get() = _initialheading
        set(value) {
            _initialheading = angleNorm(value)
            if (!value.isNaN())
                init()
        }
    @Volatile
    private var offset: Double = 0.0

    private fun calibrate() {
    }

    override fun init() {
        offset =
            if (initialHeading.isNaN()) 0.0
            else angleNorm(initialHeading - gyro.currentAngle)
    }

    override fun Context.getOutput(): Double = gyro.currentAngle + offset
}

/**
 * A block that takes in motor voltages and the wheel's current velocities, and adds the (modeled) voltage required to
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
