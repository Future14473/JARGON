package org.futurerobotics.temporaryname.hardware

import org.futurerobotics.temporaryname.control.Plant
import org.futurerobotics.temporaryname.mechanics.MotorPositions
import org.futurerobotics.temporaryname.mechanics.MotorVoltages
import org.futurerobotics.temporaryname.util.zipForEach

/**
 * Base class for a set of Motors that use encoders as measurement.
 */
abstract class BaseEncoderMotors : Plant<MotorVoltages, MotorPositions> {

    /**
     * The number of motors in this set.
     */
    abstract val numMotors: Int

    override fun start() {}
    override fun stop() {
        signal(MotorVoltages(List(numMotors) { 0.0 }, false), 0.0)
    }
}

/**
 * A [Plant] that is simply a list of [DcMotor]s, and measurements based off of encoders.
 */
class EncoderMotors(private val motors: List<DcMotor>) : BaseEncoderMotors() {

    override val numMotors: Int
        get() = motors.size
    override val measurement: MotorPositions
        get() = MotorPositions(motors.map { it.getPosition() })

    override fun start() {
        motors.forEach { it.start() }
    }

    override fun signal(signal: MotorVoltages, elapsedSeconds: Double) {
        motors.zipForEach(signal) { motor, volts ->
            motor.setVoltage(volts)
        }
    }
}