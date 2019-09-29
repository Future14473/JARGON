//package org.futurerobotics.jargon.hardware
//
//import org.futurerobotics.jargon.control.Plant
//import org.futurerobotics.jargon.mechanics.MotorPositions
//import org.futurerobotics.jargon.mechanics.MotorVoltages
//import org.futurerobotics.jargon.util.zipForEach
//
///**
// * Base class for a [Plant] that is a set of Motors that use encoder _position_ as measurement.
// */
//abstract class BaseEncoderMotors : Plant<MotorVoltages, MotorPositions> {
//
//    /**
//     * The number of motors in this set.
//     */
//    abstract val numMotors: Int
//
//    override fun init() {}
//    override fun stop() {
//        signal(List(numMotors) { 0.0 }, 0.0)
//    }
//}
//
///**
// * A [Plant] that is simply a list of [DcMotor]s, and measurements based off of encoders.
// */
//class EncoderMotors(private val motors: List<DcMotor>) : BaseEncoderMotors() {
//
//    override val numMotors: Int
//        get() = motors.size
//    override val measurement: MotorPositions
//        get() = motors.map { it.getPosition() }
//
//    override fun init() {
//        motors.forEach { it.init() }
//    }
//
//    override fun signal(signal: MotorVoltages, elapsedSeconds: Double) {
//        motors.zipForEach(signal) { motor, volts ->
//            motor.setVoltage(volts)
//        }
//    }
//}