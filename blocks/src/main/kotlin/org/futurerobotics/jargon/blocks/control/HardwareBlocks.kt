@file:Suppress("UNCHECKED_CAST")

package org.futurerobotics.jargon.blocks.control

import org.futurerobotics.jargon.blocks.*
import org.futurerobotics.jargon.blocks.Block.Processing.OUT_FIRST_ALWAYS
import org.futurerobotics.jargon.hardware.DcMotor
import org.futurerobotics.jargon.hardware.Gyro

/**
 * A block that interacts directly with motors hardware (or simulated).
 *
 * This itself is a [BlocksConfig.Input] that takes in a list of motor voltages.
 */
interface MotorsBlock : Block, BlocksConfig.Input<List<Double>> {
    /** The number of motors in this motors block */
    val numMotors: Int
    /** An output of a list of the measured motor positions, in radians. */
    val motorPositions: BlocksConfig.Output<List<Double>>
    /** An output of a list of the measured motor velocities, in radians per second */
    val motorVelocities: BlocksConfig.Output<List<Double>>
}

/**
 * A block that represents simple interaction with a list of [DcMotor]s.
 * It may be more ideal to use a "bulk signal/measurement" process instead.
 *
 * Inputs:
 * 1. Voltage signal: a List<Double>.
 *
 * Outputs:
 * 1. A List of Doubles of motor position (in radians)
 * 2. A list of doubles of motor velocity (in radians/second)
 *
 */
class MotorsListBlock(private val motors: List<DcMotor>) : SingleInputListStoreBlock<List<Double>>(2, OUT_FIRST_ALWAYS),
    MotorsBlock {
    override val numMotors: Int get() = motors.size

    private fun writeMeasurements(outputs: MutableList<Any?>) {
        outputs[0] = motors.map { it.position }
        outputs[1] = motors.map { it.velocity }
    }


    override fun init(outputs: MutableList<Any?>) {
        writeMeasurements(outputs)
    }

    override fun processInput(input: List<Double>, systemValues: SystemValues, outputs: MutableList<Any?>) {
        require(input.size == motors.size) { "Given voltage list is not the right size " }
        motors.forEachIndexed { index, dcMotor ->
            dcMotor.voltage = input[index]
        }
        writeMeasurements(outputs)
    }

    override val motorPositions: BlocksConfig.Output<List<Double>> = configOutput(0)
    override val motorVelocities: BlocksConfig.Output<List<Double>> = configOutput(1)
}


/** A block that represents reading from a [Gyro]scope. */
class GyroBlock(private val gyro: Gyro) : SingleOutputBlock<Double>(0, OUT_FIRST_ALWAYS) {
    override fun doInit(): Double? = gyro.currentAngle

    override fun processOutput(inputs: List<Any?>, systemValues: SystemValues): Double = gyro.currentAngle
}