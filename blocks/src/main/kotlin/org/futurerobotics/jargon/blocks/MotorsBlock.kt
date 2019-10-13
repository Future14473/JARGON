@file:Suppress("UNCHECKED_CAST")

package org.futurerobotics.jargon.blocks

import org.futurerobotics.jargon.blocks.Block.Processing.OUT_FIRST_ALWAYS
import org.futurerobotics.jargon.hardware.DcMotor

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
class MotorsBlock(private val motors: List<DcMotor>) : ListStoreBlock(1, 2, OUT_FIRST_ALWAYS) {

    private fun writeMeasurements(outputs: MutableList<Any?>) {
        outputs[0] = motors.map { it.position }
        outputs[1] = motors.map { it.velocity }
    }


    override fun init(outputs: MutableList<Any?>) {
        writeMeasurements(outputs)
    }

    override fun process(inputs: List<Any?>, outputs: MutableList<Any?>) {
        (inputs[0] as List<Double>).let {
            require(it.size == motors.size) { "Given voltage list is not the right size " }
            motors.forEachIndexed { index, dcMotor ->
                dcMotor.voltage = it[index]
            }
        }
        writeMeasurements(outputs)
    }
}