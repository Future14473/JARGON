package org.futurerobotics.jargon.hardware

import org.futurerobotics.jargon.control.AbstractBlock
import org.futurerobotics.jargon.control.Block.InOutOrder.OUT_FIRST
import org.futurerobotics.jargon.control.Block.Processing.ALWAYS
import org.futurerobotics.jargon.control.castToDoubleList

/**
 * A block that represents simple interaction with a list of [DcMotor]s.
 * It may be more ideal to use a "bulk signal/measurement" process instead.
 *
 * Inputs:
 * 1. Voltage signal: either A List<Double>, double array, or [Vec].
 *
 * Outputs:
 * 1. A List of Doubles of motor position (in radians)
 * 2. A list of doubles of motor velocity (in radians/second)
 *
 */
class MotorsBlock(private val motors: List<DcMotor>) : AbstractBlock(1, 2, OUT_FIRST, ALWAYS) {

    private fun writeMeasurements(outputs: MutableList<Any?>) {
        outputs[0] = motors.map { it.position }
        outputs[1] = motors.map { it.velocity }
    }


    override fun init(outputs: MutableList<Any?>) {
        writeMeasurements(outputs)
    }

    override fun process(inputs: List<Any?>, outputs: MutableList<Any?>) {
        inputs[0]!!.castToDoubleList().let {
            require(it.size == motors.size) { "Given voltage list is not the right size " }
            motors.forEachIndexed { index, dcMotor ->
                dcMotor.voltage = it[index]
            }
        }
        writeMeasurements(outputs)
    }
}