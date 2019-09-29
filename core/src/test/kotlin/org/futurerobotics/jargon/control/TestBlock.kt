package org.futurerobotics.jargon.control

import org.futurerobotics.jargon.util.fillWith

internal class TestBlock(
    val name: String,
    numInputs: Int,
    numOutputs: Int,
    inOutOrder: Block.InOutOrder = Block.InOutOrder.IN_FIRST,
    processing: Block.Processing = Block.Processing.LAZY
) : AbstractBlock(numInputs, numOutputs, inOutOrder, processing) {
    private var updateNum = 0
    override fun process(inputs: List<Any?>, outputs: MutableList<Any?>) {

        val list = if (inOutOrder == Block.InOutOrder.OUT_FIRST) {
            List(numInputs) {
                inputs[it].let { str ->
                    str as String
                    str.substring(0, str.indexOf('['))
                }
            }
        } else {
            List(numInputs) { inputs[it] }
        }.joinToString()
        outputs.fillWith { "$name$it[$list]" }
    }

    override fun init(outputs: MutableList<Any?>) {
        outputs.fillWith { "$name$it-" }
    }

    override fun toString(): String {
        return name
    }
}