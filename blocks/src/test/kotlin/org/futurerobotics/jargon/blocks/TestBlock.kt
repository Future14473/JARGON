package org.futurerobotics.jargon.blocks

import org.futurerobotics.jargon.util.fillWith

internal class TestBlock(
    private val name: String,
    numInputs: Int,
    numOutputs: Int,
    processing: Block.Processing = Block.Processing.IN_FIRST_LAZY,
    private val requireAllInputs: Boolean = true
) : ListStoreBlock(numInputs, numOutputs, processing) {
    private var updateNum = 0
    override fun process(inputs: List<Any?>, outputs: MutableList<Any?>) {
        val list = if (processing == Block.Processing.OUT_FIRST_ALWAYS) {
            List(numInputs) {
                inputs[it].let { str ->
                    if (str is String)
                        str.substring(0, str.indexOf('[')) else null
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

    override fun toString(): String = name

    fun output(index: Int = 0): BlocksConfig.Output<String> = outputIndex(index)

    fun input(index: Int = 0): BlocksConfig.Input<Any?> = inputIndex(index)

    override fun prepareAndVerify(config: BlocksConfig) {
        if (requireAllInputs) super.prepareAndVerify(config)
    }
}

