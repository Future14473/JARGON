package org.futurerobotics.jargon.blocks

import org.futurerobotics.jargon.util.uncheckedCast

internal class TestBlock(
    private val name: String,
    numInputs: Int,
    numOutputs: Int,
    processing: Processing = Processing.LAZY,
    private val requireAllInputs: Boolean = true
) : Block(processing) {

    init {
        repeat(numInputs) { Input<String>(null, !requireAllInputs) }
        repeat(numOutputs) { Output<String>(null) }
    }

    private var updateNum = 0

    override fun Context.process() {
        if (isFirstTime && processing == Processing.OUT_FIRST) {
            repeat(numOutputs) {
                outputs[it].set = "$name$it-"
            }
            return
        }
        val list = if (processing == Processing.OUT_FIRST) {
            List(numInputs) {
                inputs[it].get.let { str ->
                    if (str is String) str.substring(0, str.indexOf('[')) else null
                }
            }
        } else {
            List(numInputs) { inputs[it].get }
        }.joinToString()
        repeat(numOutputs) {
            outputs[it].set = "$name$it[$list]"
        }
    }

    fun output(index: Int): Output<String> = outputs[index].uncheckedCast()
    fun input(index: Int): Input<String?> = inputs[index].uncheckedCast()

    @Suppress("UNCHECKED_CAST")
    internal fun fromAll(builder: BlockArrangementBuilder, vararg outputs: Output<out String?>) {
        require(outputs.size <= numInputs) { "the given number of outputs ${outputs.size} must not exceed the block's number of inputs $this.numInputs" }
        outputs.forEachIndexed { index, output ->
            builder.connect(input(index), output)
        }
    }

    override fun toString(): String = name
}

