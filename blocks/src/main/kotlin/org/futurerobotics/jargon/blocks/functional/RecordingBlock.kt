package org.futurerobotics.jargon.blocks.functional

import org.futurerobotics.jargon.blocks.InputOnlyBlock
import org.futurerobotics.jargon.blocks.SystemValues
import org.futurerobotics.jargon.util.asUnmodifiableList

/**
 * A block with a single input that records the values it received every loop.
 * Usually used for graphing.
 */
class RecordingBlock<T> : InputOnlyBlock<T>() {

    private val _values = ArrayList<T>()
    /** The values recorded by this block */
    val values: List<T> = _values.asUnmodifiableList()

    override fun init() {
    }

    override fun processInput(input: T, systemValues: SystemValues) {
        _values += input
    }
}
