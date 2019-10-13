package org.futurerobotics.jargon.blocks

/**
 * Represents special inputs given to [Block]s that tap into the life of a [BlocksSystem] itself, and
 * so are special input values.
 */
interface SystemValues {
    /** The time in seconds the last loop has taken to run. */
    val loopTime: Double

    /** The number of the current loop run since `init`, starting from 0. */
    val loopNumber: Int
}

/**
 * A block whose outputs directly correspond to [SystemValues], if such connections are desired.
 */
class SystemValuesBlock : AbstractBlock(
    0, 2,
    Block.Processing.IN_FIRST_LAZY
) {
    private var systemValues: SystemValues? = null
    override fun init() {
        systemValues = null
    }

    override fun process(inputs: List<Any?>, systemValues: SystemValues) {
        this.systemValues = systemValues
    }

    override fun getOutput(index: Int): Any? = systemValues!!.run {
        when (index) {
            0 -> loopTime
            1 -> loopNumber
            else -> throw IndexOutOfBoundsException(index)
        }
    }

    val loopTime: BlockOutput<Double> get() = outputIndex(0)
    val loopNumber: BlockOutput<Int> get() = outputIndex(1)
}