package org.futurerobotics.jargon.blocks

/**
 * Represents special inputs given to [Block]s that tap into the life of a [BlocksSystem] itself, and
 * so are special input values.
 */
interface SystemValues {

    /** The time in seconds the last loop has taken to run. */
    val loopTime: Double
    /** The total amount of time elapsed since the block has first run. */
    val totalTime: Double
    /** The number of the current loop run since `init`, starting from 0. */
    val loopNumber: Int
}

/**
 * A block whose outputs directly correspond to [SystemValues], if such connections are desired.
 */
class SystemValuesBlock : AbstractBlock(0, 3, Block.Processing.IN_FIRST_LAZY) {

    /** The loopTime output */
    val loopTime: BlocksConfig.Output<Double> get() = configOutput(0)
    /** The totalTime output */
    val totalTime: BlocksConfig.Output<Double> get() = configOutput(1)
    /** the loopNumber output */
    val loopNumber: BlocksConfig.Output<Int> get() = configOutput(2)
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
            1 -> totalTime
            2 -> loopNumber
            else -> throw IndexOutOfBoundsException(index)
        }
    }
}
