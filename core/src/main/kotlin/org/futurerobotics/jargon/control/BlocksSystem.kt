package org.futurerobotics.jargon.control

import org.futurerobotics.jargon.system.LoopSystem
import kotlin.contracts.ExperimentalContracts
import kotlin.contracts.InvocationKind
import kotlin.contracts.contract

/**
 * A system made up of several connected [Block]s, which are processed accordingly.
 *
 * This is the bridge to _systems_ via [LoopSystem]
 *
 * Use [BlocksSystemBuilder] to create, configuring connections in [BlocksConfig].
 */
@Suppress("RedundantVisibilityModifier")
class BlocksSystem internal constructor(
    connections: Collection<BaseBlocksConfig.BlockConnections>,
    private val systemValues: SystemValuesBlock
) : AbstractBlocksRunner(connections), LoopSystem {


    public override fun init() {
        super.init()
    }

    override fun loop(loopTime: Double): Boolean {

        systemValues.loopNumberVal = loopNumber + 1
        systemValues.loopTimeVal = loopTime

        processOnce()

        return systemValues.shutdownVal
    }

    public override fun stop() {
        super.stop()
    }
}


/**
 * Builds a [BlocksSystem]. Connect blocks using the functions in (the superclass) [BlocksConfig], then run [build].
 *
 * For kotlin, remember to set a context with `this` as this object.
 *
 * Alternatively, use [buildBlocksSystem] which does both of the above in one go.
 */
class BlocksSystemBuilder : BaseBlocksConfig() {


    private val _systemValues = SystemValuesBlock()
    override val systemValues: SystemValues get() = _systemValues

    init {
        _systemValues.ensureAdded()
    }

    private var built = false
    /** Builds a [BlocksSystem] with the current configurations. */
    fun build(): BlocksSystem {
        check(!built) { "Already built!" }
        built = true
        verifyConfig()
        return BlocksSystem(connections, _systemValues)
    }
}

internal class SystemValuesBlock : AbstractBlock(1, 2, Block.Processing.OUT_FIRST_ALWAYS), SystemValues {
    override val shutdown: BlockInput<Boolean?> = inputIndex(0)
    var shutdownVal: Boolean = false
    override fun process(inputs: List<Any?>) {
        shutdownVal = inputs[0] as Boolean? ?: false
    }

    override var loopNumber: BlockOutput<Int> = outputIndex(0)
    var loopNumberVal: Int = 0
    override var loopTime: BlockOutput<Double> = outputIndex(1)
    var loopTimeVal: Double = Double.NaN
    override fun getOutput(index: Int, inputs: List<Any?>): Any? = when (index) {
        0 -> loopNumberVal
        1 -> loopTimeVal
        else -> throw IndexOutOfBoundsException(index)
    }

    override fun init() {
        loopNumberVal = 0
        loopTimeVal = Double.NaN
    }

    override fun verifyConfig(config: BlocksConfig) {
        //do nothing
    }
}


/**
 * DSL to build a block system.
 * Runs the [configuration] block on a [BlocksSystemBuilder] then returns its result.
 */
@UseExperimental(ExperimentalContracts::class)
inline fun buildBlocksSystem(configuration: BlocksSystemBuilder.() -> Unit): BlocksSystem {
    contract {
        callsInPlace(configuration, InvocationKind.EXACTLY_ONCE)
    }
    return BlocksSystemBuilder().run {
        configuration()
        build()
    }
}