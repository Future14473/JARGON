package org.futurerobotics.jargon.blocks

import org.futurerobotics.jargon.blocks.config.BCBuilder
import org.futurerobotics.jargon.blocks.config.BlockConfig
import org.futurerobotics.jargon.system.looping.LoopSystem
import kotlin.contracts.ExperimentalContracts
import kotlin.contracts.InvocationKind
import kotlin.contracts.contract

/**
 * A system made up of several connected [Block]s, which when run, is processed accordingly to their
 * [processing][Block.processing]. Takes in a [BlockConfig] that defines the system.
 *
 * This system also supports all [SpecialBlock]s.
 *
 * This is the bridge to _systems_ via [LoopSystem]. Hurray for decoupling.
 *
 * @see buildBlockSystem
 */
class BlockSystem(config: BlockConfig) : BlockRunner(config), LoopSystem {

    private val specials: Map<Class<*>, List<SpecialBlock>>
    private val _systemValues = object : SystemValues {
        private var totalTimeNanos: Long = 0
        var loopTimeInNanos: Long = 0
            set(value) {
                totalTimeNanos += value
                field = value
            }
        override val loopTime: Double get() = loopTimeInNanos / 1e9
        override val totalTime: Double get() = totalTimeNanos / 1e9
        override val loopNumber: Int get() = this@BlockSystem.loopNumber
    }
    override val systemValues: SystemValues = _systemValues

    init {
        specials = config.connections.keys
            .filterIsInstance<SpecialBlock>()
            .groupByTo(HashMap()) { it.javaClass }
    }

    override fun start(): Unit = super.init()

    override fun loop(loopTimeInNanos: Long): Boolean {
        _systemValues.loopTimeInNanos = loopTimeInNanos

        processOnce()

        return specials[Shutdown::class.java]?.let { list ->
            @Suppress("UNCHECKED_CAST")
            (list as List<Shutdown>).any { it.shutDownSignal }
        } == true
    }

    @Suppress("RedundantOverride") //trust me, it's not redundant
    override fun stop(): Unit = super.stop()
}

/**
 * DSL to build a block system.
 * Runs the [configuration] block on a [BCBuilder], and uses it to create a [BlockSystem].
 */
@UseExperimental(ExperimentalContracts::class)
inline fun buildBlockSystem(configuration: BCBuilder.() -> Unit): BlockSystem {
    contract {
        callsInPlace(configuration, InvocationKind.EXACTLY_ONCE)
    }
    val config = BCBuilder().build(configuration)
    return BlockSystem(config)
}
