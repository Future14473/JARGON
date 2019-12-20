package org.futurerobotics.jargon.blocks

import org.futurerobotics.jargon.running.LoopSystem
import kotlin.contracts.ExperimentalContracts
import kotlin.contracts.InvocationKind
import kotlin.contracts.contract

/**
 * A system that runs a given [BlockArrangement].
 * This system also supports all [SpecialBlock]s.
 *
 * This implements [LoopSystem], so it can tie in with the rest of the world.
 *
 *
 * @see buildBlockSystem
 */
class BlockSystem(arrangement: BlockArrangement) : BlockRunner(arrangement),
                                                   LoopSystem {

    private val specials: Map<Class<*>, List<SpecialBlock>>
    private val _systemValues = object : SystemValues {
        override var totalTimeInNanos: Long = 0
        override val totalTime: Double get() = totalTimeInNanos / 1e9
        override var loopTimeInNanos: Long = 0
            set(value) {
                totalTimeInNanos += value
                field = value
            }
        override val loopTime: Double get() = loopTimeInNanos / 1e9
        override val loopNumber: Int get() = this@BlockSystem.loopNumber
    }
    override val systemValues: SystemValues = _systemValues

    init {
        specials = arrangement.connections.keys
            .filterIsInstance<SpecialBlock>()
            .groupByTo(HashMap()) { it.javaClass }
    }

    override fun init(): Unit = super<BlockRunner>.init()

    override fun loop(loopTimeInNanos: Long): Boolean {
        _systemValues.loopTimeInNanos = loopTimeInNanos

        processOnce()

        return specials[Shutdown::class.java]?.let { list ->
            @Suppress("UNCHECKED_CAST")
            (list as List<Shutdown>).any { it.shutDownSignal }
        } == true
    }

    override fun stop(): Unit = super<BlockRunner>.stop()
}

/**
 * DSL to build a block system.
 * Runs the [configuration] block on a [BlockArrangementBuilder], and uses it to create a [BlockSystem].
 */
@UseExperimental(ExperimentalContracts::class)
inline fun buildBlockSystem(configuration: BlockArrangementBuilder.() -> Unit): BlockSystem {
    contract {
        callsInPlace(configuration, InvocationKind.EXACTLY_ONCE)
    }
    val config = BlockArrangementBuilder().build(configuration)
    return BlockSystem(config)
}
