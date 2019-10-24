package org.futurerobotics.jargon.blocks

import org.futurerobotics.jargon.system.LoopSystem
import kotlin.contracts.ExperimentalContracts
import kotlin.contracts.InvocationKind
import kotlin.contracts.contract

/**
 * A system made up of several connected [Block]s, which are processed accordingly to their [processing][Block.processing].
 * See [Block] for more details.
 *
 * Takes in a [config] to build the system; Try not to have the same block in two systems at once; as that causes
 * undefined behavior.
 *
 * This system also supports all [SpecialBlock]s.
 *
 * This is the bridge to _systems_ via [LoopSystem]. Hurray for decoupling.
 */
@Suppress("RedundantVisibilityModifier")
class BlocksSystem(config: BlocksConfig) : AbstractBlocksRunner(config), LoopSystem {

    private val specials: Map<Class<*>, SpecialBlock>
    private val _systemValues = object : SystemValues {
        private var totalTimeNanos: Long = 0
        var loopTimeInNanos: Long = 0
            set(value) {
                totalTimeNanos += value
                field = value
            }
        override val loopTime: Double get() = loopTimeInNanos / 1e9
        override val totalTime: Double get() = totalTimeNanos / 1e9
        override val loopNumber: Int get() = this@BlocksSystem.loopNumber
    }
    override val systemValues: SystemValues = _systemValues

    init {
        val specials = config.connections.keys.filterIsInstance<SpecialBlock>()
        specials.groupByTo(HashMap()) { it.javaClass }.forEach { (type, list) ->
            require(list.size == 1) {
                "cannot have more than 1 of each type of special block in a system, " + "found ${list.size} instances of ${type.simpleName}"
            }
        }
        this.specials = specials.associateByTo(HashMap()) { it.javaClass }
    }

    public override fun init() {
        super.init()
    }

    override fun loop(loopTimeInNanos: Long): Boolean {
        _systemValues.loopTimeInNanos = loopTimeInNanos

        processOnce()

        return specials[Shutdown::class.java]?.let {
            (it as Shutdown).shutDownSignal
        } == true
    }

    public override fun stop() {
        super.stop()
    }
}

/**
 * DSL to build a block system.
 * Runs the [configuration] block on a [BaseBlocksConfig] then returns the built [BlocksSystem].
 */
@UseExperimental(ExperimentalContracts::class)
inline fun buildBlocksSystem(configuration: BlocksConfig.() -> Unit): BlocksSystem {
    contract {
        callsInPlace(configuration, InvocationKind.EXACTLY_ONCE)
    }
    return BaseBlocksConfig().run {
        configuration()
        BlocksSystem(this)
    }
}