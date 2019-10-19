package org.futurerobotics.jargon.blocks

import org.futurerobotics.jargon.system.LoopSystem
import kotlin.contracts.ExperimentalContracts
import kotlin.contracts.InvocationKind
import kotlin.contracts.contract

/**
 * A system made up of several connected [Block]s, which are processed accordingly to their [processing][Block.processing].
 * See [Block] for more details.
 *
 * Use [BlocksSystemBuilder] to create, configuring connections using the DSL of [BlocksConfig].
 *
 * This system also supports all [SpecialBlock]s.
 *
 * This is the bridge to _systems_ via [LoopSystem]. Hurray for decoupling.
 */
@Suppress("RedundantVisibilityModifier")
class BlocksSystem internal constructor(
    connections: Collection<BaseBlocksConfig.BlockConnections>
) : AbstractBlocksRunner(connections), LoopSystem {
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
        val specials =
            connections.map { it.block }.filterIsInstance<SpecialBlock>()
        specials.groupByTo(HashMap()) { it.javaClass }.forEach { (type, list) ->
            require(list.size == 1) {
                "cannot have more than 1 of each type of special block in a system, " +
                        "found ${list.size} instances of ${type.simpleName}"
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
 * Builds a [BlocksSystem]. Connect blocks using the functions in (the superclass) [BlocksConfig], then run [build].
 *
 * For kotlin, remember to set a context with `this` as this object.
 *
 * Alternatively, use [buildBlocksSystem] which does both of the above in one go.
 */
open class BlocksSystemBuilder : BaseBlocksConfig() {

    private var built = false
    /** Builds a [BlocksSystem] with the current configurations. */
    open fun build(): BlocksSystem {
        check(!built) { "Already built!" }
        built = true
        verifyConfig()
        return BlocksSystem(connections)
    }
}

/**
 * DSL to build a block system.
 * Runs the [configuration] block on a [BlocksSystemBuilder] then returns the built [BlocksSystem].
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