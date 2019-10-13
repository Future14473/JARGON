package org.futurerobotics.jargon.control

import org.futurerobotics.jargon.control.Block.Processing.IN_FIRST_ALWAYS
import org.futurerobotics.jargon.control.Block.Processing.IN_FIRST_LAZY

/**
 * A block that contains within itself another block sub-system, configured in [buildSubsystem]
 *
 * When this block is processed it will process the entire sub-system.
 *
 * The subsystem does not support [SpecialBlock]s.
 */
//experimental
abstract class CompositeBlock(numInputs: Int, numOutputs: Int, processing: Block.Processing) :
    AbstractBlock(numInputs, numOutputs, processing) {
    private lateinit var subsystem: Subsystem
    override fun init(): Unit = subsystem.init()

    override fun process(inputs: List<Any?>, systemValues: SystemValues): Unit = subsystem.process(inputs, systemValues)

    override fun getOutput(index: Int): Any? = subsystem.getOutput(index)

    private inner class Subsystem(
        connections: Collection<BaseBlocksConfig.BlockConnections>,
        private val sources: Sources,
        private val outputs: Outputs
    ) : AbstractBlocksRunner(connections) {
        override lateinit var systemValues: SystemValues

        public override fun init() = super.init()
        public override fun stop() = super.stop()

        fun process(outsideSources: List<Any?>?, systemValues: SystemValues) {
            this.systemValues = systemValues
            sources.outsideSources = outsideSources
            processOnce()
        }

        fun getOutput(index: Int): Any? = outputs.extractFromSystem!![index]
    }


    private inner class Sources : AbstractBlock(0, numInputs, IN_FIRST_LAZY) {
        var outsideSources: List<Any?>? = null

        fun allOutputs() = List(this.numOutputs) { outputIndex<Any?>(it) }

        override fun init() {
            outsideSources = null
        }

        override fun process(inputs: List<Any?>, systemValues: SystemValues) {}

        override fun getOutput(index: Int): Any? = outsideSources!![index]
    }

    private inner class Outputs : AbstractBlock(numOutputs, 0, IN_FIRST_ALWAYS) {
        var extractFromSystem: List<Any?>? = null
            private set

        fun allInputs() = List(this.numInputs) { inputIndex<Any?>(it) }

        override fun init() {
            extractFromSystem = null
        }

        override fun process(inputs: List<Any?>, systemValues: SystemValues) {
            extractFromSystem = inputs
        }

        override fun getOutput(index: Int): Any? {
            throw IndexOutOfBoundsException(index)
        }
    }

    /**
     * Configures the sub-system using the given [BlocksConfig], the [BlockOutput] from the giving value [sources] from
     * the outside world, and [BlockInput] for given [outputs] to the outside world.
     *
     * Note that these inputs/outputs only work in the given subsystem. Having blocks in both the input and the output
     * subsystems leads to undefined behavior.
     */
    protected abstract fun BlocksConfig.buildSubsystem(
        sources: List<BlockOutput<Any?>>,
        outputs: List<BlockInput<Any?>>
    )

    override fun prepareAndVerify(config: BlocksConfig) {
        subsystem = SubsystemConfig().run {
            buildSubsystem(sources.allOutputs(), outputs.allInputs())
            getSystem()
        }
        doVerifyConfig(config)
    }

    /** Same thing as [doVerifyConfig] */
    protected open fun doVerifyConfig(config: BlocksConfig) {
        super.prepareAndVerify(config)
    }

    private inner class SubsystemConfig : BaseBlocksConfig() {
        val sources = Sources()
        val outputs = Outputs()

        init {
            sources.ensureAdded()
            outputs.ensureAdded()
        }

        fun getSystem() = Subsystem(connections, sources, outputs)
    }


}



