package org.futurerobotics.jargon.control

import org.futurerobotics.jargon.control.Block.Processing.IN_FIRST_ALWAYS
import org.futurerobotics.jargon.control.Block.Processing.IN_FIRST_LAZY

/**
 * A block that contains within itself another block sub-system.
 *
 * As of now [SystemValues]s cannot be retrieved from within this block.
 *
 * When this block is processed it will process the entire sub-system.
 */
//experimental
internal abstract class CompositeBlock(
    private val subNumInputs: Int,
    private val subNumOutputs: Int,
    processing: Block.Processing
) : AbstractBlock(subNumInputs + ADDITIONAL_INPUTS, subNumOutputs + ADDITIONAL_OUTPUTS, processing) {
    private lateinit var subsystem: Subsystem
    override fun init() {
        subsystem.init()
    }

    override fun process(inputs: List<Any?>) {
        subsystem.process(inputs)
    }

    override fun getOutput(index: Int, inputs: List<Any?>): Any? {
        return subsystem.getOutput(index) //also contains additional outputs.
    }

    /**
     * Configures the sub-system using the given [BlocksConfig],
     * and the [sources] from the outside world and [outputs] to the outside world, in order.
     */
    protected abstract fun BlocksConfig.configureSystem(
        sources: List<BlockOutput<Any?>>,
        outputs: List<BlockInput<Any?>>
    )

    final override fun selfConfig(config: BlocksConfig) {
        val subsystemConfig = SubsystemConfig().apply {
            configureSystem(sources.regularOutputs(), outputs.regularInputs())
            config.apply {
                if (outputs.additionalOutput<Any?>(0).isConnected())
                    config.systemValues.shutdown connectFrom super.outputIndex(subNumOutputs + 0)

                config.systemValues.loopNumber connectTo super.inputIndex(subNumInputs + 0)
                config.systemValues.loopTime connectTo super.inputIndex(subNumInputs + 1)
            }

        }
        subsystem = subsystemConfig.build()
    }

    private inner class SubsystemConfig : BaseBlocksConfig() {
        val sources = Sources()
        val outputs = Outputs()

        init {
            sources.ensureAdded()
            outputs.ensureAdded()
        }

        override val systemValues: SystemValues = object : SystemValues {
            override val shutdown: BlockInput<Boolean?>
                get() = outputs.additionalOutput(0)
            override val loopNumber: BlockOutput<Int>
                get() = sources.additionalInput(0)
            override val loopTime: BlockOutput<Double>
                get() = sources.additionalInput(1)
        }

        fun build() = Subsystem(connections, sources, outputs)
    }

    override fun <T> inputIndex(index: Int): BlockInput<T> {
        if (index !in 0..subNumInputs) throw IndexOutOfBoundsException(index) //do not allow for additional
        return BasicBlockInput(this, index)
    }

    override fun <T> outputIndex(index: Int): BlockOutput<T> {
        if (index !in 0..subNumOutputs) throw IndexOutOfBoundsException(index) //do not allow for additional
        return BasicBlockOutput(this, index)
    }

    private inner class Subsystem(
        connections: Collection<BaseBlocksConfig.BlockConnections>,
        private val sources: Sources,
        private val outputs: Outputs
    ) : AbstractBlocksRunner(connections) {

        public override fun init() = super.init()

        public override fun stop() = super.stop()

        fun process(outsideSources: List<Any?>?) {
            sources.outsideSources = outsideSources
            processOnce()
        }

        fun getOutput(index: Int): Any? {
            return outputs.extractFromSystem!![index] //also contains additional outputs.
        }
    }


    private inner class Sources :
        AbstractBlock(0, subNumInputs + ADDITIONAL_INPUTS, IN_FIRST_LAZY) {
        var outsideSources: List<Any?>? = null //also contains additional inputs.

        fun regularOutputs() = List(subNumInputs) { outputIndex<Any?>(it) }
        fun <T> additionalInput(index: Int) = outputIndex<T>(index + subNumInputs)


        override fun init() {
            outsideSources = null
        }

        override fun process(inputs: List<Any?>) {}

        override fun getOutput(index: Int, inputs: List<Any?>): Any? = outsideSources!![index]
    }

    private inner class Outputs : AbstractBlock(subNumOutputs + ADDITIONAL_OUTPUTS, 0, IN_FIRST_ALWAYS) {
        var extractFromSystem: List<Any?>? = null //also contains additional outputs.
            private set

        fun regularInputs() = List(subNumOutputs) { inputIndex<Any?>(it) }
        fun <T> additionalOutput(index: Int) = inputIndex<T>(index + subNumOutputs)

        override fun init() {
            extractFromSystem = null
        }

        override fun process(inputs: List<Any?>) {
            extractFromSystem = inputs
        }

        override fun getOutput(index: Int, inputs: List<Any?>): Any? {
            throw IndexOutOfBoundsException(index)
        }

        override fun verifyConfig(config: BlocksConfig) = config.run {
            repeat(subNumInputs) {
                //ignore additional outputs.
                if (!inputIndex<Any>(it).isConnected())
                    throw IllegalBlockConfigurationException("All inputs to $this must be connected.")
            }
        }
    }

    companion object {
        private const val ADDITIONAL_INPUTS = 2
        private const val ADDITIONAL_OUTPUTS = 1
    }
}



