package org.futurerobotics.jargon.blocks

import org.futurerobotics.jargon.blocks.Block.Processing.ALWAYS
import org.futurerobotics.jargon.blocks.Block.Processing.LAZY
import org.futurerobotics.jargon.util.uncheckedCast

/**
 * A block that only takes one [input] of type [T] and one [output] of type [R], that is ran through a [pipe] function.
 */
abstract class PipeBlock<T, R>
@JvmOverloads constructor(
    processing: Processing = LAZY
) : Block(processing) {

    /** Input to this [PipeBlock] */
    val input: Input<T> = newInput()
    /** Output to this [PipeBlock] */
    val output: Output<R> = newOutput()

    /** Pipes the input to the output. */
    protected abstract fun Context.pipe(input: T): R

    final override fun Context.process() {
        output.set = pipe(input.get)
    }

    override fun toString(): String = javaClass.simpleName.ifEmpty { "PipeBlock" }

    companion object {

        /**
         * Creates a new [PipeBlock] with the given [piping] function.
         */
        inline fun <T, R> with(crossinline piping: Context.(T) -> R): PipeBlock<T, R> = object : PipeBlock<T, R>() {
            override fun Context.pipe(input: T): R = piping(input)
        }
    }
}

/**
 * A block which has a principle [output] of type [T].
 */
abstract class PrincipalOutputBlock<T>(processing: Processing) : Block(processing) {

    /** The output to this [PrincipalOutputBlock]. */
    val output: Output<T> = newOutput()

    /** Gets the output of this block. */
    protected abstract fun Context.getOutput(): T

    final override fun Context.process() {
        output.set = getOutput()
    }
}

/**
 * A block that contains within itself another block sub-system, configured in [configSubsystem]
 *
 * When this block is processed it will process the entire sub-system.
 *
 * The subsystem does not support [SpecialBlock]s.
 */
abstract class CompositeBlock(processing: Processing) : Block(processing) {

    private lateinit var subsystem: Subsystem
    private var outsideContext: Context? = null
    final override fun init() {
        subsystem.init()
    }

    final override fun Context.process() {
        subsystem.process(this)
    }

    final override fun stop() {
        subsystem.stop()
    }

    /**
     * Builds and returns a [BlockArrangement] for the subsystem, using the given a [SubsystemMapper] to map _this_
     * blocks inputs/outputs into the _subsystem's_ outputs.inputs.
     *
     * Try not to confuse inputs/outputs as that may result in errors.
     */
    protected abstract fun SubsystemMapper.configSubsystem(builder: BlockArrangementBuilder)

    final override fun finalizeConfig() {
        super.finalizeConfig()
        val builder = BlockArrangementBuilder()
        val mapper = object : SubsystemMapper {
            val outputs = Outputs()
            val sources = Array(numInputs) { Source(inputs[it]) }
            val usedOutputs = BooleanArray(numOutputs)
            override val <T> Input<T>.subOutput: Output<T>
                get() {
                    require(block === this@CompositeBlock) {
                        throw IllegalBlockConfigurationException(
                            "In composite block ${this@CompositeBlock}: " +
                                    "Attempted to map from another block: $this"
                        )
                    }
                    return sources[index].subOutput.uncheckedCast()
                }

            override val <T> Output<T>.subInput: Input<T>
                get() {
                    require(block === this@CompositeBlock) {
                        throw IllegalBlockConfigurationException(
                            "In composite block ${this@CompositeBlock}: " +
                                    "Attempted to map from another block: $this"
                        )
                    }
                    usedOutputs[index] = true
                    return outputs.subInputs[index].uncheckedCast()
                }
        }
        builder.add(mapper.outputs)

        mapper.configSubsystem(builder)
        subsystem = Subsystem(builder.build())
    }

    /**
     * Maps a [CompositeBlock]'s inputs/outputs into the subsystem's outputs.inputs.
     */
    protected interface SubsystemMapper {

        /**
         * Gets the _subsystem_ output corresponding _this block's_ input.
         */
        val <T> Input<T>.subOutput: Output<T>

        /**
         * Gets the _subsystem_ input corresponding to _this block's_ output.
         */
        val <T> Output<T>.subInput: Input<T>
    }

    private inner class Subsystem(arrangement: BlockArrangement) : BlockRunner(arrangement) {

        override val systemValues: SystemValues get() = outsideContext!!
        public override fun init() = super.init()
        public override fun stop() = super.stop()
        fun process(context: Context) {
            outsideContext = context
            processOnce()
        }
    }

    private inner class Source<T>(private val outsideInput: Input<T>) : Block(LAZY) {
        val subOutput = Output<T>("Mapped " + outsideInput.name, outsideInput.type)

        override fun Context.process() {
            subOutput.set = outsideContext!![outsideInput]
        }
    }

    private inner class Outputs : Block(ALWAYS) {

        val subInputs = Array(this@CompositeBlock.numOutputs) {
            Input<Any?>("Mapped " + this@CompositeBlock.outputs[it].name, this@CompositeBlock.outputs[it].type, false)
        }

        override fun Context.process() {
            repeat(subInputs.size) {
                outsideContext!![this@CompositeBlock.outputs[it].uncheckedCast<Output<Any?>>()] = subInputs[it].get
            }
        }
    }
}
