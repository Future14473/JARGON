package org.futurerobotics.jargon.blocks

/**
 * # Blocks and Blocks SYSTEMS
 * This system was inspired by making block diagrams more literal.
 *
 * The root of the blocks system is the [Block]. A [Block] can represent anything with a notion of _inputs_
 * or _outputs_: any process, calculation, value, measurement, interaction, etc. These can then be connected
 * together within a [BlocksConfig], then run in a [BlocksSystem].
 *
 * A [Block] can have any number of _input_ and _outputs_, including 0, defined by [numInputs] and
 * [numOutputs]. Each input and output is then associated with a 0-based index, and this index is used everywhere
 * inputs/outputs are given.
 *
 *
 * ## Running blocks and Block Systems.
 *
 * After a [BlocksSystem] is configured and built, (see "Configuring blocks" below), blocks
 * can be run in a series of _loops_. In every loop, blocks will (maybe) be processed and have inputs and
 * values transferred accordingly. Generally:
 *
 * - [init] is called when the entire system first starts
 * - [process] will (maybe) be called with the block's inputs in a list, also special values provided by
 *   [SystemValues] will be given
 * - [getOutput] will then (maybe) called to extract the output values by index.
 *
 * The exact way a block is processed is defined by its [processing]; see there for more details.
 *
 * Subclasses should explain the available inputs and outputs and the block's behavior.
 *
 * ## Configuring blocks
 *
 * [BlocksConfig] provides a java-usable kotlin DSL (Domain Specific Language) for connecting blocks.
 * Perhaps a GUI version of this will be available in the Future.
 *
 * Every input/output is usually associated with a specific class/type. Since there is no support for variable generics and
 * can only check at runtime. However, we have [BlocksConfig.Input] and [BlocksConfig.Output] with generics to _assist_
 * with type checking at compile time.
 *
 * **Subclasses should provide methods for retrieving [BlocksConfig.Input]/[BlocksConfig.Output]s for configuration**,
 * one for each input/output index. Then, multiple blocks can have their inputs and outputs connected within a
 * [BlocksConfig] to produce a [BlocksSystem] or similar construct.
 *
 * General rules for providing configuration are as follows:
 * 1. Generally, blocks should provide a way of get a [BlocksConfig.Input] or [BlocksConfig.Output] for
 *    _every_ input and output.
 * 2. The generics of the blocks config should match exactly what is expected.
 * 3. A block with _exactly 1_ input/output is allowed to implement the [BlocksConfig.Input] or [BlocksConfig.Output]
 *   interfaces directly.
 *
 * Blocks can then later [prepareAndVerify] themselves to check if configuration is done properly.
 *
 * ## Other
 *
 * There are also [SpecialBlock]s which receive special treatment from [BlocksSystem].
 *
 * ## ***See the following for common implementations of blocks to make your life easier:***
 * - [AbstractBlock]; most blocks extend this.
 * - [SingleOutputBlock] for a block with a single output; slightly more strongly typed.
 * - [SingleInputBlock] for a block with a single output; slightly more strongly typed.
 * - [ListStoreBlock] for blocks that store all their outputs upon [process] (have no need for lazy getOutput)
 * - [SingleInputListStoreBlock] which is a combination of the two above
 * - [PipeBlock] for one-input, one-output blocks. Quick inline lambda versions of this are also available in
 *   [BlocksConfig.pipe]
 * - [CombineBlock] for two-input, one-output blocks. Quick inline lambda versions of this are also available in
 *   [BlocksConfig.combine]
 * - [CompositeBlock] that is a block made up of an entire sub-system of blocks.
 */
interface Block {

    /** The number of inputs to this block;*/
    val numInputs: Int
    /** The number of outputs to this block */
    val numOutputs: Int
    /** The processing policy of this block. See [Processing] */
    val processing: Processing

    /**
     * Defines how this component is run, which can be:
     *
     * - [IN_FIRST_LAZY]
     * - [IN_FIRST_ALWAYS]
     * - [OUT_FIRST_ALWAYS]
     *
     * These options allow for more dynamic behavior, as blocks may not be run every loop.
     *
     * At least one block in a component system must be _not_ [IN_FIRST_LAZY] since if everyone is lazy nobody
     * will process.
     *
     * @property isAlwaysProcess if this [Processing] is an _always process_.
     * @property isOutFirst if this [Processing] is OutFirst.
     */
    enum class Processing(val isAlwaysProcess: Boolean, val isOutFirst: Boolean) {

        /**
         * A block with [IN_FIRST_LAZY] processing will only [process] or poll [getOutput] if another block requests its
         * outputs, otherwise it may not process.
         *
         * Blocks that do nothing more than process their inputs directly into output without storing information
         * of any kind should have this processing.
         * @see [Processing]
         */
        IN_FIRST_LAZY(false, false),
        /**
         * A block with [IN_FIRST_ALWAYS] processing will always be [process]ed every loop, and inputs must be given
         * before outputs are extracted. However, [getOutput] will still only be called if necessary.
         *
         * Blocks that require that it receives information every single loop should have this kind of processing.
         * @see [Processing]
         */
        IN_FIRST_ALWAYS(true, false),
        /**
         * A block with [OUT_FIRST_ALWAYS] processing will have its outputs polled _before_ [process] is called, every
         * loop, and [process] is only called at the very end of every loop.
         * **A block with [OUT_FIRST_ALWAYS] will always request _every single input, every loop_**. This is (one of)
         * the only ways to prevent values for next cycle being given out instead.
         *
         * At least one block in a loop of blocks must be [OUT_FIRST_ALWAYS]; For example a block that directly
         * interacts with hardware or external sources to be this kind of processing since measurements (outputs) are
         * usually taken _before_ signal (inputs)
         * @see [Processing]
         */
        OUT_FIRST_ALWAYS(true, true);
        //There is no OUT_FIRST_LAZY since that causes problems and is rarely needed.
    }

    /**
     * Does any possible necessary preparation for this block to actually run from the given [config]. Also, verifies
     * that the current configuration on the given [BlocksConfig] is valid (for example, all required inputs are
     * connected); if not, throws an [IllegalBlockConfigurationException].
     */
    fun prepareAndVerify(config: BlocksConfig)

    /**
     * Resets and initializes this block; Called when the _entire_ system first starts.
     */
    fun init()

    /**
     * Processes this block to prepare for outputs. Only called at most once per loop, and if this block is
     * _not_ [Processing.OUT_FIRST_ALWAYS], before any [getOutput] calls.
     *
     * It may be possible for [process] to be called but not [getOutput] if this block is [Processing.IN_FIRST_ALWAYS]
     *
     * @param inputs the current inputs to the block. A block is allowed to store the inputs but should not assume
     * that the inputs list will stay the same every loop as it might not be.
     */
    fun process(inputs: List<Any?>, systemValues: SystemValues)

    /**
     * Gets an output of this block by [index]. A given output index will only be requested at most once per loop.
     *
     * If this block is _not_ [Processing.OUT_FIRST_ALWAYS]:
     * - [process] will be called before any outputs are retrieved, per cycle.
     * - [getOutput] will _only_ be called if another block requests its outputs.
     */
    fun getOutput(index: Int): Any?

    /**
     * Stops this block and does nessecary cleanup; called when the _entire_ system shuts down.
     */
    fun stop()
}
