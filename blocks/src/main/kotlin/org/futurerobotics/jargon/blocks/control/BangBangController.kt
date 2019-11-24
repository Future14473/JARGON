package org.futurerobotics.jargon.blocks.control

import org.futurerobotics.jargon.blocks.PrincipalOutputBlock

/**
 * A [Controller], which only outputs three different values depending on if the state is less than,
 * equal to, or greater than the reference.
 *
 * @param lessThanOutput the signal to output if the current state is less than the reference
 * @param greaterThanOutput the signal to output if the current state is greater than the reference
 * @param equalOutput the signal to output if the current state is equal to the reference
 */
class BangBangController<State, Signal>(
    private val lessThanOutput: Signal, private val greaterThanOutput: Signal, private val equalOutput: Signal
) : PrincipalOutputBlock<Signal>(Processing.LAZY),
    Controller<State, State, Signal> where State : Comparable<State> {

    override val reference: Input<State> = newInput()
    override val state: Input<State> = newInput()
    override val signal: Output<Signal> get() = super.output

    override fun init() {}

    override fun stop() {}

    override fun Context.getOutput(): Signal {
        val comp = state.get.compareTo(reference.get)
        return when {
            comp < 0 -> lessThanOutput
            comp > 0 -> greaterThanOutput
            else -> equalOutput
        }
    }
}
