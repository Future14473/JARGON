@file:Suppress("UNCHECKED_CAST")

package org.futurerobotics.jargon.blocks

import org.futurerobotics.jargon.math.Pose2d
import org.futurerobotics.jargon.mechanics.FixedDriveModel
import org.futurerobotics.jargon.mechanics.MotionOnly


/**
 * A controller component that only outputs up to three values depending if the current state is less than, equal to,
 * or greater than the reference.
 *
 * Inputs:
 *  1. the reference state, should not be null
 *  2. the current state, should not be null
 *
 * Outputs:
 *  1. the signal
 *
 * @param lessThanOutput the signal to output if the current state is less than the reference
 * @param greaterThanOutput the signal to output if the current state is greater than the reference
 * @param equalOutput the signal to output if the current state is equal to the reference
 */
class BangBangController<State, Signal>(
    private val lessThanOutput: Signal,
    private val greaterThanOutput: Signal,
    private val equalOutput: Signal
) : Combine<State, State, Signal>() where State : Comparable<State>, State : Any {

    override fun combine(a: State, b: State): Signal {
        val comp = b.compareTo(a)
        return when {
            comp < 0 -> lessThanOutput
            comp > 0 -> greaterThanOutput
            else -> equalOutput
        }
    }


    /** The reference [BlockInput] */
    val reference: BlockInput<State> get() = first
    /** The state [BlockInput] */
    val state: BlockInput<State> get() = second
    /** the signal [BlockOutput] */
    val signal: BlockOutput<Signal> = this

}

/**
 * A open-loop controller for a [FixedDriveModel], that takes the [MotionOnly] of Pose as references,
 * and produces the modeled motor voltages as a list of doubles, using the [model].
 *
 *Inputs:
 *  1. the current pose [MotionOnly]
 *
 * Outputs
 *  1. the modeled motor voltages as a [List] of Doubles
 */
class FixedDriveOpenController(private val model: FixedDriveModel) :
    Pipe<MotionOnly<Pose2d>, List<Double>>() {

    override fun pipe(input: MotionOnly<Pose2d>): List<Double> = model.getModeledVoltages(input)

}
