package org.futurerobotics.jargon.blocks.functional

import org.futurerobotics.jargon.blocks.Block.Processing.LAZY
import org.futurerobotics.jargon.blocks.PipeBlock
import org.futurerobotics.jargon.math.MotionOnly
import org.futurerobotics.jargon.math.MotionState

/**
 * A block that is inputted [MotionOnly]; and pipes it by "shifting" the velocity and acceleration
 * into position and velocity, respectively, into a velocity [MotionState].
 *
 * It needs a [zero] value to fill in the empty acceleration of the outputted [MotionState].
 */
class MotionOnlyToVelocityState<T : Any>(private val zero: T) : PipeBlock<MotionOnly<T>, MotionState<T>>(LAZY) {

    override fun Context.pipe(
        input: MotionOnly<T>
    ): MotionState<T> = MotionState(
        input.deriv,
        input.secondDeriv,
        zero
    )
}

/**
 * A [PipeBlock] that maps each component of a [MotionState] through a [mapping] function.
 */
class MapMotionState<T : Any, R : Any>(
    private val mapping: (T) -> R
) : PipeBlock<MotionState<T>, MotionState<R>>(LAZY) {

    override fun Context.pipe(
        input: MotionState<T>
    ): MotionState<R> = input.map(mapping)
}

/**
 * A [PipeBlock] that maps each component of a [MotionOnly] through a [mapping] function.
 */
class MapMotionOnly<T : Any, R : Any>(
    private val mapping: (T) -> R
) : PipeBlock<MotionOnly<T>, MotionOnly<R>>(LAZY) {

    override fun Context.pipe(
        input: MotionOnly<T>
    ): MotionOnly<R> = input.map(mapping)
}
