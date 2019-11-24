package org.futurerobotics.jargon.blocks.functional

import org.futurerobotics.jargon.blocks.Block.Processing.LAZY
import org.futurerobotics.jargon.blocks.PipeBlock
import org.futurerobotics.jargon.blocks.functional.MapMotionOnly.Companion.with
import org.futurerobotics.jargon.blocks.functional.MapMotionState.Companion.with
import org.futurerobotics.jargon.math.MotionOnly
import org.futurerobotics.jargon.math.MotionState
import org.futurerobotics.jargon.math.ValueMotionOnly
import org.futurerobotics.jargon.math.ValueMotionState

/**
 * A block that is inputted [MotionOnly]; and pipes it by "shifting" the velocity and acceleration
 * into position and velocity, respectively, into a velocity [MotionState].
 *
 * It needs a [zero] value to fill in the empty acceleration of the outputted [MotionState].
 */
class MotionOnlyToVelocityState<T : Any>(private val zero: T) : PipeBlock<MotionOnly<T>, MotionState<T>>(LAZY) {

    override fun Context.pipe(
        input: MotionOnly<T>
    ): MotionState<T> = object : MotionState<T> {
        override val value: T
            get() = input.vel
        override val deriv: T
            get() = input.accel
        override val secondDeriv: T = zero
    }
}

/**
 * A [PipeBlock] that maps each component of a [MotionState] through a [map] function.
 *
 * Can be created by subclassing or by using [with] that takes a lambda.
 */
abstract class MapMotionState<T : Any, R : Any> : PipeBlock<MotionState<T>, MotionState<R>>(LAZY) {

    override fun Context.pipe(
        input: MotionState<T>
    ): MotionState<R> =
        ValueMotionState(
            map(input.value),
            map(input.deriv),
            map(input.secondDeriv)
        )

    /** Maps the [value] into a new value; run on all components of a [MotionState]. */
    protected abstract fun map(value: T): R

    companion object {
        /** Creates a [MapMotionState] that uses the given [mapping] function. */
        inline fun <T : Any, R : Any> with(crossinline mapping: (T) -> R): MapMotionState<T, R> =
            object : MapMotionState<T, R>() {
                override fun map(value: T): R = mapping(value)
            }
    }
}

/**
 * A [PipeBlock] that maps each component of a [MotionOnly] through a [map] function.
 *
 * Can be created by subclassing or by using function/method [with] that takes a lambda.
 */
abstract class MapMotionOnly<T : Any, R : Any> : PipeBlock<MotionOnly<T>, MotionOnly<R>>(LAZY) {

    override fun Context.pipe(
        input: MotionOnly<T>
    ): MotionOnly<R> =
        ValueMotionOnly(map(input.vel), map(input.accel))

    /** Maps the [value] into a new value; run on all components of a [MotionOnly]. */
    protected abstract fun map(value: T): R

    companion object {
        /** Creates a [MapMotionOnly] that uses the given [mapping] function. */
        inline fun <T : Any, R : Any> with(crossinline mapping: (T) -> R): MapMotionOnly<T, R> =
            object : MapMotionOnly<T, R>() {
                override fun map(value: T): R = mapping(value)
            }
    }
}
