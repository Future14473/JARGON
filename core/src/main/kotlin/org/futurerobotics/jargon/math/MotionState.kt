package org.futurerobotics.jargon.math

/**
 * Generic representation of just the motion of some quantity of type [T], meaning [deriv]ocity and [secondDeriv]eration.
 * @see MotionState
 * @see MotionOnly
 * @see LinearMotionOnly
 */
interface AnyMotionOnly<out T> {

    /** The velocity of this [MotionOnly] */
    val deriv: T
    /** The acceleration of this [MotionOnly] */
    val secondDeriv: T
}

/**
 * Generic representation of the motion state of some quantity of type [T], meaning [value], [deriv], and [secondDeriv].
 * @see MotionState
 * @see MotionOnly
 * @see LinearMotionState
 */
interface AnyMotionState<out T> {

    /** The value of this [MotionState] */
    val value: T
    /** The velocity of this [MotionOnly] */
    val deriv: T
    /** The acceleration of this [MotionState] */
    val secondDeriv: T
}

/**
 * Represents just the motion of some quantity of type [T], meaning [deriv]ocity and [secondDeriv]eration.
 * @see MotionState
 * @see MotionOnly
 * @see LinearMotionOnly
 */
data class MotionOnly<out T>(
    override val deriv: T, override val secondDeriv: T
) : AnyMotionOnly<T>

/**
 * Representation of the motion state of some quantity of type [T], meaning [value], [deriv], and [secondDeriv].
 * @see MotionState
 * @see MotionOnly
 * @see LinearMotionState
 */
data class MotionState<out T>(
    override val value: T, override val deriv: T, override val secondDeriv: T
) : AnyMotionState<T> {

    /** Creates a [MotionOnly] with same [deriv] and [secondDeriv] as this [MotionState]. */
    fun toMotionOnly(): MotionOnly<T> = MotionOnly(deriv, secondDeriv)

    inline fun <R : Any> map(transform: (T) -> R): MotionState<R> = MotionState(
        transform(value),
        transform(deriv),
        transform(secondDeriv)
    )

    companion object {
        /** Creates a [MotionState] with all values equal to the given [value] */
        @JvmStatic
        fun <T : Any> ofAll(value: T): MotionState<T> =
            MotionState(value, value, value)
    }
}
