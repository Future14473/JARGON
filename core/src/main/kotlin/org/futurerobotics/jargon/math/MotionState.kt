package org.futurerobotics.jargon.math

/**
 * Generic representation of just the motion of some quantity of type [T], meaning [deriv]ocity and [secondDeriv]eration.
 * @see MotionState
 * @see MotionOnly
 * @see RealMotionOnly
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
 * @see RealMotionState
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
 * @see RealMotionOnly
 */
data class MotionOnly<T>(
    override val deriv: T, override val secondDeriv: T
) : AnyMotionOnly<T> {

    /** The velocity, a.k.a [deriv] */
    inline val velocity: T get() = deriv
    /** The velocity, a.k.a [deriv] */
    inline val v: T get() = deriv
    /** The acceleration, a.k.a [deriv] */
    inline val acceleration: T get() = deriv
    /** The acceleration, a.k.a [deriv] */
    inline val a: T get() = deriv

    /**
     * Maps all elements of this motion only through the given [transform] function.
     */
    inline fun <R> map(transform: (T) -> R): MotionOnly<R> = MotionOnly(
        transform(deriv),
        transform(secondDeriv)
    )

    /**
     * Maps all elements of this motion only through the given [transform] function,
     * returning a [RealMotionOnly].
     */
    @JvmName("mapToReal")
    inline fun map(transform: (T) -> Double): RealMotionOnly = RealMotionOnly(
        transform(deriv),
        transform(secondDeriv)
    )

    /**
     * Converts this motion only into a velocity motion state, where the value of the returned
     * motion state is this's velocity, and the deriv of the motion state is this's acceleration.
     *
     * A [secondDeriv] still needs to be supplied to fill in values.
     */
    fun toVelocityMotionState(secondDeriv: T): MotionState<T> =
        MotionState(deriv, this.secondDeriv, secondDeriv)
}

/**
 * Representation of the motion state of some quantity of type [T], meaning [value], [deriv], and [secondDeriv].
 * @see MotionState
 * @see MotionOnly
 * @see RealMotionState
 */
data class MotionState<T>(
    override val value: T, override val deriv: T, override val secondDeriv: T
) : AnyMotionState<T> {

    /** The velocity, [deriv] */
    inline val velocity: T get() = deriv
    /** The velocity, [deriv] */
    inline val v: T get() = deriv
    /** The acceleration, [secondDeriv] */
    inline val acceleration: T get() = deriv
    /** The acceleration, [secondDeriv] */
    inline val a: T get() = deriv

    /** Creates a [MotionOnly] with same [deriv] and [secondDeriv] as this [MotionState]. */
    fun toMotionOnly(): MotionOnly<T> = MotionOnly(deriv, secondDeriv)

    /**
     * Maps all elements of this motion state through the given [transform] function.
     */
    inline fun <R> map(transform: (T) -> R): MotionState<R> = MotionState(
        transform(value),
        transform(deriv),
        transform(secondDeriv)
    )

    /**
     * Maps all elements of this motion state through the given [transform] function,
     * returning a [RealMotionState].
     */
    @JvmName("mapToReal")
    inline fun map(transform: (T) -> Double): RealMotionState = RealMotionState(
        transform(value),
        transform(deriv),
        transform(secondDeriv)
    )

    companion object {
        /** Creates a [MotionState] with all values equal to the given [value] */
        @JvmStatic
        fun <T> ofAll(value: T): MotionState<T> =
            MotionState(value, value, value)
    }
}
