package org.futurerobotics.jargon.math

/**
 * Represents information about value, first, and second derivatives of a numerical value.
 */
interface Derivatives<T> {

    /** The value */
    val value: T
    /** The value's derivative */
    val deriv: T
    /** The value's second derivative */
    val secondDeriv: T

    /** [value] */
    @JvmDefault
    operator fun component1(): T = value

    /** [deriv] */
    @JvmDefault
    operator fun component2(): T = deriv

    /** [secondDeriv] */
    @JvmDefault
    operator fun component3(): T = secondDeriv
}

/** A simple [Derivatives] implementation that holds values in fields */
class ValueDerivatives<T>(override val value: T, override val deriv: T, override val secondDeriv: T) : Derivatives<T>

