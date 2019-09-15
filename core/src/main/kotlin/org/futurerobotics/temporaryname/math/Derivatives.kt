package org.futurerobotics.temporaryname.math

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
}

/** @return value */
operator fun <T> Derivatives<T>.component1(): T = value

/** @return deriv */
operator fun <T> Derivatives<T>.component2(): T = deriv

/** @return secondDeriv */
operator fun <T> Derivatives<T>.component3(): T = secondDeriv

/** A simple [Derivatives] implementation that holds values in fields */
class ValueDerivatives<T>(override val value: T, override val deriv: T, override val secondDeriv: T) : Derivatives<T>

